/* Detector and sweep control. */

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include "error.h"
#include "hardware.h"
#include "epics_device.h"
#include "epics_extra.h"
#include "sequencer.h"
#include "numeric.h"

#include "detector.h"



#define TUNE_LENGTH    (BUF_DATA_LENGTH / 4)


/* Detector bunches: one bunch for each operating channel. */
static unsigned int detector_bunches[4];

/* The tune scale is used both for presentation and as part of the tune
 * calculation, so we hang onto it here. */
static float tune_scale[TUNE_LENGTH];
static int scale_length;
/* Each time a sequencer setting changes we'll need to recompute the scale. */
static bool tune_scale_needs_refresh = true;
static struct epics_record *tune_scale_pv;


/* Information about a single channel of sweep measurement. */
struct sweep_info {
    short wf_i[TUNE_LENGTH];
    short wf_q[TUNE_LENGTH];
    int power[TUNE_LENGTH];
    double power_tune;
    double power_phase;
    int cumsum_i[TUNE_LENGTH];
    int cumsum_q[TUNE_LENGTH];
    double cumsum_tune;
    double cumsum_phase;
};

/* We have five channels of sweep measurement: one for each channel plus an
 * aggregate across all four channels. */
static struct sweep_info channel_sweep[4];
static struct sweep_info mean_sweep;
/* Used to trigger update of all sweep info. */
static struct epics_interlock *iq_trigger;


/* Waveforms for compensating IQ by group_delay.
 *
 * The DDC skew is a group delay in ns.  We translate this into a phase
 * advance of
 *
 *      phase = 0.5 * skew * 2pi * frequency / 936
 *
 * Here the frequency is in tunes, one revolution per turn, which we convert
 * into one revolution per bunch, or half a revolution per nanosecond (at
 * 500MHz RF frequency).
 *    For economy of calculation, we precompute the rotations as scaled
 * integers so that the final computation can be a simple integer
 * multiplication (one instruction when the compiler is in the right mood). */
static int group_delay = 0;
static int rotate_I[TUNE_LENGTH];  // 2**30 * cos(phase)
static int rotate_Q[TUNE_LENGTH];  // 2**30 * sin(phase)

/* Helper constants for fast tune scale and rotation waveform calculations
 * corresponding to multiplication by 936*2^-32. */
static uint32_t wf_scaling;
static int wf_shift;



unsigned int tune_to_freq(double tune)
{
    return (unsigned int) round(tune * (pow(2, 32) / (double) MAX_BUNCH_COUNT));
}


static void store_one_tune_freq(unsigned int freq, int ix)
{
    unsigned_fixed_to_single(freq, &tune_scale[ix], wf_scaling, wf_shift);
    cos_sin(-freq * group_delay, &rotate_I[ix], &rotate_Q[ix]);
}

/* Computes frequency scale directly from sequencer settings.  Triggered
 * whenever the sequencer state changes. */
static void update_det_scale(void)
{
    unsigned int state;
    const struct seq_entry *sequencer_table = read_sequencer_table(&state);

    int ix = 0;
    unsigned int f0 = 0;
    for ( ; state > 0  &&  ix < TUNE_LENGTH; state --)
    {
        const struct seq_entry *entry = &sequencer_table[state];
        f0 = entry->start_freq;
        for (unsigned int i = 0;
             i < entry->capture_count + 1  &&  ix < TUNE_LENGTH; i ++)
        {
            store_one_tune_freq(f0, ix++);
            f0 += entry->delta_freq;
        }
    }

    /* Pad the rest of the scale.  The last frequency is a good a choice as any,
     * anything that goes here is invalid. */
    scale_length = ix;
    while (ix < TUNE_LENGTH)
        store_one_tune_freq(f0, ix++);

    tune_scale_needs_refresh = false;
}


void seq_settings_changed(void)
{
    tune_scale_needs_refresh = true;
}


static void set_group_delay(int delay)
{
    group_delay = delay;
    seq_settings_changed(); // Not really true, but does enough to get by
}


/* IQ values are packed with four channels of data for each sample.  Here we
 * extract the four channels, compute an average channel and compensate for
 * group delay. */
static void extract_iq(const short buffer_low[], const short buffer_high[])
{
    for (int i = 0; i < scale_length; i ++)
    {
        int rot_I = rotate_I[i];
        int rot_Q = rotate_Q[i];
        int I_sum = 0, Q_sum = 0;
        for (int channel = 0; channel < 4; channel ++)
        {
            int raw_I = buffer_high[4 * i + channel];
            int raw_Q = buffer_low[4 * i + channel];
            int I = MulSS(raw_I, rot_I) - MulSS(raw_Q, rot_Q);
            int Q = MulSS(raw_I, rot_Q) + MulSS(raw_Q, rot_I);
            I_sum += I;
            Q_sum += Q;

            struct sweep_info *sweep = &channel_sweep[channel];
            sweep->wf_i[i] = I;
            sweep->wf_q[i] = Q;
        }
        mean_sweep.wf_i[i] = I_sum;
        mean_sweep.wf_q[i] = Q_sum;
    }

    /* Pad rest of results with zeros: none of this data is valid! */
    for (int i = scale_length; i < TUNE_LENGTH; i ++)
    {
        for (int channel = 0; channel < 4; channel ++)
        {
            struct sweep_info *sweep = &channel_sweep[channel];
            sweep->wf_i[i] = 0;
            sweep->wf_q[i] = 0;
        }
        mean_sweep.wf_i[i] = 0;
        mean_sweep.wf_q[i] = 0;
    }
}


/* Converts a tune, detected as an index into the tune sweep waveform, into the
 * corresponding tune frequency offset and phase. */
static void index_to_tune(
    struct sweep_info *sweep, int ix, double *tune, double *phase)
{
    double harmonic;
    *tune = modf(tune_scale[ix], &harmonic);
    *phase = 180.0 / M_PI * atan2(sweep->wf_i[ix], sweep->wf_q[ix]);
}


#define SQR(x)      ((x) * (x))


/* Computes tune from maximum power.  Somewhat crude but can give quite good
 * results. */
static void compute_power_tune(struct sweep_info *sweep)
{
    /* Simple power computation. */
    for (int i = 0; i < TUNE_LENGTH; i ++)
        sweep->power[i] = SQR(sweep->wf_i[i]) + SQR(sweep->wf_q[i]);

    /* Take the peak power as the reported tune. */
    int peak_value = 0;
    int peak_index = 0;
    for (int i = 0; i < TUNE_LENGTH; i ++)
    {
        if (sweep->power[i] > peak_value)
        {
            peak_value = sweep->power[i];
            peak_index = i;
        }
    }

    /* Convert detected peak offset into tune frequency and phase. */
    index_to_tune(sweep, peak_index, &sweep->power_tune, &sweep->power_phase);
}


/* Computes tune from cumulative sum. */
static void compute_cumsum_tune(struct sweep_info *sweep)
{
    /* Simple cumulative sum. */
    int sum_i = 0, sum_q = 0;
    for (int i = 0; i < TUNE_LENGTH; i ++)
    {
        sum_i += sweep->wf_i[i];
        sum_q += sweep->wf_q[i];
        sweep->cumsum_i[i] = sum_i;
        sweep->cumsum_q[i] = sum_q;
    }

    /* Search for peak absolute magnitude. */
    int peak_value = 0;
    int64_t peak_index = 0;
    for (int i = 0; i < TUNE_LENGTH; i ++)
    {
        int64_t value =
            SQR((int64_t) sweep->cumsum_i[i]) +
            SQR((int64_t) sweep->cumsum_q[i]);
        if (value > peak_value)
        {
            peak_value = value;
            peak_index = i;
        }
    }

    index_to_tune(sweep, peak_index, &sweep->cumsum_tune, &sweep->cumsum_phase);
}


/* Once I/Q waveforms have been computed we can perform final processing on all
 * five channels independently.  The power spectrum and cumulative sum is
 * generated and tune measurements made. */
static void compute_sweep(struct sweep_info *sweep)
{
    compute_power_tune(sweep);
    compute_cumsum_tune(sweep);
}


/* This is called when IQ data has been read into the fast buffer.  Process it
 * according to our current configuration into separate IQ wavforms.  One
 * separate I/Q value is extracted from each channel and rotated to compensate
 * for the precomputed group delay, and an average is also stored. */
void update_iq(const short buffer_low[], const short buffer_high[])
{
    if (tune_scale_needs_refresh)
    {
        update_det_scale();
        trigger_record(tune_scale_pv, 0, NULL);
    }

    interlock_wait(iq_trigger);
    extract_iq(buffer_low, buffer_high);
    for (int i = 0; i < 4; i ++)
        compute_sweep(&channel_sweep[i]);
    compute_sweep(&mean_sweep);
    interlock_signal(iq_trigger, NULL);
}


static void write_det_bunches(void)
{
    hw_write_det_bunches(detector_bunches);
}


static void write_nco_freq(double tune)
{
    hw_write_nco_freq(tune_to_freq(tune));
}


static void publish_channel(const char *name, struct sweep_info *sweep)
{
    char buffer[20];
#define FORMAT(field) \
    (sprintf(buffer, "DET:%s:%s", field, name), buffer)

    PUBLISH_WF_READ_VAR(short, FORMAT("I"), TUNE_LENGTH, sweep->wf_i);
    PUBLISH_WF_READ_VAR(short, FORMAT("Q"), TUNE_LENGTH, sweep->wf_q);

    PUBLISH_WF_READ_VAR(int, FORMAT("POWER"), TUNE_LENGTH, sweep->power);
    PUBLISH_READ_VAR(ai, FORMAT("PTUNE"), sweep->power_tune);
    PUBLISH_READ_VAR(ai, FORMAT("PPHASE"), sweep->power_phase);

    PUBLISH_WF_READ_VAR(int, FORMAT("CUMSUMI"), TUNE_LENGTH, sweep->cumsum_i);
    PUBLISH_WF_READ_VAR(int, FORMAT("CUMSUMQ"), TUNE_LENGTH, sweep->cumsum_q);
    PUBLISH_READ_VAR(ai, FORMAT("CTUNE"), sweep->cumsum_tune);
    PUBLISH_READ_VAR(ai, FORMAT("CPHASE"), sweep->cumsum_phase);

#undef FORMAT
}


static bool reset_window = true;

/* Compute the appropriate windowing function for the detector. */
static void write_detector_window(void *context, short *window, size_t *length)
{
    if (reset_window)
    {
        /* Let's use the Hamming window.  As this is only done once at startup
         * it doesn't matter if we take our time and use full floating point
         * arithmetic. */
        double a = 0.54;
        double b = 1 - a;
        for (int i = 0; i < DET_WINDOW_LENGTH; i ++)
            window[i] = (uint16_t) round(32767 *
                (a - b * cos(2 * M_PI * i / (DET_WINDOW_LENGTH - 1))));

        reset_window = false;
    }
    hw_write_det_window((uint16_t *) window);
    *length = DET_WINDOW_LENGTH;
}

static void reset_detector_window(void)
{
    reset_window = true;
}


bool initialise_detector(void)
{
    PUBLISH_WRITER_P(bo, "DET:MODE", hw_write_det_mode);
    PUBLISH_WRITER_P(mbbo, "DET:GAIN", hw_write_det_gain);
    PUBLISH_WRITER_P(mbbo, "DET:INPUT", hw_write_det_input_select);
    PUBLISH_WRITER_P(longout, "DET:SKEW", set_group_delay);

    for (int i = 0; i < 4; i ++)
    {
        char name[20];
        sprintf(name, "DET:BUNCH%d", i);
        PUBLISH_WRITE_VAR_P(ulongout, name, detector_bunches[i]);
        sprintf(name, "%d", i);
        publish_channel(name, &channel_sweep[i]);
    }
    publish_channel("M", &mean_sweep);
    iq_trigger = create_interlock("DET:TRIG", "DET:DONE", false);

    PUBLISH_ACTION("DET:WBUNCH", write_det_bunches);

    tune_scale_pv = PUBLISH_WF_READ_VAR_I(
        float, "DET:SCALE", TUNE_LENGTH, tune_scale);

    PUBLISH_WRITER_P(ao,   "NCO:FREQ", write_nco_freq);
    PUBLISH_WRITER_P(mbbo, "NCO:GAIN", hw_write_nco_gain);

    /* Initialise the scaling constants so that
     *  wf_scaling * 2^wf_shift = 936 * 2^-32. */
    compute_scaling(MAX_BUNCH_COUNT, &wf_scaling, &wf_shift);
    wf_shift -= 32;     // Divide by 2^32.

    /* Program the sequencer window. */
    PUBLISH_WAVEFORM(
        short, "DET:WINDOW", DET_WINDOW_LENGTH, write_detector_window);
    PUBLISH_ACTION("DET:RESET_WIN", reset_detector_window);

    return true;
}
