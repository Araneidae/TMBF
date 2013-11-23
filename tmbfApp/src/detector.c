/* Detector and sweep control. */

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
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
#include "tune.h"

#include "detector.h"




/* Detector bunches: one bunch for each operating channel. */
static unsigned int detector_bunches[4];

/* Selects single bunch mode if set to true. */
static bool detector_mode;

/* Each time a sequencer setting changes we'll need to recompute the scale. */
static bool tune_scale_needs_refresh = true;
static struct epics_record *tune_scale_pv;
static int linear_scale[TUNE_LENGTH];

/* Overflow status from the last sweep. */
static bool overflows[OVERFLOW_BIT_COUNT];

/* Gain and autogain control.  If autogain is enabled then we'll bump the
 * detector gain up or down where necessary and possible. */
static bool autogain_enable;
static unsigned int detector_gain;
static struct epics_record *gain_setting;
#define MAX_DET_GAIN    7       // 3 bit enumeration
/* The gain will be pushed up if the signal is below this threshold.  This has
 * to be a factor of at least 4 below the maximum signal of 32767, and there's a
 * further factor of around 3/4 to avoid premature switching and bouncing. */
#define GAIN_UP_THRESHOLD   6100

/* Completely analysed data from a successful detector sweep. */
static struct sweep_info sweep_info;
/* Used to trigger update of all sweep info. */
static struct epics_interlock *iq_trigger;


/* Waveforms for compensating IQ by loop_delay.
 *
 * The DDC skew is a group delay in ns.  We translate this into a phase
 * advance of
 *
 *      phase = 0.5 * skew * 2pi * frequency / BUNCHES_PER_TURN
 *
 * Here the frequency is in tunes, one revolution per turn, which we convert
 * into one revolution per bunch, or half a revolution per nanosecond (at
 * 500MHz RF frequency).
 *    For economy of calculation, we precompute the rotations as scaled
 * integers so that the final computation can be a simple integer
 * multiplication (one instruction when the compiler is in the right mood). */
static double adc_loop_delay;       // Base ADC loop delay as input by user
static double fir_loop_delay;       // Extra FIR loop delay
static int rotate_I[TUNE_LENGTH];   // 2**30 * cos(phase)
static int rotate_Q[TUNE_LENGTH];   // 2**30 * sin(phase)

/* Helper constants for fast tune scale and rotation waveform calculations
 * corresponding to multiplication by 936*2^-32. */
static uint32_t wf_scaling;
static int wf_shift;

static int compensated_delay;       // Compensated delay in bunches
static unsigned int detector_input; // FIR or ADC, affects compensated delay


/* This is called each time an IQ waveform has been captured.  If autogain is
 * enabled we bump the gain up or down as appropriate if necessary and possible.
 * A certain amount of hystersis is added to avoid bouncing. */
static void update_autogain(int abs_max)
{
    if (autogain_enable)
    {
        unsigned int new_gain = detector_gain;
        if (overflows[OVERFLOW_IQ_SCALE])
            /* Push gain down on overflow. */
            new_gain += 1;
        else if (abs_max < GAIN_UP_THRESHOLD)
            /* Push gain up if signal too low. */
            new_gain -= 1;

        /* Only write new gain if in range and changed.  Push the update
         * through the EPICS layer so the outside is fully informed. */
        if (new_gain <= MAX_DET_GAIN  &&  new_gain != detector_gain)
            WRITE_OUT_RECORD(mbbo, gain_setting, new_gain, true);
    }
}


/* Convert fractional tune in cycles per machine revolution to phase advance per
 * bunch in hardware units. */
unsigned int tune_to_freq(double tune)
{
    /* Convert the incoming tune in cycles per machine revolution into phase
     * advance per bunch by scaling and reducing to the half open interval
     * [0, 1). */
    double integral;
    double fraction = modf(tune / BUNCHES_PER_TURN, &integral);
    if (fraction < 0.0)
        fraction += 1.0;
    /* Can now scale up to hardware units. */
    return (unsigned int) round(fraction * pow(2, 32));
}


/* Computes compensated_delay (in bunches) from user entered loop delay (in
 * turns together with input specific delay. */
static void compute_delay(void)
{
    int adc_delay, fir_delay;
    hw_read_det_delays(&adc_delay, &fir_delay);
    double loop_delay = adc_loop_delay;
    if (!detector_input)
        /* If input is FIR then add on FIR loop delay. */
        loop_delay += fir_loop_delay;
    int compensation = detector_input ? adc_delay : fir_delay;
    compensated_delay = (int) round(
        BUNCHES_PER_TURN * loop_delay + compensation);
}

static void unsigned_fixed_to_double(
    uint32_t input, double *result, uint32_t scaling, int scaling_shift)
{
    *result = ldexp((double) input * scaling, scaling_shift);
}

static void store_one_tune_freq(unsigned int freq, int ix)
{
    unsigned_fixed_to_double(
        freq, &sweep_info.tune_scale[ix], wf_scaling, wf_shift);
    cos_sin(-freq * compensated_delay, &rotate_I[ix], &rotate_Q[ix]);
}

/* Computes frequency scale directly from sequencer settings.  Triggered
 * whenever the sequencer state changes. */
static void update_det_scale(void)
{
    unsigned int state;
    const struct seq_entry *sequencer_table = read_sequencer_table(&state);

    compute_delay();

    int ix = 0;
    unsigned int f0 = 0;
    for ( ; state > 0  &&  ix < TUNE_LENGTH; state --)
    {
        const struct seq_entry *entry = &sequencer_table[state];
        if (entry->write_enable)
        {
            f0 = entry->start_freq;
            for (unsigned int i = 0;
                 i < entry->capture_count + 1  &&  ix < TUNE_LENGTH; i ++)
            {
                store_one_tune_freq(f0, ix++);
                f0 += entry->delta_freq;
            }
        }
    }

    /* Record how many points will actually be captured. */
    sweep_info.sweep_length = ix;

    /* Pad the rest of the scale.  The last frequency is a good a choice as any,
     * anything that goes here is invalid. */
    while (ix < TUNE_LENGTH)
        store_one_tune_freq(f0, ix++);

    tune_scale_needs_refresh = false;
}


static void set_adc_loop_delay(double delay)
{
    adc_loop_delay = delay;
    tune_scale_needs_refresh = true;
}

static void set_fir_loop_delay(double delay)
{
    fir_loop_delay = delay;
    tune_scale_needs_refresh = true;
}

static void write_det_input_select(unsigned int input)
{
    detector_input = input;
    tune_scale_needs_refresh = true;
}


/* Returns 2^-31 * (a*c + b*d) with rounding of the last bit.  The scaling here
 * is chosen to balance the 2^30 scaling on (sin,cos) without risking overflow,
 * which unfortunately is possible if we scale by 2^-30. */
static int32_t dot_product(int32_t a, int32_t b, int32_t c, int32_t d)
{
    int64_t full = (int64_t) a * c + (int64_t) b * d;
    return (int32_t) ((full + (1UL << 30)) >> 31);
}


/* Extracts and scales IQ for one channel from the incoming raw IQ buffer.  Also
 * updates *abs_max for autogain calculation. */
static void extract_iq(
    const short buffer_low[], const short buffer_high[], int channel,
    struct channel_sweep *sweep, int *abs_max)
{
    for (int i = 0; i < sweep_info.sweep_length; i ++)
    {
        int raw_I = buffer_high[4 * i + channel];
        int raw_Q = buffer_low[4 * i + channel];
        if (abs(raw_I) > *abs_max)  *abs_max = abs(raw_I);
        if (abs(raw_Q) > *abs_max)  *abs_max = abs(raw_Q);

        int rot_I = rotate_I[i];
        int rot_Q = rotate_Q[i];
        sweep->wf_i[i] = dot_product(raw_I, raw_Q, rot_I, -rot_Q);
        sweep->wf_q[i] = dot_product(raw_I, raw_Q, rot_Q, rot_I);
    }

    /* Ensure the entire array is filled.  If IQ capture was short we extend the
     * last read value to fill the rest of the waveform.  This makes the
     * resulting display look better on a display tool like EDM. */
    for (int i = sweep_info.sweep_length; i < TUNE_LENGTH; i ++)
    {
        sweep->wf_i[i] = sweep->wf_i[sweep_info.sweep_length-1];
        sweep->wf_q[i] = sweep->wf_q[sweep_info.sweep_length-1];
    }
}


/* Once IQ has been computed for the four channels, the mean IQ is just the mean
 * of the four channels. */
static void compute_mean_iq(void)
{
    for (int i = 0; i < TUNE_LENGTH; i ++)
    {
        int I_sum = 0, Q_sum = 0;
        for (int channel = 0; channel < 4; channel ++)
        {
            struct channel_sweep *sweep = &sweep_info.channels[channel];
            I_sum += sweep->wf_i[i];
            Q_sum += sweep->wf_q[i];
        }
        sweep_info.mean.wf_i[i] = (I_sum + 2) / 4;  // Rounded average of four
        sweep_info.mean.wf_q[i] = (Q_sum + 2) / 4;
    }
}


/* The power waveform for each sweep is simply the sum of squares. */
#define SQR(x)      ((x) * (x))
static void compute_power(struct channel_sweep *sweep)
{
    for (int i = 0; i < TUNE_LENGTH; i ++)
        sweep->power[i] = SQR(sweep->wf_i[i]) + SQR(sweep->wf_q[i]);
}


/* From the raw buffer extract IQ data for each channel and update the computed
 * power waveform. */
static void update_sweep_info(
    const short buffer_low[], const short buffer_high[], int *abs_max)
{
    *abs_max = 0;
    for (int channel = 0; channel < 4; channel ++)
    {
        struct channel_sweep *sweep = &sweep_info.channels[channel];
        extract_iq(buffer_low, buffer_high, channel, sweep, abs_max);
        compute_power(sweep);
    }
    compute_mean_iq();
    compute_power(&sweep_info.mean);
}


/* Read out accumulated overflow bits over the last capture. */
static bool update_overflow(void)
{
    const bool read_mask[OVERFLOW_BIT_COUNT] = {
        [OVERFLOW_IQ_ACC] = true,
        [OVERFLOW_IQ_SCALE] = true,
    };
    hw_read_overflows(read_mask, overflows);
    return overflows[OVERFLOW_IQ_ACC] || overflows[OVERFLOW_IQ_SCALE];
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
    bool overflow = update_overflow();

    int abs_max;
    update_sweep_info(buffer_low, buffer_high, &abs_max);
    update_autogain(abs_max);
    sweep_info.single_bunch_mode = detector_mode;

    interlock_signal(iq_trigger, NULL);

    /* Perform tune sweep update outside trigger interlock so that the tune
     * layer can do its own separate update signalling. */
    update_tune_sweep(&sweep_info, overflow);
}


static void write_nco_freq(double tune)
{
    hw_write_nco_freq(tune_to_freq(tune));
}


void prepare_detector(bool settings_changed)
{
    /* We don't want the following states to change during a detector sweep, so
     * we write them now immediately before starting a fresh sweep. */
    hw_write_det_bunches(detector_bunches);
    hw_write_det_gain(detector_gain);
    hw_write_det_input_select(detector_input);
    hw_write_det_mode(detector_mode);
    if (settings_changed)
        tune_scale_needs_refresh = true;
}


static void publish_channel(const char *name, struct channel_sweep *sweep)
{
    char buffer[20];
#define FORMAT(field) \
    (sprintf(buffer, "DET:%s:%s", field, name), buffer)

    PUBLISH_WF_READ_VAR(short, FORMAT("I"), TUNE_LENGTH, sweep->wf_i);
    PUBLISH_WF_READ_VAR(short, FORMAT("Q"), TUNE_LENGTH, sweep->wf_q);
    PUBLISH_WF_READ_VAR(int, FORMAT("POWER"), TUNE_LENGTH, sweep->power);

#undef FORMAT
}


static bool reset_window = true;

/* Compute the appropriate windowing function for the detector.  If called after
 * reset_window has been set then the incoming window is replaced by a standard
 * window before being written to hardware. */
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
    gain_setting = PUBLISH_WRITE_VAR_P(mbbo, "DET:GAIN", detector_gain);
    PUBLISH_WRITE_VAR_P(bo, "DET:AUTOGAIN", autogain_enable);
    PUBLISH_WRITER_P(mbbo, "DET:INPUT", write_det_input_select);
    PUBLISH_WRITE_VAR_P(bo, "DET:MODE", detector_mode);
    PUBLISH_WRITER_P(ao, "DET:LOOP:ADC", set_adc_loop_delay);
    PUBLISH_WRITER_P(ao, "DET:LOOP:FIR", set_fir_loop_delay);

    PUBLISH_READ_VAR(bi, "DET:OVF:ACC", overflows[OVERFLOW_IQ_ACC]);
    PUBLISH_READ_VAR(bi, "DET:OVF:IQ",  overflows[OVERFLOW_IQ_SCALE]);

    for (int i = 0; i < 4; i ++)
    {
        char name[20];
        sprintf(name, "DET:BUNCH%d", i);
        PUBLISH_WRITE_VAR_P(ulongout, name, detector_bunches[i]);
        sprintf(name, "%d", i);
        publish_channel(name, &sweep_info.channels[i]);
    }
    publish_channel("M", &sweep_info.mean);
    iq_trigger = create_interlock("DET:TRIG", "DET:DONE", false);

    tune_scale_pv = PUBLISH_WF_READ_VAR_I(
        double, "DET:SCALE", TUNE_LENGTH, sweep_info.tune_scale);
    PUBLISH_WF_READ_VAR(int, "DET:LINEAR", TUNE_LENGTH, linear_scale);

    PUBLISH_WRITER_P(ao,   "NCO:FREQ", write_nco_freq);
    PUBLISH_WRITER_P(mbbo, "NCO:GAIN", hw_write_nco_gain);

    /* Initialise the scaling constants so that
     *  wf_scaling * 2^wf_shift = 936 * 2^-32. */
    compute_scaling(BUNCHES_PER_TURN, &wf_scaling, &wf_shift);
    wf_shift -= 32;     // Divide by 2^32.
    /* Initialise the linear scale. */
    for (int i = 0; i < TUNE_LENGTH; i ++)
        linear_scale[i] = i;

    /* Program the sequencer window. */
    PUBLISH_WAVEFORM(
        short, "DET:WINDOW", DET_WINDOW_LENGTH, write_detector_window);
    PUBLISH_ACTION("DET:RESET_WIN", reset_detector_window);

    return true;
}
