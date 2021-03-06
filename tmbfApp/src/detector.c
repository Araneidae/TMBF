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
#include "bunch_select.h"
#include "tmbf.h"

#include "detector.h"




/* Detector bunches: one bunch for each operating channel. */
static unsigned int detector_bunches[4];

/* Selects single bunch mode if set to true. */
static bool detector_mode;

/* Each time a sequencer setting changes we'll need to recompute the scale. */
static bool tune_scale_needs_refresh = true;
static double detector_delay;
static int timebase[TUNE_LENGTH];
static struct epics_interlock *tune_scale_trigger;

/* Overflow status from the last sweep. */
static bool overflows[PULSED_BIT_COUNT];

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
/* The minimum gain has a huge jump (8 bits) so we use a different threshold in
 * this case.  Shift by an extra 6 bits on top of the 2 bits already counted. */
#define MIN_GAIN_UP_THRESHOLD   (GAIN_UP_THRESHOLD >> 6)

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
static int rotate_I[TUNE_LENGTH];   // 2**30 * cos(phase)
static int rotate_Q[TUNE_LENGTH];   // 2**30 * sin(phase)

/* Helper constants for fast tune scale and rotation waveform calculations
 * corresponding to multiplication by BUNCHES_PER_TURN*2^-32. */
static uint32_t wf_scaling;
static int wf_shift;

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
        else if (detector_gain < MAX_DET_GAIN  &&  abs_max < GAIN_UP_THRESHOLD)
            /* Push gain up if signal too low. */
            new_gain -= 1;
        else if (detector_gain == MAX_DET_GAIN  &&
                 abs_max < MIN_GAIN_UP_THRESHOLD)
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


/* Computes compensated delay (in bunches) from user entered loop delay (in
 * turns together with input specific delay. */
static int compute_delay(void)
{
    int adc_delay, fir_delay;
    hw_read_det_delays(&adc_delay, &fir_delay);
    int delay = detector_input == DET_IN_ADC ? adc_delay : fir_delay;
    /* In ADC input mode we compensate for the overall configured loop delay
     * because what we want is a measurement of the machine in isolation.  In
     * FIR input mode, however, we're interested in the closed loop system
     * response so we ignore this. */
    if (detector_input == DET_IN_ADC)
        delay += lround(BUNCHES_PER_TURN * adc_loop_delay);
    return delay;
}

static void unsigned_fixed_to_double(
    uint32_t input, double *result, uint32_t scaling, int scaling_shift)
{
    *result = ldexp((double) input * scaling, scaling_shift);
}

static void store_one_tune_freq(int delay, unsigned int freq, unsigned int ix)
{
    unsigned_fixed_to_double(
        freq, &sweep_info.tune_scale[ix], wf_scaling, wf_shift);
    cos_sin(-(int) freq * delay, &rotate_I[ix], &rotate_Q[ix]);
}

/* Computes frequency scale directly from sequencer settings.  Triggered
 * whenever the sequencer state changes. */
static void update_det_scale(
    unsigned int state_count, const struct seq_entry *sequencer_table,
    unsigned int super_count, const uint32_t offsets[])
{
    int delay = compute_delay();
    detector_delay = (double) delay / BUNCHES_PER_TURN;

    unsigned int ix = 0;
    unsigned int total_time = 0;     // Accumulates captured timebase
    unsigned int gap_time = 0;       // Accumulates non captured time
    unsigned int f0 = 0;
    for (unsigned int super = 0;
         super < super_count  &&  ix < TUNE_LENGTH; super ++)
        for (unsigned int state = state_count;
             state > 0  &&  ix < TUNE_LENGTH; state --)
        {
            const struct seq_entry *entry = &sequencer_table[state - 1];
            unsigned int dwell_time = entry->dwell_time + entry->holdoff;
            if (entry->write_enable)
            {
                f0 = entry->start_freq + offsets[super];
                total_time += gap_time;
                gap_time = 0;
                for (unsigned int i = 0;
                     i < entry->capture_count  &&  ix < TUNE_LENGTH;
                     i ++, ix ++)
                {
                    store_one_tune_freq(delay, f0, ix);
                    f0 += entry->delta_freq;
                    total_time += dwell_time;
                    timebase[ix] = (int) total_time;
                }
            }
            else
                gap_time += dwell_time * entry->capture_count;
        }

    /* Record how many points will actually be captured. */
    sweep_info.sweep_length = ix;

    /* Pad the rest of the scale.  The last frequency is a good a choice as any,
     * anything that goes here is invalid. */
    for ( ; ix < TUNE_LENGTH; ix ++)
    {
        store_one_tune_freq(delay, f0, ix);
        timebase[ix] = (int) total_time;
    }

    tune_scale_needs_refresh = false;
}


static void set_adc_loop_delay(double delay)
{
    adc_loop_delay = delay;
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
    const short buffer_low[], const short buffer_high[], unsigned int channel,
    struct channel_sweep *sweep, int *abs_max)
{
    for (unsigned int i = 0; i < sweep_info.sweep_length; i ++)
    {
        int raw_I = buffer_low[4 * i + channel];
        int raw_Q = buffer_high[4 * i + channel];
        if (abs(raw_I) > *abs_max)  *abs_max = abs(raw_I);
        if (abs(raw_Q) > *abs_max)  *abs_max = abs(raw_Q);

        int rot_I = rotate_I[i];
        int rot_Q = rotate_Q[i];
        sweep->wf_i[i] = (short) dot_product(raw_I, raw_Q, rot_I, -rot_Q);
        sweep->wf_q[i] = (short) dot_product(raw_I, raw_Q, rot_Q, rot_I);
    }

    /* Ensure the entire array is filled.  If IQ capture was short we extend the
     * last read value to fill the rest of the waveform.  This makes the
     * resulting display look better on a display tool like EDM. */
    for (unsigned int i = sweep_info.sweep_length; i < TUNE_LENGTH; i ++)
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
        sweep_info.mean.wf_i[i] = (short) ((I_sum + 2) / 4);  // Rounded average
        sweep_info.mean.wf_q[i] = (short) ((Q_sum + 2) / 4);  // of 4 channels
    }
}


/* The power waveform for each sweep is simply the sum of squares. */
#define SQR(x)      ((x) * (x))
void compute_power(struct channel_sweep *sweep)
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
    for (unsigned int channel = 0; channel < 4; channel ++)
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
    const bool read_mask[PULSED_BIT_COUNT] = {
        [OVERFLOW_IQ_FIR] = true,
        [OVERFLOW_IQ_ACC] = true,
        [OVERFLOW_IQ_SCALE] = true,
    };
    hw_read_pulsed_bits(read_mask, overflows);
    return
        overflows[OVERFLOW_IQ_FIR] ||
        overflows[OVERFLOW_IQ_ACC] ||
        overflows[OVERFLOW_IQ_SCALE];
}


/* This is called when IQ data has been read into the fast buffer.  Process it
 * according to our current configuration into separate IQ wavforms.  One
 * separate I/Q value is extracted from each channel and rotated to compensate
 * for the precomputed group delay, and an average is also stored. */
void update_iq(const short buffer_low[], const short buffer_high[])
{
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


void prepare_detector(
    bool settings_changed,
    unsigned int sequencer_pc, const struct seq_entry *sequencer_table,
    unsigned int super_count, const uint32_t offsets[])
{
    /* We don't want the following states to change during a detector sweep, so
     * we write them now immediately before starting a fresh sweep. */
    hw_write_det_bunches(detector_bunches);
    hw_write_det_gain(detector_gain);
    hw_write_det_input_select(detector_input);
    hw_write_det_mode(detector_mode);

    /* Update detector or tune scale at start of tune sweep. */
    if (settings_changed  ||  tune_scale_needs_refresh)
    {
        interlock_wait(tune_scale_trigger);
        update_det_scale(sequencer_pc, sequencer_table, super_count, offsets);
        interlock_signal(tune_scale_trigger, NULL);
    }
}


/* Called during I,Q,S injection so that our frequency sweep scale matches that
 * seen by the tune sweep detection.  Note that we only update the frequency
 * waveform, all other detector sweep parameters are left alone. */
void inject_tune_scale(const double tune_scale[TUNE_LENGTH])
{
    interlock_wait(tune_scale_trigger);
    memcpy(sweep_info.tune_scale, tune_scale, sizeof(sweep_info.tune_scale));
    interlock_signal(tune_scale_trigger, NULL);
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


static void compute_default_window(float window[])
{
    /* Let's use the Hamming window.  As this is only done once at startup it
     * doesn't matter if we take our time and use full floating point
     * arithmetic. */
    float a = 0.54F;
    float b = 1 - a;
    float f = 2 * (float) M_PI / (DET_WINDOW_LENGTH - 1);
    for (int i = 0; i < DET_WINDOW_LENGTH; i ++)
        window[i] = a - b * cosf(f * (float) i);
}


static bool reset_window = true;

/* Compute the appropriate windowing function for the detector.  If called after
 * reset_window has been set then the incoming window is replaced by a standard
 * window before being written to hardware. */
static void write_detector_window(float window[])
{
    if (reset_window)
    {
        compute_default_window(window);
        reset_window = false;
    }

    int window_int[DET_WINDOW_LENGTH];
    float_array_to_int(DET_WINDOW_LENGTH, window, window_int, 16, 0);
    hw_write_det_window(window_int);
}

static void reset_detector_window(void)
{
    reset_window = true;
}


static const char *tune_sweep_state(
    bool single_bunch, unsigned int bunch, bool *sweep_ok)
{
    unsigned int sweep_bank = READ_NAMED_RECORD(mbbo, "SEQ:1:BANK");
    const int *out_wf = read_bank_out_wf(sweep_bank);

    /* Check whether the configured bank is performing sweeps. */
    bool any_sweep = false;
    bool full_sweep = true;
    bool pure_sweep = true;
    for (unsigned int i = 0; i < BUNCHES_PER_TURN; i ++)
    {
        /* Ignore bunch state for bunches outside of detector. */
        if (!single_bunch  ||  i == bunch)
        {
            if (out_wf[i] & DAC_OUT_SWEEP)
                any_sweep = true;
            else
                full_sweep = false;
            if (out_wf[i] != DAC_OUT_SWEEP)
                pure_sweep = false;
        }
    }

    *sweep_ok = any_sweep;      // Accept a sweep with any excitation
    if (!any_sweep)
        return "No tune sweep";
    else if (pure_sweep)
        return "Sweep";
    else if (full_sweep)
        return "Mixed sweep";
    else
        return "Partial sweep";
}

/* Converts gain enum into a string.  Alas, this replicates the corresponding
 * definition in sequencer.py. */
static const char *lookup_gain(unsigned int gain)
{
    const char *gain_lookup[] = {
        "0dB",   "-6dB",  "-12dB", "-18dB", "-24dB", "-30dB", "-36dB", "-42dB",
        "-48dB", "-54dB", "-60dB", "-66dB", "-72dB", "-78dB", "Off" };
    ASSERT_OK(gain < ARRAY_SIZE(gain_lookup));
    return gain_lookup[gain];
}

/* Inspects state of all settings involved with Tune operation and computes a
 * status string summarising the status. */
static EPICS_STRING read_tune_mode(void)
{
    /* Start by evaluating the sequencer.  We expect a single sequencer state
     * with data capture, sequencer enabled, and IQ buffer capture. */
    const char *status = NULL;
    if (READ_NAMED_RECORD(mbbo, "TRG:SEQ:SEL") == SEQ_DISABLED)
        status = "Not enabled";
    else if (READ_NAMED_RECORD(ulongout, "SEQ:SUPER:COUNT") != 1)
        status = "Super sequencer active";
    else if (READ_NAMED_RECORD(ulongout, "SEQ:PC") != 1)
        status = "Multi-state sequencer";
    else if (!READ_NAMED_RECORD(bo, "SEQ:1:CAPTURE")  ||
             READ_NAMED_RECORD(mbbo, "BUF:SELECT") != BUF_SELECT_IQ)
        status = "No data capture";
    else
    {
        /* At this point the sequencer is running and capturing data.  Now
         * assess whether we're generating a sweep and whether it's consistent
         * with the detector mode. */
        bool single_bunch = READ_NAMED_RECORD(bo, "DET:MODE");
        unsigned int bunch = READ_NAMED_RECORD(ulongout, "TUNE:BUNCH");

        /* Check whether the configured bank is performing sweeps. */
        bool sweep_ok;
        const char *sweep = tune_sweep_state(single_bunch, bunch, &sweep_ok);
        if (sweep_ok)
        {
            const char *gain =
                lookup_gain(READ_NAMED_RECORD(mbbo, "SEQ:1:GAIN"));
            EPICS_STRING result;
            if (single_bunch)
                snprintf(result.s, 40, "%s: bunch %u (%s)", sweep, bunch, gain);
            else
                snprintf(result.s, 40, "%s: multibunch (%s)", sweep, gain);
            return result;
        }
        else
            status = sweep;
    }

    EPICS_STRING result;
    snprintf(result.s, 40, status);
    return result;
}


bool initialise_detector(void)
{
    gain_setting = PUBLISH_WRITE_VAR_P(mbbo, "DET:GAIN", detector_gain);
    PUBLISH_WRITE_VAR_P(bo, "DET:AUTOGAIN", autogain_enable);
    PUBLISH_WRITER_P(mbbo, "DET:INPUT", write_det_input_select);
    PUBLISH_WRITE_VAR_P(bo, "DET:MODE", detector_mode);
    PUBLISH_WRITER_P(ao, "DET:LOOP:ADC", set_adc_loop_delay);

    PUBLISH_READ_VAR(bi, "DET:OVF:INP", overflows[OVERFLOW_IQ_FIR]);
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
    iq_trigger = create_interlock("DET", false);

    PUBLISH_READ_VAR(ai, "DET:DELAY", detector_delay);
    PUBLISH_WF_READ_VAR(
        double, "DET:SCALE", TUNE_LENGTH, sweep_info.tune_scale);
    PUBLISH_WF_READ_VAR(int, "DET:TIMEBASE", TUNE_LENGTH, timebase);
    tune_scale_trigger = create_interlock("DET:SCALE", false);

    /* Initialise the scaling constants so that
     *  wf_scaling * 2^wf_shift = BUNCHES_PER_TURN * 2^-32. */
    compute_scaling(BUNCHES_PER_TURN, &wf_scaling, &wf_shift);
    wf_shift -= 32;     // Divide by 2^32.

    /* Program the sequencer window. */
    PUBLISH_WF_ACTION(
        float, "DET:WINDOW", DET_WINDOW_LENGTH, write_detector_window);
    PUBLISH_ACTION("DET:RESET_WIN", reset_detector_window);

    PUBLISH_READER(stringin, "TUNE:MODE", read_tune_mode);

    return true;
}
