/* More advanced tune handling.  This processes the IQ waveforms after the
 * detector module has handled them.  This separation is so that more complex
 * and experimental tune detection can be separated from the core code. */

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <string.h>
#include <math.h>

#include "error.h"
#include "epics_device.h"
#include "epics_extra.h"
#include "hardware.h"
#include "detector.h"
#include "sequencer.h"
#include "tune_peaks.h"

#include "tune.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Tune helper functions. */

int find_max_val(int length, const int array[])
{
    int max_val = array[0];
    for (int i = 1; i < length; i ++)
        if (array[i] > max_val)
            max_val = array[i];
    return max_val;
}


/* We are given a waveform of points wf[] to which we wish to fit a quatratic
 * and return the index of axis.  If we fit a polynomial y = a + b x + c x^2
 * then the centre line is at y' = 0, ie at x = - b / 2 c.
 *
 * We'll start with a classic presentation, eg
 *  http://en.wikipedia.org/wiki/Polynomial_regression
 * Write V for the Vandermonde matrix, for i ranging over the length of input
 * and j = 0, 1, 2:
 *
 *           j
 *  V    = x  ,
 *   i,j    i
 *
 * where each x_i is one of the input indexes.  In our case the x_i are just
 * indexes into the input data wf[], and for a technical reason which we'll see
 * in a moment we'll offset them so that Sum_i x_i = 0.
 *
 * Given this definition of V, and writing y_i = wf[x_i] (before applying the
 * offset mentioned above), then we want to construct a 3 element vector
 * A = [a, b, c] to minimise the error  V A - y .  The next step I don't really
 * understand, but the trick is to take:
 *   T        T                           T   -1 T
 *  V  V A = V y  and then compute  A = (V  V)  V y .
 *
 * When we multiply it out we discover that M = V^T V is very uniform in
 * structure, and indeed
 *               i+j
 *  M    = Sum  x
 *   i,j      i
 * Now by taking the x_i to be symmetrically arranged we can ensure that M_i,j
 * is zero when i+j is odd, and so in the end our problem reduces to the very
 * simple form:
 *
 *  [a]   [M_0  0    M_2]-1 [Y_0]
 *  [b] = [0    M_2  0  ]   [Y_1]
 *  [c]   [M_2  0    M_4]   [Y_2]
 *
 * where  M_n = Sum_i x^n  and  Y_n = Sum_i x^n y_i .
 *
 * We still need to invert V^T V, but its form is particularly simple.  Also,
 * we're not interested in the first row of the result (a), and as we're taking
 * the ratio of b and c we're not interested in the determinant.  Taking both
 * these into account we get the result (for concision we're writing Mn for M_n)
 *                                      [Y0]
 *  [b] = [  0     M0 M4 - M2^2    0  ] [Y1]
 *  [c]   [-M2^2        0        M0 M2] [Y2] ,
 * ie,
 *      b = (M0 M4 - M2^2) Y1
 *      c = M0 M2 Y2 - M2^2 Y0 . */
bool fit_quadratic(int length, const int wf[], double *result)
{
    /* Confusingly, here M[n] = M2n from the context above. */
    double M[3] = { length, 0, 0 };
    double Y[3] = { 0, 0, 0 };
    double centre = (length - 1) / 2.0;
    for (int i = 0; i < length; i ++)
    {
        double n = i - centre;
        double n2 = n * n;
        M[1] += n2;
        M[2] += n2 * n2;

        int y = wf[i];
        Y[0] += y;
        Y[1] += n * y;
        Y[2] += n2 * y;
    }

    double b = (M[0] * M[2] - M[1] * M[1]) * Y[1];
    double c = M[0] * M[1] * Y[2] - M[1] * M[1] * Y[0];

    /* Now sanity check before calculating result: we expect final result in the
     * range 0..N-1, or before correcting for the offset, we expect
     * abs(-b/2c - (N-1)/2) < (N-1)/2; after some rearrangement, we get the test
     * below. */
    if (fabs(b) < fabs((length - 1) * c))
    {
        *result = centre - 0.5 * b / c;
        return true;
    }
    else
        return false;
}


/* Interpolate between two adjacent indicies in a waveform */
#define INTERPOLATE(ix, frac, wf) \
    ((1 - (frac)) * (wf)[ix] + (frac) * (wf)[(ix) + 1])


/* Converts a tune, detected as an index into the tune sweep waveform, into the
 * corresponding tune frequency offset and phase. */
void index_to_tune(
    const struct channel_sweep sweep[], const double tune_scale[],
    double ix, double *tune, double *phase)
{
    double base;
    double frac = modf(ix, &base);
    int ix0 = (int) base;

    double harmonic;
    *tune = modf(INTERPOLATE(ix0, frac, tune_scale), &harmonic);
    *phase = 180.0 / M_PI * atan2(
        INTERPOLATE(ix0, frac, sweep->wf_q),
        INTERPOLATE(ix0, frac, sweep->wf_i));
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Tune measurement. */

static double threshold_fraction = 0.3;
static int min_block_separation = 20;
static int min_block_length = 20;


/* Searches given waveform for blocks meeting the peak detection criteria,
 * returns number of blocks found.  Only MAX_BLOCKS blocks will be searched for,
 * returns MAX_BLOCKS if at least this many blocks found. */
#define MAX_BLOCKS  3
struct block { int start; int end; };
static int find_blocks(
    int length, const int wf[], int threshold,
    struct block blocks[MAX_BLOCKS])
{
    int count = 0;

    struct block dummy = { 0, 0 };
    struct block *last_block = &dummy;
    int ix = 0;
    while (count < MAX_BLOCKS)
    {
        struct block *block = &blocks[count];

        /* Search for start of block. */
        while (ix < length  &&  wf[ix] < threshold)
            ix += 1;
        if (ix >= length)  break;
        block->start = ix;

        /* Search for end of block. */
        while (ix < length  &&  wf[ix] >= threshold)
            ix += 1;
        block->end = ix;

        /* Assess what we've found.  Block must be long enough and sufficiently
         * separated from the previous block. */
        bool sep_ok = block->start - last_block->end >= min_block_separation;
        bool len_ok = block->end - block->start >= min_block_length;
        if (sep_ok  &&  len_ok)
        {
            last_block = block;
            count += 1;
        }
    }

    return count;
}


static enum tune_status find_peak(
    const int wf[], struct block *block, double *peak_ix)
{
    double result;
    if (fit_quadratic(block->end - block->start, wf + block->start, &result))
    {
        *peak_ix = block->start + result;
        return TUNE_OK;
    }
    else
        return TUNE_BAD_FIT;
}


static int find_high_ix(
    const struct block *blocks, int count, const double tune_scale[])
{
    int low_ix  = (blocks[0].start + blocks[0].end) / 2;
    int high_ix = (blocks[count-1].start + blocks[count-1].end) / 2;
    if (tune_scale[low_ix] < tune_scale[high_ix])
        return count-1;
    else
        return 0;
}

static epicsAlarmSeverity measure_tune(
    int length, const struct channel_sweep *sweep,
    const double tune_scale[], bool overflow,
    unsigned int *tune_status, double *tune, double *phase)
{
    /* Very crude algorithm:
     *  1.  Find peak
     *  2.  Slice data to 1/3 of peak height and look for contiguous and well
     *      separated blocks above this threshold.
     *  3.  If one block found, use it, if two blocks found take second peak.
     * Refinements include:
     *  -   Median filter on incoming data
     *  -   Quadratic fit for peak detect
     *  -   Knockout waveform for spike removal. */
    int peak_val = find_max_val(length, sweep->power);
    int threshold = (int) (threshold_fraction * peak_val);
    struct block blocks[MAX_BLOCKS];
    int block_count = find_blocks(length, sweep->power, threshold, blocks);

    double tune_ix;
    switch (block_count)
    {
        case 0:
            *tune_status = TUNE_NO_PEAK;
            break;
        case MAX_BLOCKS:
            *tune_status = TUNE_EXTRA_PEAKS;
            break;
        case 1:
            *tune_status = find_peak(sweep->power, &blocks[0], &tune_ix);
            break;
        case 2:
        {
            // Take the peak with the higher frequency
            int block_ix = find_high_ix(blocks, 2, tune_scale);
            *tune_status = find_peak(sweep->power, &blocks[block_ix], &tune_ix);
            break;
        }
    }

    if (*tune_status == TUNE_OK)
    {
        index_to_tune(sweep, tune_scale, tune_ix, tune, phase);
        return epicsSevNone;
    }
    else
        return epicsSevInvalid;
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Control. */

/* Trigger interlock for record update. */
static struct epics_interlock *tune_trigger;

/* Tune settings. */
static int harmonic;            // Base frequency for tune sweep
static double centre_tune;      // Centre of tune sweep
static double half_range;       // Min/max tune setting
static double alarm_range;      // Alarm range to test
static bool reverse_tune;       // Set to sweep tune backwards
static int selected_bunch;      // Selected single bunch

/* Waveforms from last detector sweep. */
static struct channel_sweep sweep;
static float phase_waveform[TUNE_LENGTH];
static int cumsum_i[TUNE_LENGTH];
static int cumsum_q[TUNE_LENGTH];
static double mean_power;
static int max_power;

/* Tune measurements. */
static double measured_tune;    // Tune measurement
static double measured_phase;   // and associated phase
static struct epics_record *measured_tune_rec;
static struct epics_record *measured_phase_rec;
static unsigned int tune_status;

/* Fake data for test injection. */
static struct sweep_info injection_sweep;


static void update_iq_power(const struct sweep_info *sweep_info)
{
    const struct channel_sweep *channel = sweep_info->single_bunch_mode ?
        &sweep_info->channels[selected_bunch % 4] :
        &sweep_info->mean;
    memcpy(&sweep, channel, sizeof(sweep));

    /* Update the total and max power statistics. */
    double total_power = 0;
    max_power = 0;
    for (int i = 0; i < sweep_info->sweep_length; i ++)
    {
        int power = sweep.power[i];
        total_power += power;
        if (power > max_power)
            max_power = power;
    }
    mean_power = total_power / sweep_info->sweep_length;
}


static void update_phase_wf(void)
{
    for (int i = 0; i < TUNE_LENGTH; i ++)
        phase_waveform[i] = 180.0 / M_PI * atan2f(sweep.wf_q[i], sweep.wf_i[i]);
}


static void update_cumsum(const struct sweep_info *sweep_info)
{
    int sum_i = 0, sum_q = 0;
    for (int i = 0; i < sweep_info->sweep_length; i ++)
    {
        sum_i += sweep.wf_i[i];
        sum_q += sweep.wf_q[i];
        cumsum_i[i] = sum_i;
        cumsum_q[i] = sum_q;
    }

    /* Pad the rest of the waveform with repeats of the last point. */
    for (int i = sweep_info->sweep_length; i < TUNE_LENGTH; i ++)
    {
        cumsum_i[i] = sum_i;
        cumsum_q[i] = sum_q;
    }
}


void update_tune_sweep(const struct sweep_info *sweep_info, bool overflow)
{
    interlock_wait(tune_trigger);
    update_iq_power(sweep_info);
    update_phase_wf();
    update_cumsum(sweep_info);

    epicsAlarmSeverity severity;
    if (overflow)
    {
        severity = epicsSevMajor;
        tune_status = TUNE_OVERFLOW;
    }
    else
    {
        process_peaks(&sweep, sweep_info->tune_scale);

        severity = measure_tune(
            sweep_info->sweep_length, &sweep, sweep_info->tune_scale, overflow,
            &tune_status, &measured_tune, &measured_phase);
        if (severity == epicsSevNone  &&
            /* Check for measured tune within given alarm range. */
            fabs(measured_tune - centre_tune) >= alarm_range)
        {
            severity = epicsSevMinor;
            tune_status = TUNE_RANGE;
        }
    }
    trigger_record(measured_tune_rec, severity, NULL);
    trigger_record(measured_phase_rec, severity, NULL);
    interlock_signal(tune_trigger, NULL);
}


/* Inject given test data as power sweep by forcing call to update_tune_sweep
 * with synthetic sweep info structure. */
static void inject_test_data(void *context, int *array, size_t *length)
{
    injection_sweep.sweep_length = *length;
    injection_sweep.single_bunch_mode = false;
    memcpy(injection_sweep.mean.power, array,
        sizeof(injection_sweep.mean.power));

    /* We don't have ready access to the frequency scale, so just fake one. */
    for (int i = 0; i < TUNE_LENGTH; i ++)
        injection_sweep.tune_scale[i] = (double) i / TUNE_LENGTH;

    update_tune_sweep(&injection_sweep, false);
}


static bool tune_setting;
static struct epics_record *tune_setting_rec;

static void set_tune_setting(bool setting)
{
    if (tune_setting != setting)
    {
        tune_setting = setting;
        trigger_record(tune_setting_rec, 0, NULL);
    }
}

void tune_setting_changed(void)
{
    set_tune_setting(false);
}


static void set_bunch_control(void)
{
    /* Copy FIR and gain settings from bank 0 to bank 1. */
    char fir_wf[BUNCHES_PER_TURN];
    float gain_wf[BUNCHES_PER_TURN];
    READ_NAMED_RECORD_WF(char,  "BUN:0:FIRWF_S", fir_wf, BUNCHES_PER_TURN);
    READ_NAMED_RECORD_WF(float, "BUN:0:GAINWF_S", gain_wf, BUNCHES_PER_TURN);
    WRITE_NAMED_RECORD_WF(char,  "BUN:1:FIRWF_S", fir_wf, BUNCHES_PER_TURN);
    WRITE_NAMED_RECORD_WF(float, "BUN:1:GAINWF_S", gain_wf, BUNCHES_PER_TURN);

    bool single_bunch_mode = READ_NAMED_RECORD(bo, "DET:MODE");

    char out_wf[BUNCHES_PER_TURN];
    if (single_bunch_mode)
    {
        READ_NAMED_RECORD_WF(char, "BUN:0:OUTWF_S", out_wf, BUNCHES_PER_TURN);
        out_wf[selected_bunch] = 4;
    }
    else
        memset(out_wf, 4, BUNCHES_PER_TURN);
    WRITE_NAMED_RECORD_WF(char, "BUN:1:OUTWF_S", out_wf, BUNCHES_PER_TURN);
}


/* Clips frequency to range 0..BUNCHES_PER_TURN (maximum possible mode). */
static double clip_freq_range(double frequency)
{
    if (frequency < 0)
        return 0;
    else if (frequency > BUNCHES_PER_TURN)
        return BUNCHES_PER_TURN;
    else
        return frequency;
}

/* When the user wishes to use the tune settings this function will force the
 * sequencer and detector to use the settings configured here. */
static void set_tune_settings(void)
{
    /* Configure appropriate channel for selected single bunch.  Do this whether
     * we're currently in single or multi bunch mode. */
    char bunch_channel[20];
    sprintf(bunch_channel, "DET:BUNCH%d", selected_bunch % 4);
    WRITE_NAMED_RECORD(ulongout, bunch_channel, selected_bunch / 4);

    /* Force buffer to IQ and ensure the buffer is enabled. */
    WRITE_NAMED_RECORD(mbbo, "BUF:SELECT", BUF_SELECT_IQ);
    WRITE_NAMED_RECORD(bo,   "TRG:SEQ:ENA", true);

    /* Configure the sequencer with the selected tune range.  Force count and
     * capture to sensible values and set the sequencer PC to 1. */
    double centre = harmonic + centre_tune;
    WRITE_NAMED_RECORD(ulongout, "SEQ:1:COUNT", 4096);
    WRITE_NAMED_RECORD(ao, "SEQ:1:START_FREQ", clip_freq_range(
        centre + (reverse_tune ? + half_range : - half_range)));
    WRITE_NAMED_RECORD(ao, "SEQ:1:END_FREQ", clip_freq_range(
        centre + (reverse_tune ? - half_range : + half_range)));
    WRITE_NAMED_RECORD(bo,       "SEQ:1:CAPTURE", true);
    WRITE_NAMED_RECORD(mbbo,     "SEQ:1:BANK", 1);
    WRITE_NAMED_RECORD(ulongout, "SEQ:PC", 1);

    /* Effectively disable the super sequencer by setting it to 1 and resetting
     * the offsets waveform. */
    WRITE_NAMED_RECORD(ulongout, "SEQ:SUPER:COUNT", 0);
    WRITE_NAMED_RECORD(bo, "SEQ:SUPER:RESET", true);

    /* Configure the bunch control as a copy of bank 0, but with sweep enabled
     * for output. */
    set_bunch_control();

    /* Let the user know that the settings are now valid. */
    set_tune_setting(true);
}


bool initialise_tune(void)
{
    tune_trigger = create_interlock("TUNE", false);
    PUBLISH_WRITE_VAR_P(longout, "TUNE:HARMONIC", harmonic);
    PUBLISH_WRITE_VAR_P(ao, "TUNE:CENTRE", centre_tune);
    PUBLISH_WRITE_VAR_P(ao, "TUNE:RANGE", half_range);
    PUBLISH_WRITE_VAR_P(bo, "TUNE:DIRECTION", reverse_tune);
    PUBLISH_WRITE_VAR_P(ao, "TUNE:ALARM", alarm_range);
    PUBLISH_WRITE_VAR_P(longout, "TUNE:BUNCH", selected_bunch);

    PUBLISH_WF_READ_VAR(short, "TUNE:I", TUNE_LENGTH, sweep.wf_i);
    PUBLISH_WF_READ_VAR(short, "TUNE:Q", TUNE_LENGTH, sweep.wf_q);
    PUBLISH_WF_READ_VAR(int, "TUNE:POWER", TUNE_LENGTH, sweep.power);
    PUBLISH_WF_READ_VAR(float, "TUNE:PHASEWF", TUNE_LENGTH, phase_waveform);
    PUBLISH_READ_VAR(ai, "TUNE:MEANPOWER", mean_power);
    PUBLISH_READ_VAR(longin, "TUNE:MAXPOWER", max_power);
    PUBLISH_WF_READ_VAR(int, "TUNE:CUMSUMI", TUNE_LENGTH, cumsum_i);
    PUBLISH_WF_READ_VAR(int, "TUNE:CUMSUMQ", TUNE_LENGTH, cumsum_q);

    PUBLISH_READ_VAR(mbbi, "TUNE:STATUS", tune_status);
    measured_tune_rec  = PUBLISH_READ_VAR(ai, "TUNE:TUNE",  measured_tune);
    measured_phase_rec = PUBLISH_READ_VAR(ai, "TUNE:PHASE", measured_phase);

    PUBLISH_ACTION("TUNE:CHANGED", tune_setting_changed);
    tune_setting_rec = PUBLISH_READ_VAR_I(bi, "TUNE:SETTING", tune_setting);
    PUBLISH_ACTION("TUNE:SET", set_tune_settings);

    /* Control parameters for tune measurement algorithm. */
    PUBLISH_WRITE_VAR_P(ao, "TUNE:THRESHOLD", threshold_fraction);
    PUBLISH_WRITE_VAR_P(longout, "TUNE:BLK:SEP", min_block_separation);
    PUBLISH_WRITE_VAR_P(longout, "TUNE:BLK:LEN", min_block_length);

    PUBLISH_WAVEFORM(int, "TUNE:INJECT:P", TUNE_LENGTH, inject_test_data);

    trigger_record(tune_setting_rec, 0, NULL);

    return true;
}
