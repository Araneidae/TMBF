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

#include "tune.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Experimental peak measurement code. */

#define MAX_EDGES   20
#define MAX_PEAKS   4

struct edge_info {
    bool up;
    int start;
    int end;
};

struct peak_info {
    size_t length;      // Length of waveform
    int min_threshold;
    int max_threshold;
    int edge_count;
    int peak_count;
    struct edge_info edges[MAX_EDGES];
    int edge_wf[MAX_EDGES];
    int peak_ix_wf[MAX_PEAKS];
    int peak_val_wf[MAX_PEAKS];
    float peak_q[MAX_PEAKS];
    int wf[];           // Smoothed power waveform
};

static struct peak_info *peak_info_4;
static struct peak_info *peak_info_16;
static struct peak_info *peak_info_64;

static double peak_threshold_min;
static double peak_threshold_max;


/* Returns index of maximum element of array. */
static int find_max(int length, const int *array)
{
    int max_val = array[0];
    int max_ix = 0;
    for (int i = 1; i < length; i ++)
        if (array[i] > max_val)
        {
            max_val = array[i];
            max_ix = i;
        }
    return max_ix;
}


static void emit_edge(struct peak_info *info, int start, int end, bool up)
{
    if (info->edge_count < MAX_EDGES)
    {
        struct edge_info *edge = &info->edges[info->edge_count];
        edge->up = up;
        edge->start = start;
        edge->end = end;
        info->edge_wf[info->edge_count] = info->wf[end] - info->wf[start];
        info->edge_count += 1;
    }
}

/* Scan for peaks. */
static void scan_edges(struct peak_info *info)
{
    info->edge_count = 0;
    memset(info->edges, 0, sizeof(info->edges));
    memset(info->edge_wf, 0, sizeof(info->edge_wf));

    for (size_t ix = 1; ix < info->length; )
    {
        size_t start_ix = ix - 1;
        while (ix < info->length  &&  info->wf[ix] >= info->wf[ix - 1])
            ix ++;
        size_t end_ix = ix - 1;
        int step_up = info->wf[end_ix] - info->wf[start_ix];
        if (step_up >= info->max_threshold)
            emit_edge(info, start_ix, end_ix, true);

        start_ix = ix - 1;
        while (ix < info->length  &&  info->wf[ix] <= info->wf[ix - 1])
            ix ++;
        end_ix = ix - 1;
        int step_dn = info->wf[start_ix] - info->wf[end_ix];
        if (step_dn >= info->max_threshold)
            emit_edge(info, start_ix, end_ix, false);
    }
}


#define MAX(a, b) ((a) > (b) ? (a) : (b))

static void emit_peak(struct peak_info *info, int edge_ix)
{
    if (info->peak_count < MAX_PEAKS)
    {
        int left_ix = info->edges[edge_ix-1].start;
        int middle_ix = info->edges[edge_ix].start;
        int right_ix = info->edges[edge_ix].end;
        int left = info->wf[left_ix];
        int middle = info->wf[middle_ix];
        int right = info->wf[right_ix];

        info->peak_ix_wf[info->peak_count] = middle_ix;
        info->peak_val_wf[info->peak_count] = middle;

        /* Compute a simple peak quality factor. */
        info->peak_q[info->peak_count] =
            (middle - MAX(left, right)) / (float) info->max_threshold;

        info->peak_count += 1;
    }
}

static void find_peaks(struct peak_info *info)
{
    info->peak_count = 0;
    for (int ix = 0; ix < info->edge_count; ix ++)
    {
        while (ix < info->edge_count  &&  info->edges[ix].up)
            ix += 1;
        if (0 < ix  &&  ix < info->edge_count)
            emit_peak(info, ix);
        while (ix < info->edge_count  &&  !info->edges[ix].up)
            ix += 1;
    }
    for (int ix = info->peak_count; ix < MAX_PEAKS; ix ++)
    {
        info->peak_ix_wf[ix] = 0;
        info->peak_val_wf[ix] = 0;
        info->peak_q[ix] = 0;
    }
}


static void process_peak(struct peak_info *info)
{
    int max_val = info->wf[find_max(info->length, info->wf)];
    info->min_threshold = (int) round(peak_threshold_min * max_val);
    info->max_threshold = (int) round(peak_threshold_max * max_val);
    scan_edges(info);
    find_peaks(info);
}


/* Smooths input waveform by taking means of 4 bins at a time. */
static void smooth_waveform(size_t length, const int *wf_in, int *wf_out)
{
    for (size_t i = 0; i < length / 4; i ++)
    {
        int64_t accum = 0;
        for (size_t j = 0; j < 4; j ++)
            accum += wf_in[4*i + j];
        wf_out[i] = (int) (accum / 4);
    }
}


static void process_peaks(const int power[TUNE_LENGTH])
{
    smooth_waveform(TUNE_LENGTH,    power,            peak_info_4->wf);
    smooth_waveform(TUNE_LENGTH/4,  peak_info_4->wf,  peak_info_16->wf);
    smooth_waveform(TUNE_LENGTH/16, peak_info_16->wf, peak_info_64->wf);
    process_peak(peak_info_4);
    process_peak(peak_info_16);
    process_peak(peak_info_64);
}


static struct peak_info *publish_peak_info(size_t length, const char *suffix)
{
    struct peak_info *info =
        malloc(sizeof(struct peak_info) + length * sizeof(int));
    info->length = length;

    char buffer[20];
#define FORMAT(name) \
    (sprintf(buffer, "TUNE:%s:%s", name, suffix), buffer)
    PUBLISH_WF_READ_VAR(int, FORMAT("POWER"), length, info->wf);
    PUBLISH_WF_READ_VAR(int, FORMAT("EDGE"), MAX_EDGES, info->edge_wf);
    PUBLISH_WF_READ_VAR(int, FORMAT("PEAKIX"), MAX_PEAKS, info->peak_ix_wf);
    PUBLISH_WF_READ_VAR(int, FORMAT("PEAKV"), MAX_PEAKS, info->peak_val_wf);
    PUBLISH_WF_READ_VAR(float, FORMAT("PEAKQ"), MAX_PEAKS, info->peak_q);
    PUBLISH_READ_VAR(longin, FORMAT("ECOUNT"), info->edge_count);
#undef FORMAT
    return info;
}

static void publish_peaks(void)
{
    peak_info_4  = publish_peak_info(TUNE_LENGTH/4,  "4");
    peak_info_16 = publish_peak_info(TUNE_LENGTH/16, "16");
    peak_info_64 = publish_peak_info(TUNE_LENGTH/64, "64");

    PUBLISH_WRITE_VAR_P(ao, "TUNE:PEAK:MIN", peak_threshold_min);
    PUBLISH_WRITE_VAR_P(ao, "TUNE:PEAK:MAX", peak_threshold_max);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Tune measurement. */

static double threshold_fraction = 0.3;
static int min_block_separation = 20;
static int min_block_length = 20;


static double interpolate(double ix, const short wf[])
{
    double base;
    double frac = modf(ix, &base);
    int ix0 = (int) base;
    return (1 - frac) * wf[ix0] + frac * wf[ix0];
}


/* Converts a tune, detected as an index into the tune sweep waveform, into the
 * corresponding tune frequency offset and phase. */
static void index_to_tune(
    const struct channel_sweep *sweep, const double *tune_scale,
    double ix, double *tune, double *phase)
{
    double harmonic;
    *tune = modf(tune_scale[(int) round(ix)], &harmonic);
    *phase = 180.0 / M_PI * atan2(
        interpolate(ix, sweep->wf_q), interpolate(ix, sweep->wf_i));
}


enum tune_status {
    TUNE_INVALID,       // Not calculated
    TUNE_OK,            // Satisfactory result
    TUNE_NO_PEAK,       // Peak not found
    TUNE_EXTRA_PEAKS,   // Too many peaks found
    TUNE_BAD_FIT,       // Misshapen peak
    TUNE_OVERFLOW,      // Detector input overflow
    TUNE_RANGE,         // Alarm range error
};


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


#if 0
/* This concise snippet of code was lifted from
 *      http://stackoverflow.com/a/984286
 * This is a direct application of Cramer's rule to invert a 3x3 matrix. */
static void invert_matrix(const double A[3][3], double result[3][3])
{
    double determinant =
        +A(0,0)*(A(1,1)*A(2,2) - A(2,1)*A(1,2))
        -A(0,1)*(A(1,0)*A(2,2) - A(1,2)*A(2,0))
        +A(0,2)*(A(1,0)*A(2,1) - A(1,1)*A(2,0));
    double invdet = 1/determinant;
    result(0,0) =  (A(1,1)*A(2,2) - A(2,1)*A(1,2))*invdet;
    result(1,0) = -(A(0,1)*A(2,2) - A(0,2)*A(2,1))*invdet;
    result(2,0) =  (A(0,1)*A(1,2) - A(0,2)*A(1,1))*invdet;
    result(0,1) = -(A(1,0)*A(2,2) - A(1,2)*A(2,0))*invdet;
    result(1,1) =  (A(0,0)*A(2,2) - A(0,2)*A(2,0))*invdet;
    result(2,1) = -(A(0,0)*A(1,2) - A(1,0)*A(0,2))*invdet;
    result(0,2) =  (A(1,0)*A(2,1) - A(2,0)*A(1,1))*invdet;
    result(1,2) = -(A(0,0)*A(2,1) - A(2,0)*A(0,1))*invdet;
    result(2,2) =  (A(0,0)*A(1,1) - A(1,0)*A(0,1))*invdet;
}
#endif

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
static bool fit_quadratic(int length, const int wf[], double *result)
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


static epicsAlarmSeverity measure_tune(
    int length, const struct channel_sweep *sweep,
    const double *tune_scale, bool overflow,
    unsigned int *tune_status, double *tune, double *phase)
{
    process_peaks(sweep->power);

    /* Very crude algorithm:
     *  1.  Find peak
     *  2.  Slice data to 1/3 of peak height and look for contiguous and well
     *      separated blocks above this threshold.
     *  3.  If one block found, use it, if two blocks found take second peak.
     * Refinements include:
     *  -   Median filter on incoming data
     *  -   Quadratic fit for peak detect
     *  -   Knockout waveform for spike removal. */
    int peak_ix = find_max(length, sweep->power);
    int threshold = (int) (threshold_fraction * sweep->power[peak_ix]);
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
        case 2:
            *tune_status = find_peak(
                sweep->power, &blocks[block_count - 1], &tune_ix);
            break;
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
    int gain_wf[BUNCHES_PER_TURN];
    READ_NAMED_RECORD_WF(char, "BUN:0:FIRWF_S", fir_wf, BUNCHES_PER_TURN);
    READ_NAMED_RECORD_WF(int,  "BUN:0:GAINWF_S", gain_wf, BUNCHES_PER_TURN);
    WRITE_NAMED_RECORD_WF(char, "BUN:1:FIRWF_S", fir_wf, BUNCHES_PER_TURN);
    WRITE_NAMED_RECORD_WF(int,  "BUN:1:GAINWF_S", gain_wf, BUNCHES_PER_TURN);

    bool single_bunch_mode = READ_NAMED_RECORD_VALUE(bo, "DET:MODE");

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
    WRITE_NAMED_RECORD(ao, "SEQ:1:START_FREQ",
        centre + (reverse_tune ? + half_range : - half_range));
    WRITE_NAMED_RECORD(ao, "SEQ:1:END_FREQ",
        centre + (reverse_tune ? - half_range : + half_range));
    WRITE_NAMED_RECORD(bo,       "SEQ:1:CAPTURE", true);
    WRITE_NAMED_RECORD(mbbo,     "SEQ:1:BANK", 1);
    WRITE_NAMED_RECORD(ulongout, "SEQ:PC", 1);

    /* Configure the bunch control as a copy of bank 0, but with sweep enabled
     * for output. */
    set_bunch_control();

    /* Let the user know that the settings are now valid. */
    set_tune_setting(true);
}


bool initialise_tune(void)
{
    tune_trigger = create_interlock("TUNE:TRIG", "TUNE:DONE", false);
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

    trigger_record(tune_setting_rec, 0, NULL);

    publish_peaks();
    return true;
}
