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
/* Tune measurement. */

static double threshold_fraction = 0.3;
static int min_block_separation = 20;
static int min_block_length = 20;


/* Returns value of maximum element of array. */
static int find_max_val(int length, const int *array)
{
    int max_val = array[0];
    for (int i = 1; i < length; i ++)
        if (array[i] > max_val)
            max_val = array[i];
    return max_val;
}


/* Interpolate between two adjacent indicies in a waveform */
#define INTERPOLATE(ix, frac, wf) \
    ((1 - (frac)) * (wf)[ix] + (frac) * (wf)[(ix) + 1])


/* Converts a tune, detected as an index into the tune sweep waveform, into the
 * corresponding tune frequency offset and phase. */
static void index_to_tune(
    const struct channel_sweep *sweep, const double *tune_scale,
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
/* Experimental peak measurement code. */

#define MAX_PEAKS   4

struct peak_data {
    int ix;
    int left;
    int right;
    float quality;
    float ratio;
    bool valid;
};

struct peak_info {
    int length;             // Length of waveform
    int scaling;            // Scaling factor (4 or 16 or 64)
    struct peak_data peaks[MAX_PEAKS];

    int *power;             // Smoothed power waveform
    int *power_dd;          // Second derivative of smoothed waveform

    /* Waveforms for EPICS viewing. */
    int peak_ix_wf[MAX_PEAKS];
    int peak_val_wf[MAX_PEAKS];
    int peak_left_wf[MAX_PEAKS];
    int peak_right_wf[MAX_PEAKS];
    float peak_quality[MAX_PEAKS];
    float peak_ratio[MAX_PEAKS];

    int valid_peak_count;
};

static struct peak_info peak_info_4;
static struct peak_info peak_info_16;
static struct peak_info peak_info_64;

static double min_peak_quality;
static double min_peak_height;
static double min_peak_ratio;
static double peak_fit_ratio;

enum { PEAK_4, PEAK_16, PEAK_64 };
static unsigned int peak_select;

/* Measurement results. */
static unsigned int peak_tune_status;

static struct in_epics_record_ai *peak_measured_tune;
static struct in_epics_record_ai *peak_measured_phase;


/* Skip index over peaks that have already been discovered.  Designed for use as
 * part of an iterator for walking through available points. */
static int skip_peaks(struct peak_info *info, int peak_count, int ix)
{
    for (int p = 0; p < peak_count; p ++)
        if (info->peaks[p].left <= ix &&  ix <= info->peaks[p].right)
            ix = info->peaks[p].right + 1;
    return ix;
}
#define FOR_VALID_PEAKS(info, peak_count, ix) \
    for (int ix = 0; \
         ix = skip_peaks(info, peak_count, ix), ix < info->length; \
         ix ++)


/* Searches for point of highest downwards curvature not enclosed by a peak
 * already found.  Returns -1 if nothing left to find. */
static int find_peak_ix(struct peak_info *info, int peak_count)
{
    int min_val = 0;
    int peak_ix = -1;
    FOR_VALID_PEAKS(info, peak_count, ix)
        if (info->power_dd[ix] < min_val)
        {
            peak_ix = ix;
            min_val = info->power_dd[ix];
        }
    return peak_ix;
}

/* Smooths input waveform by taking means of 4 bins at a time. */
static void smooth_waveform(int length, const int *wf_in, int *wf_out)
{
    for (int i = 0; i < length / 4; i ++)
    {
        int64_t accum = 0;
        for (int j = 0; j < 4; j ++)
            accum += wf_in[4*i + j];
        wf_out[i] = (int) (accum / 4);
    }
}

/* Computes second derivative of curve. */
static void compute_dd(int length, const int *wf_in, int *wf_out)
{
    wf_out[0] = 0;
    for (int i = 1; i < length - 1; i ++)
    {
        int64_t accum =
            (int64_t) wf_in[i-1] -
            (int64_t) 2 * wf_in[i] +
            (int64_t) wf_in[i+1];
        if (accum > INT32_MAX)
            wf_out[i] = INT32_MAX;
        else
            wf_out[i] = (int) accum;
    }
    wf_out[length - 1] = 0;
}


/* Given a point at the peak (as determined by DD) track away in both directions
 * from this peak so that we span the entire raised peak.  Because the detected
 * "peak" may be slightly off peak, we track up before tracking down.  Note that
 * this can result in overlapping peak ranges, but this shouldn't matter.  */
static void find_peak_limits(
    struct peak_info *info, int peak_ix, struct peak_data *peak)
{
    /* Track up and then down from the putative peak to both left and right. */
    int left = peak_ix;
    while (left > 0  &&  info->power[left - 1] >= info->power[left])
        left -= 1;
    while (left > 0  &&  info->power[left - 1] <= info->power[left])
        left -= 1;

    int right = peak_ix;
    while (right < info->length - 1  &&
           info->power[right + 1] >= info->power[right])
        right += 1;
    while (right < info->length - 1  &&
           info->power[right + 1] <= info->power[right])
        right += 1;

    peak->ix = peak_ix;
    peak->left = left;
    peak->right = right;
}


/* Simple insertion sort of peak data using index as sort key.  Peaks need to be
 * kept in ascending order so that skipping over already detected peaks will
 * work correctly. */
static void insert_peak(
    struct peak_info *info, struct peak_data *peak, int peak_count)
{
    int ix = 0;
    while (ix < peak_count  &&  peak->ix > info->peaks[ix].ix)
        ix ++;
    /* Move up the rest to make room. */
    memmove(info->peaks + ix + 1, info->peaks + ix,
        sizeof(struct peak_data) * (peak_count - ix));
    info->peaks[ix] = *peak;
}


static float compute_dd_variance(struct peak_info *info, int peak_count)
{
    int64_t variance = 0;
    int count = 0;
    FOR_VALID_PEAKS(info, peak_count, ix)
    {
        count += 1;
        variance += (int64_t) info->power_dd[ix] * info->power_dd[ix];
    }
    if (count > 0)
        return (float) variance / count;
    else
        return 0;
}


/* Peak ratio relative to the peak floor, used for quality assessment. */
static float peak_ratio(struct peak_info *info, struct peak_data *peak)
{
    int height = info->power[peak->ix];
    int left   = info->power[peak->left];
    int right  = info->power[peak->right];
    int base = (left + right) / 2;
    if (base > 0)
        return (float) height / base;
    else
        return (float) height;
}

/* Determine a quality factor for each peak and determine whether it will count
 * as valid for subsequent processing. */
static void assess_peak_quality(struct peak_info *info)
{
    float deviation = sqrtf(compute_dd_variance(info, MAX_PEAKS));
    if (deviation == 0)
        deviation = 0.1;

    int max_height = find_max_val(info->length, info->power);
    int min_height = (int) (min_peak_height * max_height);

    info->valid_peak_count = 0;
    for (int p = 0; p < MAX_PEAKS; p ++)
    {
        struct peak_data *peak = &info->peaks[p];
        if (peak->valid)
        {
            peak->quality = -info->power_dd[peak->ix] / deviation;
            peak->ratio = peak_ratio(info, peak);
            peak->valid =
                peak->quality >= min_peak_quality  &&
                peak->ratio >= min_peak_ratio  &&
                info->power[peak->ix] >= min_height;
        }
        else
            peak->quality = 0;

        info->valid_peak_count += peak->valid;
    }
}


/* Extract display waveforms for EPICS. */
static void compute_waveforms(struct peak_info *info)
{
    for (int i = 0; i < MAX_PEAKS; i ++)
    {
        struct peak_data *peak = &info->peaks[i];
        info->peak_ix_wf[i] = peak->ix;
        info->peak_val_wf[i] = info->power[peak->ix];
        info->peak_left_wf[i] = peak->left;
        info->peak_right_wf[i] = peak->right;
        info->peak_quality[i] = peak->quality;
        info->peak_ratio[i] = peak->ratio;
    }
}


/* Top level routine for peak processing. */
static void process_peak(struct peak_info *info)
{
    compute_dd(info->length, info->power, info->power_dd);

    /* Work through second derivative and extract all peaks. */
    for (int p = 0; p < MAX_PEAKS; p ++)
    {
        struct peak_data peak = { .valid = true };
        int peak_ix = find_peak_ix(info, p);
        if (peak_ix >= 0)
            find_peak_limits(info, peak_ix, &peak);
        else
            peak = (struct peak_data) {
                .ix = 0, .left = 0, .right = 0, .valid = false, .quality = 0 };
        insert_peak(info, &peak, p);
    }

    assess_peak_quality(info);
    compute_waveforms(info);
}


/* To simplify tune processing, all valid peaks are extracted into a contiguous
 * array. */
static void extract_valid_peaks(
    struct peak_info *info, struct peak_data peaks[3])
{
    int peak_count = 0;
    for (int p = 0; p < MAX_PEAKS; p ++)
    {
        struct peak_data *peak = &info->peaks[p];
        if (peak->valid  &&  peak_count < 3)
            peaks[peak_count++] = *peak;
    }
}


/* Returns true if peak1 is larger than peak2, false otherwise. */
static bool compare_peaks(
    struct peak_info *info, struct peak_data *peak1, struct peak_data *peak2)
{
    /* Compare the heights of the smoothed peaks, we're less trusting of the
     * original raw data for this. */
    return info->power[peak1->ix] >= info->power[peak2->ix];
}


/* Uses the programmed threshold to find sensible left and right boundaries of
 * the peak for fitting. */
static void threshold_peak(
    struct peak_info *info, struct peak_data *peak, int *left, int *right)
{
    int max_val = find_max_val(
        peak->right - peak->left + 1, info->power + peak->left);
    int threshold = (int) (peak_fit_ratio * max_val);

    *left = peak->left;
    while (*left < peak->ix  &&  info->power[*left + 1] < threshold)
        *left += 1;
    *right = peak->right;
    while (*right > peak->ix  &&  info->power[*right - 1] < threshold)
        *right -= 1;
}

/* Having found the peak computes the accurate peak index by fitting a quadratic
 * to the peak. */
static unsigned int compute_peak_tune(
    const struct channel_sweep *sweep, const double *tune_scale,
    struct peak_info *info, struct peak_data *peak,
    double *tune, double *phase)
{
    /* First compute left and right ranges for the peak. */
    int left, right;
    threshold_peak(info, peak, &left, &right);
    /* Convert ranges into original data ranges. */
    left  = left  * info->scaling + info->scaling / 2;
    right = right * info->scaling + info->scaling / 2;

    /* Perform polynomial fit on the data. */
    double result;
    if (fit_quadratic(right - left + 1, sweep->power + left, &result))
    {
        index_to_tune(sweep, tune_scale, left + result, tune, phase);
        return TUNE_OK;
    }
    else
        return TUNE_BAD_FIT;
}


/* The core peak processing algorithm is straightforward enough once we've found
 * the peaks, but it does rely on accurate peak detection.  Depending on the
 * number of peaks we proceed thus:
 *  0 peaks => nothing to see
 *  1 peak  => nothing to choose
 *  2 peaks => rising synchrotron side band.  On the assumption that we'll see
 *             the third peak before it grows to large, take the largest peak
 *  3 peaks => both immediate synchrotron side bands visible, take the middle
 *  4 peaks => too complicated */
static void process_peak_tune(
    const struct channel_sweep *sweep, const double *tune_scale,
    struct peak_info *info)
{
    struct peak_data peaks[3];
    extract_valid_peaks(info, peaks);

    int peak_ix = -1;
    switch (info->valid_peak_count)
    {
        case 0:     peak_tune_status = TUNE_NO_PEAK;        break;
        case 1:     peak_ix = 0;                            break;
        case 2:
            peak_ix = compare_peaks(info, &peaks[0], &peaks[1]) ? 0 : 1;
            break;
        case 3:     peak_ix = 1;                            break;
        case 4:     peak_tune_status = TUNE_EXTRA_PEAKS;    break;
    }

    int severity = epicsSevInvalid;
    double tune = 0, phase = 0;
    if (peak_ix >= 0)
    {
        peak_tune_status = compute_peak_tune(
            sweep, tune_scale, info, &peaks[peak_ix], &tune, &phase);
        if (peak_tune_status == TUNE_OK)
        {
#if 0
            /* Check for measured tune within given alarm range. */
            if (fabs(tune - centre_tune) >= alarm_range)
            {
                severity = epicsSevMinor;
                peak_tune_status = TUNE_RANGE;
            }
            else
#endif
                severity = epicsSevNone;
        }
    }
    if (severity == epicsSevInvalid)
    {
        WRITE_IN_RECORD_SEV(ai, peak_measured_tune, severity);
        WRITE_IN_RECORD_SEV(ai, peak_measured_phase, severity);
    }
    else
    {
        WRITE_IN_RECORD(ai, peak_measured_tune, tune);
        WRITE_IN_RECORD(ai, peak_measured_phase, phase);
    }
}


static struct peak_info *select_peak_info(void)
{
    switch (peak_select)
    {
        case PEAK_4:    return &peak_info_4;
        default:        // (arbitrary choice for default)
        case PEAK_16:   return &peak_info_16;
        case PEAK_64:   return &peak_info_64;
    }
}

/* Process each waveform in turn by smoothing it and then searching for peaks.
 * Once done, process the selected smoothing level to calculate the tune. */
static void process_peaks(
    const struct channel_sweep *sweep, const double *tune_scale)
{
    smooth_waveform(TUNE_LENGTH,    sweep->power,       peak_info_4.power);
    smooth_waveform(TUNE_LENGTH/4,  peak_info_4.power,  peak_info_16.power);
    smooth_waveform(TUNE_LENGTH/16, peak_info_16.power, peak_info_64.power);

    process_peak(&peak_info_4);
    process_peak(&peak_info_16);
    process_peak(&peak_info_64);

    process_peak_tune(sweep, tune_scale, select_peak_info());
}


static struct peak_info *publish_peak_info(struct peak_info *info, int ratio)
{
    int length = TUNE_LENGTH / ratio;
    info->scaling = ratio;
    info->length = length;
    info->power = malloc(length * sizeof(int));
    info->power_dd = malloc(length * sizeof(int));

    char buffer[20];
#define FORMAT(name) \
    (sprintf(buffer, "TUNE:%s:%d", name, ratio), buffer)
    PUBLISH_WF_READ_VAR(int, FORMAT("POWER"), length, info->power);
    PUBLISH_WF_READ_VAR(int, FORMAT("PDD"), length, info->power_dd);
    PUBLISH_WF_READ_VAR(int, FORMAT("PEAKIX"), MAX_PEAKS, info->peak_ix_wf);
    PUBLISH_WF_READ_VAR(int, FORMAT("PEAKV"), MAX_PEAKS, info->peak_val_wf);
    PUBLISH_WF_READ_VAR(int, FORMAT("PEAKL"), MAX_PEAKS, info->peak_left_wf);
    PUBLISH_WF_READ_VAR(int, FORMAT("PEAKR"), MAX_PEAKS, info->peak_right_wf);
    PUBLISH_WF_READ_VAR(float, FORMAT("PEAKQ"), MAX_PEAKS, info->peak_quality);
    PUBLISH_WF_READ_VAR(float, FORMAT("PEAKH"), MAX_PEAKS, info->peak_ratio);
    PUBLISH_READ_VAR(longin, FORMAT("PEAKC"), info->valid_peak_count);
#undef FORMAT
    return info;
}

static void publish_peaks(void)
{
    publish_peak_info(&peak_info_4, 4);
    publish_peak_info(&peak_info_16, 16);
    publish_peak_info(&peak_info_64, 64);

    PUBLISH_WRITE_VAR_P(ao, "TUNE:PEAK:MINQ", min_peak_quality);
    PUBLISH_WRITE_VAR_P(ao, "TUNE:PEAK:MINH", min_peak_height);
    PUBLISH_WRITE_VAR_P(ao, "TUNE:PEAK:MINR", min_peak_ratio);
    PUBLISH_WRITE_VAR_P(ao, "TUNE:PEAK:FIT", peak_fit_ratio);
    PUBLISH_WRITE_VAR_P(mbbo, "TUNE:PEAK:SEL", peak_select);

    PUBLISH_READ_VAR(mbbi, "TUNE:PEAK:STATUS", peak_tune_status);
    peak_measured_tune =
        PUBLISH_IN_VALUE(ai, "TUNE:PEAK:TUNE", .force_update = true);
    peak_measured_phase =
        PUBLISH_IN_VALUE(ai, "TUNE:PEAK:PHASE", .force_update = true);
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

    PUBLISH_WAVEFORM(int, "TUNE:INJECT:P", TUNE_LENGTH, inject_test_data);

    trigger_record(tune_setting_rec, 0, NULL);

    publish_peaks();
    return true;
}
