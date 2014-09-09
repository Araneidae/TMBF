/* Tune peak identification. */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <string.h>
#include <complex.h>
#include <math.h>

#include "error.h"
#include "epics_device.h"
#include "epics_extra.h"
#include "hardware.h"
#include "detector.h"
#include "tune_support.h"
#include "tune.h"

#include "tune_peaks.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Experimental peak measurement code. */

#define MAX_PEAKS   4

struct peak_data {
    unsigned int ix;
    unsigned int left;
    unsigned int right;
    float quality;
    float ratio;
    bool valid;
};

struct peak_info {
    unsigned int length;    // Length of waveform
    unsigned int scaling;   // Scaling factor (4 or 16 or 64)
    struct peak_data peaks[MAX_PEAKS];

    int *power;             // Smoothed power waveform
    int *power_dd;          // Second derivative of smoothed waveform

    /* Waveforms for EPICS viewing. */
    int peak_ix_wf[MAX_PEAKS];      // Index of peak
    int peak_val_wf[MAX_PEAKS];     // Value at peak index
    int peak_left_wf[MAX_PEAKS];    // Index of left boundary
    int peak_right_wf[MAX_PEAKS];   // Index of right boundary
    float peak_quality[MAX_PEAKS];  // Peak quality assessment
    float peak_ratio[MAX_PEAKS];    // Peak ratio

    unsigned int valid_peak_count;
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
static unsigned int skip_peaks(
    struct peak_info *info, unsigned int peak_count, unsigned int ix)
{
    for (unsigned int p = 0; p < peak_count; p ++)
        if (info->peaks[p].left <= ix &&  ix <= info->peaks[p].right)
            ix = info->peaks[p].right + 1;
    return ix;
}
#define FOR_VALID_PEAKS(info, peak_count, ix) \
    for (unsigned int ix = 0; \
         ix = skip_peaks(info, peak_count, ix), ix < info->length; \
         ix ++)


/* Searches for point of highest downwards curvature not enclosed by a peak
 * already found.  Returns -1 if nothing left to find. */
static bool find_peak_ix(
    struct peak_info *info, unsigned int peak_count, unsigned int *peak_ix)
{
    int min_val = 0;
    bool found = false;
    FOR_VALID_PEAKS(info, peak_count, ix)
        if (info->power_dd[ix] < min_val)
        {
            *peak_ix = ix;
            min_val = info->power_dd[ix];
            found = true;
        }
    return found;
}

/* Smooths input waveform by taking means of 4 bins at a time. */
static void smooth_waveform(unsigned int length, const int *wf_in, int *wf_out)
{
    for (unsigned int i = 0; i < length / 4; i ++)
    {
        int64_t accum = 0;
        for (unsigned int j = 0; j < 4; j ++)
            accum += wf_in[4*i + j];
        wf_out[i] = (int) (accum / 4);
    }
}

/* Computes second derivative of curve. */
static void compute_dd(unsigned int length, const int *wf_in, int *wf_out)
{
    wf_out[0] = 0;
    for (unsigned int i = 1; i < length - 1; i ++)
    {
        int64_t accum =
            (int64_t) wf_in[i-1] -
            (int64_t) 2 * wf_in[i] +
            (int64_t) wf_in[i+1];
        if (accum > INT32_MAX)
            wf_out[i] = INT32_MAX;
        else if (accum < INT32_MIN)
            wf_out[i] = INT32_MIN;
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
    struct peak_info *info, unsigned int peak_ix, struct peak_data *peak)
{
    /* Track up and then down from the putative peak to both left and right. */
    unsigned int left = peak_ix;
    while (left > 0  &&  info->power[left - 1] >= info->power[left])
        left -= 1;
    while (left > 0  &&  info->power[left - 1] <= info->power[left])
        left -= 1;

    unsigned int right = peak_ix;
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
    struct peak_info *info, struct peak_data *peak, unsigned int peak_count)
{
    unsigned int ix = 0;
    while (ix < peak_count  &&  peak->ix > info->peaks[ix].ix)
        ix ++;
    /* Move up the rest to make room. */
    memmove(info->peaks + ix + 1, info->peaks + ix,
        sizeof(struct peak_data) * (peak_count - ix));
    info->peaks[ix] = *peak;
}


static float compute_dd_variance(
    struct peak_info *info, unsigned int peak_count)
{
    int64_t variance = 0;
    unsigned int count = 0;
    FOR_VALID_PEAKS(info, peak_count, ix)
    {
        count += 1;
        variance += (int64_t) info->power_dd[ix] * info->power_dd[ix];
    }
    if (count > 0)
        return (float) variance / (float) count;
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
        return (float) height / (float) base;
    else
        return (float) height;
}

/* Determine a quality factor for each peak and determine whether it will count
 * as valid for subsequent processing. */
static void assess_peak_quality(struct peak_info *info)
{
    float deviation = sqrtf(compute_dd_variance(info, MAX_PEAKS));
    if (deviation == 0)
        deviation = 0.1F;

    int max_height = find_max_val(info->length, info->power);
    int min_height = (int) (min_peak_height * max_height);

    info->valid_peak_count = 0;
    for (unsigned int p = 0; p < MAX_PEAKS; p ++)
    {
        struct peak_data *peak = &info->peaks[p];
        if (peak->valid)
        {
            peak->quality = (float) -info->power_dd[peak->ix] / deviation;
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
    for (unsigned int i = 0; i < MAX_PEAKS; i ++)
    {
        struct peak_data *peak = &info->peaks[i];
        info->peak_ix_wf[i]     = (int) peak->ix;
        info->peak_val_wf[i]    = info->power[peak->ix];
        info->peak_left_wf[i]   = (int) peak->left;
        info->peak_right_wf[i]  = (int) peak->right;
        info->peak_quality[i]   = peak->quality;
        info->peak_ratio[i]     = peak->ratio;
    }
}


/* Top level routine for peak processing. */
static void process_peak(struct peak_info *info)
{
    compute_dd(info->length, info->power, info->power_dd);

    /* Work through second derivative and extract all peaks. */
    for (unsigned int p = 0; p < MAX_PEAKS; p ++)
    {
        struct peak_data peak = { .valid = true };
        unsigned int peak_ix = 0;
        if (find_peak_ix(info, p, &peak_ix))
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
    unsigned int peak_count = 0;
    for (unsigned int p = 0; p < MAX_PEAKS; p ++)
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
    struct peak_info *info, struct peak_data *peak,
    unsigned int *left, unsigned int *right)
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
    unsigned int left, right;
    threshold_peak(info, peak, &left, &right);
    /* Convert ranges into original data ranges. */
    left  = left  * info->scaling + info->scaling / 2;
    right = right * info->scaling + info->scaling / 2;

    /* Perform polynomial fit on the data. */
    double result;
    if (fit_quadratic(right - left + 1, sweep->power + left, &result))
    {
        index_to_tune(
            tune_scale, sweep->wf_i, sweep->wf_q, left + result, tune, phase);
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
void process_peaks(
    unsigned int length, const struct channel_sweep *sweep,
    const double *tune_scale)
{
    smooth_waveform(TUNE_LENGTH,    sweep->power,       peak_info_4.power);
    smooth_waveform(TUNE_LENGTH/4,  peak_info_4.power,  peak_info_16.power);
    smooth_waveform(TUNE_LENGTH/16, peak_info_16.power, peak_info_64.power);

    process_peak(&peak_info_4);
    process_peak(&peak_info_16);
    process_peak(&peak_info_64);

    process_peak_tune(sweep, tune_scale, select_peak_info());
}


static struct peak_info *publish_peak_info(
    struct peak_info *info, unsigned int ratio)
{
    unsigned int length = TUNE_LENGTH / ratio;
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
    PUBLISH_READ_VAR(ulongin, FORMAT("PEAKC"), info->valid_peak_count);
#undef FORMAT
    return info;
}

bool initialise_tune_peaks(void)
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
    return true;
}
