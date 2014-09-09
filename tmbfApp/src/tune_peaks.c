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


/* Some externally configured control parameters. */
static double min_peak_d2_ratio;
static double peak_fit_threshold;
static double min_peak_width;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Peak detection and extraction. */

/* Result of peak extraction is the index of the maximum D2 point for each peak
 * together with a span covering the peak. */
struct raw_peak_data {
    unsigned int ix;        // Index of discovered peak (for info only)
    unsigned int left;      // Left and right boundaries of peak in smoothed
    unsigned int right;     //  data indices.
};


/* Searches for point of highest downwards curvature not enclosed by a peak
 * already found.  Returns -1 if nothing left to find. */
static bool find_peak_ix(
    unsigned int length, const int dd[], bool peak_marks[],
    unsigned int *peak_ix)
{
    int min_val = 0;
    bool found = false;
    for (unsigned int ix = 0; ix < length; ix ++)
        if (!peak_marks[ix]  &&  dd[ix] < min_val)
        {
            *peak_ix = ix;
            min_val = dd[ix];
            found = true;
        }
    return found;
}


/* Given a point at the peak (as determined by DD) track away in both directions
 * from this peak so that we span the entire raised peak.  Because the detected
 * "peak" may be slightly off peak, we track up before tracking down.  Note that
 * this can result in overlapping peak ranges, but this shouldn't matter.  */
static void find_peak_limits(
    unsigned int length, const int power[], unsigned int peak_ix,
    unsigned int *peak_left, unsigned int *peak_right)
{
    /* Track up and then down from the putative peak to both left and right. */
    unsigned int left = peak_ix;
    while (left > 0  &&  power[left - 1] >= power[left])
        left -= 1;
    while (left > 0  &&  power[left - 1] <= power[left])
        left -= 1;

    unsigned int right = peak_ix;
    while (right < length - 1  &&  power[right + 1] >= power[right])
        right += 1;
    while (right < length - 1  &&  power[right + 1] <= power[right])
        right += 1;

    *peak_left = left;
    *peak_right = right;
}


/* Walks through second derivative and extracts all peaks in descending order of
 * size.  Returns the number of peaks successfully extracted, all remaining
 * peaks are set to invalid values. */
static unsigned int extract_peaks(
    unsigned int length, unsigned int max_peaks,
    const int power[], const int dd[],
    bool peak_marks[], struct raw_peak_data peak_data[])
{
    unsigned int peak_ix = 0;
    for (; peak_ix < max_peaks; peak_ix ++)
    {
        unsigned int max_ix = 0;
        if (!find_peak_ix(length, dd, peak_marks, &max_ix))
            break;

        /* Discover the bounds of the peak and mark it off as seen. */
        unsigned int left, right;
        find_peak_limits(length, power, max_ix, &left, &right);
        for (unsigned int i = left; i <= right; i ++)
            peak_marks[i] = true;

        peak_data[peak_ix] = (struct raw_peak_data) {
            .ix = max_ix, .left = left, .right = right };
    }

    return peak_ix;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Peak digest.  Having extracted peaks by second derivative extraction we
 * publish information about each peak and perform some preliminary processing
 * and qualification. */

#define MAX_PEAKS   4


struct peak_info {
    unsigned int length;    // Length of waveform
    unsigned int scaling;   // Scaling factor (4 or 16 or 64)

    int *power;             // Smoothed power waveform
    int *power_dd;          // Second derivative of smoothed waveform

    unsigned int peak_count;    // Number of valid peaks found

    /* Waveforms for EPICS viewing. */
    int peak_ix_wf[MAX_PEAKS];      // Index of peak
    int peak_val_wf[MAX_PEAKS];     // Value at peak index
    int peak_left_wf[MAX_PEAKS];    // Index of left boundary
    int peak_right_wf[MAX_PEAKS];   // Index of right boundary
    float d2_ratio[MAX_PEAKS];      // Size of D2 peaks relative to noise
};


/* Extract peak data into info structure for publishing over EPICS. */
static void extract_peak_data(
    struct raw_peak_data peak_data[], struct peak_info *info)
{
    memset(info->peak_ix_wf, 0, sizeof(info->peak_ix_wf));
    memset(info->peak_val_wf, 0, sizeof(info->peak_val_wf));
    memset(info->peak_left_wf, 0, sizeof(info->peak_left_wf));
    memset(info->peak_right_wf, 0, sizeof(info->peak_right_wf));
    memset(info->d2_ratio, 0, sizeof(info->d2_ratio));

    for (unsigned int i = 0; i < info->peak_count; i ++)
    {
        struct raw_peak_data *peak = &peak_data[i];
        info->peak_ix_wf[i] = (int) peak->ix;
        info->peak_val_wf[i] = info->power[peak->ix];
        info->peak_left_wf[i] = (int) peak->left;
        info->peak_right_wf[i] = (int) peak->right;
    }
}


/* Computes the background variance of the second derivative over that part of
 * the waveform where no peaks were detected.  Used to qualify the peaks that
 * were found. */
static float compute_dd_deviation(
    unsigned int length, const int dd[], const bool peak_marks[])
{
    int64_t variance = 0;
    unsigned int count = 0;
    for (unsigned int ix = 0; ix < length; ix ++)
        if (!peak_marks[ix])
        {
            count += 1;
            variance += (int64_t) dd[ix] * dd[ix];
        }
    if (count > 0)
        return sqrtf((float) variance / (float) count);
    else
        return 0;
}


/* Determine a quality factor for each peak and determine whether it will count
 * as valid for subsequent processing. */
static void compute_d2_peak_quality(
    struct raw_peak_data peak_data[], float deviation, struct peak_info *info)
{
    for (unsigned int i = 0; i < info->peak_count; i ++)
        info->d2_ratio[i] = (float) -info->power_dd[peak_data[i].ix];

    if (deviation > 0)
        for (unsigned int i = 0; i < info->peak_count; i ++)
            info->d2_ratio[i] /= deviation;
}


/* If any peak's D2 ratio is smaller than the minimum ratio then discard it and
 * all smaller peaks. */
static void qualify_d2_peaks(struct peak_info *info)
{
    for (unsigned int i = 0; i < info->peak_count; i ++)
        if (info->d2_ratio[i] < min_peak_d2_ratio)
        {
            info->peak_count = i;
            break;
        }
}


/* Top level routine for peak processing.  Takes as input smoothed peak data and
 * from this computes the second derivatives and extracts low level peaks. */
static void process_peak_info(struct peak_info *info)
{
    /* Compute second derivative of smoothed data for peak detection. */
    compute_dd(info->length, info->power, info->power_dd);

    /* Work through second derivative and extract all peaks. */
    bool peak_marks[info->length];
    memset(peak_marks, 0, sizeof(peak_marks));
    struct raw_peak_data peak_data[MAX_PEAKS];
    info->peak_count = extract_peaks(
        info->length, MAX_PEAKS, info->power, info->power_dd,
        peak_marks, peak_data);

    /* Convert peak data into presentation format. */
    extract_peak_data(peak_data, info);

    /* Use background variance for a preliminary filter on the peak quality. */
    float deviation = compute_dd_deviation(
        info->length, info->power_dd, peak_marks);
    compute_d2_peak_quality(peak_data, deviation, info);

    /* Use relative d2 quality to qualify peaks. */
    qualify_d2_peaks(info);
}


/* EPICS interface for the peak processing stage. */
static void publish_peak_info(struct peak_info *info, unsigned int ratio)
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
    PUBLISH_WF_READ_VAR(float, FORMAT("PEAKQ"), MAX_PEAKS, info->d2_ratio);
    PUBLISH_READ_VAR(ulongin, FORMAT("PEAKC"), info->peak_count);
#undef FORMAT
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Peak fitting and final evaluation. */



#if 0

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

#endif


static double peak_height[MAX_PEAKS];
static double peak_phase[MAX_PEAKS];
static double peak_width[MAX_PEAKS];
static double peak_centre[MAX_PEAKS];


/* Converts peak bounds on the filtered data into enclosing bounds on the raw
 * data for peak fitting. */
static void extract_peak_ranges(
    struct peak_info *info, struct peak_range ranges[])
{
    unsigned int *left  = (unsigned int *) info->peak_left_wf;
    unsigned int *right = (unsigned int *) info->peak_right_wf;
    for (unsigned int i = 0; i < info->peak_count; i ++)
        ranges[i] = (struct peak_range) {
            .left  = info->scaling * left[i],
            .right = info->scaling * (right[i] + 1) - 1 };
}


/* Convert fit coefficients into physically significant numbers. */
static void update_peak_fit_detail(
    unsigned int peak_count, const struct one_pole fits[])
{
    for (unsigned int i = 0; i < peak_count; i ++)
        decode_one_pole(&fits[i],
            &peak_height[i], &peak_width[i], &peak_centre[i], &peak_phase[i]);

    for (unsigned int i = peak_count; i < MAX_PEAKS; i ++)
    {
        peak_height[i] = 0;
        peak_width[i] = 0;
        peak_centre[i] = 0;
        peak_phase[i] = 0;
    }
}


static bool threshold_peak(const struct one_pole *fit)
{
    /* For the moment just a simple width threshold. */
    return -cimag(fit->b) > min_peak_width;
}


/* Discard all peaks which fail the selection critera. */
static unsigned int threshold_peaks(
    unsigned int peak_count, struct one_pole fits[])
{
    bool valid_peak[peak_count];
    for (unsigned int i = 0; i < peak_count; i ++)
        valid_peak[i] = threshold_peak(&fits[i]);

    unsigned int count = 0;
    for (unsigned int i = 0; i < peak_count; i ++)
        if (valid_peak[i])
        {
            if (i != count) fits[count] = fits[i];
            count += 1;
        }
    return count;
}


/* Once we've filtered out the peaks we don't want we sort the remainer into
 * ascending order of peak centre frequency. */
static void sort_peak_fits(
    unsigned int peak_count, struct one_pole fits[])
{
    /* Sort peak fits in ascending order of real(.b), this being the true tune
     * centre for each peak.  The list is tiny (up to four points), so a simple
     * insertion sort is just fine. */
    for (unsigned int n = 1; n < peak_count; n ++)
    {
        struct one_pole fit = fits[n];
        unsigned int i = n;
        while (i > 0  &&  creal(fits[i - 1].b) > creal(fit.b))
        {
            fits[i] = fits[i - 1];
            i -= 1;
        }
        fits[i] = fit;
    }
}


static const struct one_pole *choose_larger(
    const struct one_pole *fit1, const struct one_pole *fit2)
{
    if (cabs2(fit1->a) > cabs2(fit2->a))
        return fit1;
    else
        return fit2;
}


static unsigned int extract_peak_tune(
    unsigned int peak_count, const struct one_pole fits[],
    double *tune, double *phase)
{
    const struct one_pole *fit = NULL;
    switch (peak_count)
    {
        case 0: return TUNE_NO_PEAK;
        case 1: fit = &fits[0];                             break;
        case 2: fit = choose_larger(&fits[0], &fits[1]);    break;
        case 3: fit = &fits[1];                             break;
        default:
        case 4: return TUNE_EXTRA_PEAKS;
    }

    double harmonic;
    *tune = modf(creal(fit->b), &harmonic);
    *phase = carg(-I * fit->a);
    return TUNE_OK;
}


static void process_peak_tune(
    const struct channel_sweep *sweep, const double tune_scale[],
    struct peak_info *info,
    unsigned int *status, double *tune, double *phase)
{
    unsigned int peak_count = info->peak_count;
    struct peak_range ranges[peak_count];
    extract_peak_ranges(info, ranges);

    struct one_pole fits[peak_count];
    /* First compute the fit without any preconceptions.  This means an
     * unweighted fit which doesn't take any preexisting fit into account. */
    peak_count = fit_multiple_peaks(
        peak_count, peak_fit_threshold, false,
        tune_scale, sweep->wf_i, sweep->wf_q, sweep->power, ranges, fits);
    /* Next repeat the fit to refine it taking the existing fits into account
     * with weights and data correction. */
    peak_count = fit_multiple_peaks(
        peak_count, peak_fit_threshold, true,
        tune_scale, sweep->wf_i, sweep->wf_q, sweep->power, ranges, fits);

    update_peak_fit_detail(peak_count, fits);

    peak_count = threshold_peaks(peak_count, fits);
    sort_peak_fits(peak_count, fits);

    *status = extract_peak_tune(peak_count, fits, tune, phase);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

static struct peak_info peak_info_4;
static struct peak_info peak_info_16;
static struct peak_info peak_info_64;

enum { PEAK_4, PEAK_16, PEAK_64 };
static unsigned int peak_select;



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
void measure_tune_peaks(
    unsigned int length, const struct channel_sweep *sweep,
    const double *tune_scale,
    unsigned int *status, double *tune, double *phase)
{
    smooth_waveform_4(TUNE_LENGTH,    sweep->power,       peak_info_4.power);
    smooth_waveform_4(TUNE_LENGTH/4,  peak_info_4.power,  peak_info_16.power);
    smooth_waveform_4(TUNE_LENGTH/16, peak_info_16.power, peak_info_64.power);

    process_peak_info(&peak_info_4);
    process_peak_info(&peak_info_16);
    process_peak_info(&peak_info_64);

    struct peak_info *peak_info = select_peak_info();
    process_peak_tune(sweep, tune_scale, peak_info, status, tune, phase);
}


bool initialise_tune_peaks(void)
{
    publish_peak_info(&peak_info_4, 4);
    publish_peak_info(&peak_info_16, 16);
    publish_peak_info(&peak_info_64, 64);

    PUBLISH_WRITE_VAR_P(ao, "TUNE:PEAK:MIND2", min_peak_d2_ratio);
    PUBLISH_WRITE_VAR_P(ao, "TUNE:PEAK:THRESHOLD", peak_fit_threshold);
    PUBLISH_WRITE_VAR_P(ao, "TUNE:PEAK:MINWIDTH", min_peak_width);

    PUBLISH_WRITE_VAR_P(mbbo, "TUNE:PEAK:SEL", peak_select);

    PUBLISH_WF_READ_VAR(double, "TUNE:PEAK:WFHEIGHT", MAX_PEAKS, peak_height);
    PUBLISH_WF_READ_VAR(double, "TUNE:PEAK:WFPHASE",  MAX_PEAKS, peak_phase);
    PUBLISH_WF_READ_VAR(double, "TUNE:PEAK:WFWIDTH",  MAX_PEAKS, peak_width);
    PUBLISH_WF_READ_VAR(double, "TUNE:PEAK:WFCENTRE", MAX_PEAKS, peak_centre);

    return true;
}
