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
static double max_fit_error;


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

#define MAX_PEAKS   3


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


/* Very simple assessment of D2 peak: always accept the first peak, then try
 * filtering the remaining simply by ratio against the the size of the first and
 * largest. */
static void assess_d2_peaks(
    struct raw_peak_data peak_data[], struct peak_info *info)
{
    float dd_0 = (float) -info->power_dd[peak_data[0].ix];
    if (info->peak_count > 0  &&  dd_0 > 0)
    {
        for (unsigned int i = 0; i < info->peak_count; i ++)
            info->d2_ratio[i] = (float) -info->power_dd[peak_data[i].ix] / dd_0;

        for (unsigned int i = 0; i < info->peak_count; i ++)
            if (info->d2_ratio[i] < min_peak_d2_ratio)
            {
                info->peak_count = i;
                break;
            }
    }
    else
        info->peak_count = 0;
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

    /* Assess D2 peaks and reject those that are too small. */
    assess_d2_peaks(peak_data, info);
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


/* Published results from final fitting and evaluation stage. */

static double peak_height[MAX_PEAKS];
static double peak_phase[MAX_PEAKS];
static double peak_width[MAX_PEAKS];
static double peak_centre[MAX_PEAKS];

static unsigned fitted_peak_count;



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


/* Preliminary sanity checking on a fit candidate.  The fit should fit
 * comfortably within the selected interval and should not be narrower than our
 * minimum fit threshold. */
static bool sanity_check_peak(
    const struct peak_range *range, const double tune_scale[],
    const struct one_pole *fit, double error)
{
    double left  = tune_scale[range->left];
    double right = tune_scale[range->right];
    if (left > right)
    {
        right = tune_scale[range->left];
        left  = tune_scale[range->right];
    }
    double centre = creal(fit->b);

    return
        /* Discard if fit error too large. */
        error < max_fit_error  &&
        /* Discard if peak not comfortably within fit area. */
        left <= centre  &&  centre <= right  &&
        /* Discard if peak too narrow. */
        -cimag(fit->b) > min_peak_width;
}


static unsigned int sanity_check_peaks(
    unsigned int peak_count, const struct peak_range ranges[],
    const double tune_scale[], const struct one_pole fits[],
    const double errors[])
{
    unsigned int peak_ix = 0;
    for (; peak_ix < peak_count; peak_ix ++)
        if (!sanity_check_peak(
                &ranges[peak_ix], tune_scale, &fits[peak_ix], errors[peak_ix]))
            break;
    return peak_ix;
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


static bool threshold_peak(const struct one_pole *fit, double error)
{
    return
        /* Discard if fit error too large. */
        error < max_fit_error  &&
        /* Discard if peak too narrow. */
        -cimag(fit->b) > min_peak_width;
}


/* Only accept peaks which pass the selection critera. */
static unsigned int threshold_peaks(
    unsigned int peak_count,
    const struct one_pole fits_in[], const double errors[],
    struct one_pole fits_out[])
{
    unsigned int count = 0;
    for (unsigned int i = 0; i < peak_count; i ++)
        if (threshold_peak(&fits_in[i], errors[i]))
        {
            fits_out[count] = fits_in[i];
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
    double area1 = cabs2(fit1->a) / -cimag(fit1->b);
    double area2 = cabs2(fit2->a) / -cimag(fit2->b);
    return area1 >= area2 ? fit1 : fit2;
}


static unsigned int extract_peak_tune(
    unsigned int peak_count, const struct one_pole fits[],
    double *tune, double *phase)
{
    const struct one_pole *fit = NULL;
    switch (peak_count)
    {
        default:
        case 0: return TUNE_NO_PEAK;
        case 1: fit = &fits[0];                             break;
        case 2: fit = choose_larger(&fits[0], &fits[1]);    break;
        case 3: fit = &fits[1];                             break;
    }

    double harmonic;
    *tune = modf(creal(fit->b), &harmonic);
    *phase = 180 / M_PI *
        carg(eval_one_pole_model(peak_count, fits, creal(fit->b)));
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

    /* First compute the fit without any preconceptions.  This means an
     * unweighted fit which doesn't take any preexisting fit into account. */
    struct one_pole fits[peak_count];
    double errors[peak_count];
    peak_count = fit_multiple_peaks(
        peak_count, peak_fit_threshold, false,
        tune_scale, sweep->wf_i, sweep->wf_q, sweep->power, ranges,
        fits, errors);

    /* Perform sanity filtering on the first fits.  This allows us to discard
     * some clearly wrong fits.  Again, we discard all candidates after the
     * first failure. */
    peak_count = sanity_check_peaks(
        peak_count, ranges, tune_scale, fits, errors);

    /* Next repeat the fit to refine it taking the existing fits into account
     * with weights and data correction. */
    peak_count = fit_multiple_peaks(
        peak_count, peak_fit_threshold, true,
        tune_scale, sweep->wf_i, sweep->wf_q, sweep->power, ranges,
        fits, errors);

    update_peak_fit_detail(peak_count, fits);

    /* Finally check all the remaining peaks for sanity.  Anything that's passed
     * all the checks is for real, so we can sort the result into ascending
     * order of frequency and finally compute the tune. */
    struct one_pole final_fits[peak_count];
    peak_count = threshold_peaks(peak_count, fits, errors, final_fits);
    fitted_peak_count = peak_count;
    sort_peak_fits(peak_count, final_fits);
    *status = extract_peak_tune(peak_count, final_fits, tune, phase);
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
    PUBLISH_WRITE_VAR_P(ao, "TUNE:PEAK:FITERROR", max_fit_error);

    PUBLISH_WRITE_VAR_P(mbbo, "TUNE:PEAK:SEL", peak_select);

    PUBLISH_WF_READ_VAR(double, "TUNE:PEAK:WFHEIGHT", MAX_PEAKS, peak_height);
    PUBLISH_WF_READ_VAR(double, "TUNE:PEAK:WFPHASE",  MAX_PEAKS, peak_phase);
    PUBLISH_WF_READ_VAR(double, "TUNE:PEAK:WFWIDTH",  MAX_PEAKS, peak_width);
    PUBLISH_WF_READ_VAR(double, "TUNE:PEAK:WFCENTRE", MAX_PEAKS, peak_centre);
    PUBLISH_READ_VAR(ulongin, "TUNE:PEAK:COUNT", fitted_peak_count);

    return true;
}
