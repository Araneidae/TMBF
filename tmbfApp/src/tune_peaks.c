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
static double peak_fit_threshold = 0.3;
static double min_peak_width = 0;
static double max_peak_width = 1;
static double max_fit_error = 1;


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
};


/* Extract peak data into info structure for publishing over EPICS. */
static void extract_peak_data(
    struct raw_peak_data peak_data[], struct peak_info *info)
{
    memset(info->peak_ix_wf, 0, sizeof(info->peak_ix_wf));
    memset(info->peak_val_wf, 0, sizeof(info->peak_val_wf));
    memset(info->peak_left_wf, 0, sizeof(info->peak_left_wf));
    memset(info->peak_right_wf, 0, sizeof(info->peak_right_wf));

    for (unsigned int i = 0; i < info->peak_count; i ++)
    {
        struct raw_peak_data *peak = &peak_data[i];
        info->peak_ix_wf[i] = (int) peak->ix;
        info->peak_val_wf[i] = info->power[peak->ix];
        info->peak_left_wf[i] = (int) peak->left;
        info->peak_right_wf[i] = (int) peak->right;
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
#undef FORMAT
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Peak fitting results. */

enum peak_status { PEAK_GOOD, PEAK_NO_RANGE, PEAK_NO_FIT, PEAK_REJECTED };

/* This structure contains information about a round of peak fitting. */
struct peak_fit_result {
    /* Number of valid peaks. */
    unsigned int peak_count;
    /* The ranges over which the peaks were fitted. */
    struct peak_range ranges[MAX_PEAKS];
    /* The actual peak fits. */
    struct one_pole fits[MAX_PEAKS];
    /* Relative fit errors. */
    double errors[MAX_PEAKS];
    /* Assessment for each peak. */
    enum peak_status status[MAX_PEAKS];
};


/* Ensures that a<b by swapping them if necessary. */
#define ENSURE_ORDERED(a, b) \
    do if (a > b) \
    { \
        typeof(a) t__ = (a); \
        (a) = (b); \
        (b) = (t__); \
    } while (0)



/* Peak width computation.  Used to refine region of second fit. */

/* Find index corresponding to the given tune by a binary search of tune_scale.
 * Alas the frequency list in tune_scale can go in either order, so this makes
 * our search more tricky. */
static unsigned int tune_to_index(
    unsigned int length, const double tune_scale[], double tune)
{
    unsigned int left = 0;
    unsigned int right = length - 1;
    bool forwards = tune_scale[left] < tune_scale[right];

    while (right > left)
    {
        unsigned int centre = (left + right) / 2;
        double value = tune_scale[centre];
        bool in_left = forwards ? tune <= value : tune >= value;
        if (in_left)
            right = centre;
        else
            left = centre + 1;
    }
    return left;
}


/* Computes a peak range [left<right] corresponding to the given fit and
 * threshold.  Involves searching the tune_scale to convert frequencies into
 * index. */
static void compute_peak_bounds(
    const struct one_pole *fit, double threshold,
    unsigned int length, const double tune_scale[], struct peak_range *range)
{
    /* Given z = a/(s-b) with b = s_0 + i w we have
     *                    2                          2
     *         2       |a|                    2   |a|
     *      |z|  = -------------  and  max |z|  = ----
     *                    2    2                    2
     *             (s-s_0)  + w                    b
     *
     * Thus given a threshold k we solve for |z|^2 = k max |z|^2 as equal to
     *                     (1 - k)
     *      s_0 +- w * sqrt(-----)
     *                     (  k  )
     */
    double delta = peak_width(fit) * sqrt((1 - threshold) / threshold);
    double left  = peak_centre(fit) + delta;
    double right = peak_centre(fit) - delta;
    range->left  = tune_to_index(length, tune_scale, left);
    range->right = tune_to_index(length, tune_scale, right);
    ENSURE_ORDERED(range->left, range->right);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Peak fitting, processing, and evaluation. */


static void reset_fit_result(
    struct peak_fit_result *peak_fit, unsigned int peak_count)
{
    for (unsigned int i = peak_count; i < MAX_PEAKS; i ++)
    {
        peak_fit->ranges[i] = (struct peak_range) { };
        peak_fit->status[i] = PEAK_NO_RANGE;
        peak_fit->errors[i] = 0;
    }
    peak_fit->peak_count = peak_count;
}


/* Converts peak bounds on the filtered data into enclosing bounds on the raw
 * data for peak fitting and initialises the peak_fit_result structure. */
static void extract_peak_ranges(
    unsigned int length,
    const struct peak_info *info, struct peak_fit_result *peak_fit)
{
    const unsigned int *left  = (const unsigned int *) info->peak_left_wf;
    const unsigned int *right = (const unsigned int *) info->peak_right_wf;
    for (unsigned int i = 0; i < info->peak_count; i ++)
    {
        struct peak_range range = {
            .left  = info->scaling * left[i],
            .right = info->scaling * (right[i] + 1) - 1 };
        /* Clip range to actual length. */
        if (range.left  >= length)  range.left  = length - 1;
        if (range.right >= length)  range.right = length - 1;
        peak_fit->ranges[i] = range;
    }
    reset_fit_result(peak_fit, info->peak_count);
}


/* Extracts peaks assessed as of good quality into a fresh peak fit struct.  In
 * this case we compute the ranges from the previous fit. */
static void extract_good_peaks(
    unsigned int length, const double tune_scale[],
    const struct peak_fit_result *peak_fit_in,
    struct peak_fit_result *peak_fit_out)
{
    unsigned int peak_count = 0;
    for (unsigned int i = 0; i < MAX_PEAKS; i ++)
        if (peak_fit_in->status[i] == PEAK_GOOD)
        {
            peak_fit_out->fits[peak_count] = peak_fit_in->fits[i];
            compute_peak_bounds(
                &peak_fit_in->fits[i], peak_fit_threshold,
                length, tune_scale, &peak_fit_out->ranges[peak_count]);
            peak_count += 1;
        }
    reset_fit_result(peak_fit_out, peak_count);
}


/* Peak validity assessment.  Checks that the fit error is within the selected
 * threshold, and that the peak is wider that a selected threshold.  Note that
 * the width is in tune frequency units. */
static bool assess_peak(
    const struct peak_range *range, const double tune_scale[],
    const struct one_pole *fit, double error)
{
    return
        /* Discard if peak not within fit area. */
        tune_scale[range->left] < peak_centre(fit)  &&
        peak_centre(fit) < tune_scale[range->right]  &&
        /* Discard if fit error too large. */
        error < max_fit_error  &&
        /* Discard if peak too narrow.  This will automatically reject peaks of
         * negative width, this should not arise! */
        min_peak_width < peak_width(fit)  &&
        peak_width(fit) < max_peak_width;
}


/* Performs peak fitting or refinement and assesses the results. */
static void fit_peaks(
    const struct channel_sweep *sweep, const double tune_scale[],
    struct peak_fit_result *peak_fit, bool refine_fit)
{
    double threshold = refine_fit ? 0 : peak_fit_threshold;
    unsigned int fit_count = fit_multiple_peaks(
        peak_fit->peak_count, threshold, refine_fit,
        tune_scale, sweep->wf_i, sweep->wf_q, sweep->power,
        peak_fit->ranges, peak_fit->fits, peak_fit->errors);

    for (unsigned int i = fit_count; i < peak_fit->peak_count; i ++)
        peak_fit->status[i] = PEAK_NO_FIT;
    for (unsigned int i = 0; i < fit_count; i ++)
        if (assess_peak(
               &peak_fit->ranges[i], tune_scale,
               &peak_fit->fits[i], peak_fit->errors[i]))
            peak_fit->status[i] = PEAK_GOOD;
        else
            peak_fit->status[i] = PEAK_REJECTED;

    peak_fit->peak_count = fit_count;
}


/* Extract final good peak fit from fitted results in ascending order of centre
 * frequency.  For the tiny number of peaks insertion sort is good. */
static unsigned int extract_sorted_fits(
    const struct peak_fit_result *peak_fit, struct one_pole fits[])
{
    unsigned int peak_count = 0;
    for (unsigned int i = 0; i < MAX_PEAKS; i ++)
        if (peak_fit->status[i] == PEAK_GOOD)
        {
            const struct one_pole *fit = &peak_fit->fits[i];

            /* Push this new value down as far as it will go. */
            unsigned int n = peak_count;
            while (n > 0  &&  peak_centre(fit) < peak_centre(&fits[n-1]))
            {
                fits[n] = fits[n-1];
                n -= 1;
            }
            fits[n] = *fit;

            peak_count += 1;
        }

    /* Fill remaining fields with zeros. */
    for (unsigned int i = peak_count; i < MAX_PEAKS; i ++)
        fits[i] = (struct one_pole) { };
    return peak_count;
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Published results from final fitting and evaluation stage. */

/* Published results for a single identified peak. */
struct peak_result {
    double tune;
    double phase;
    double area;
    double width;
    double height;
    bool valid;
};

struct peak_result_relative {
    double delta_tune;
    double delta_phase;
    double rel_area;
    double rel_height;
};

static unsigned fitted_peak_count;

/* Results are published for the central tune resonance and the two immediate
 * synchrotron sidebands, if they are detected. */
static struct peak_result left_peak;
static struct peak_result centre_peak;
static struct peak_result right_peak;
static struct peak_result_relative left_peak_relative;
static struct peak_result_relative right_peak_relative;
static double synchrotron_tune;


/* Computes tune properties (centre frequency, phase, area, width) from an
 * optional fit argument, sets entire result to invalid if no peak found. */
static void update_peak_result(
    struct peak_result *result, const struct one_pole *fit)
{
    result->valid = fit != NULL;
    if (fit)
    {
        double harmonic;
        result->tune = modf(peak_centre(fit), &harmonic);
        result->phase = 180 / M_PI * peak_phase(fit);
        result->width = peak_width(fit);
        result->area = peak_area(fit);
        result->height = result->area / result->width;
    }
    else
    {
        result->tune  = NAN;
        result->phase = NAN;
        result->area  = NAN;
        result->width = NAN;
        result->height = NAN;
    }
}

static double wrap_angle(double angle)
{
    if (angle > 180)
        return angle - 360;
    else if (angle < -180)
        return angle + 360;
    else
        return angle;
}

static void update_peak_result_relative(
    const struct peak_result *absolute, struct peak_result_relative *result)
{
    result->delta_tune  = fabs(absolute->tune - centre_peak.tune);
    result->delta_phase = wrap_angle(absolute->phase - centre_peak.phase);
    result->rel_area    = absolute->area / centre_peak.area;
    result->rel_height  = absolute->height / centre_peak.height;
}


/* From the final selection of peaks, in ascending order of frequency, we select
 * the central tune peak and its two synchrotron sidebands ... if possible.  The
 * heart of this is a switch statement on the number of peaks actually detected,
 * and when there are only two we have to make a choice. */
static unsigned int extract_peak_tune(
    unsigned int peak_count, const struct one_pole fits[],
    double *tune, double *phase)
{
    const struct one_pole *left   = NULL;
    const struct one_pole *centre = NULL;
    const struct one_pole *right  = NULL;

    switch (peak_count)
    {
        default:
        case 0:
            break;

        case 1:
            centre = &fits[0];
            break;
        case 2:
            /* Take the "largest" peak as the tune. */
            if (peak_area(&fits[0]) >= peak_area(&fits[1]))
            {
                centre = &fits[0];
                right  = &fits[1];
            }
            else
            {
                left   = &fits[0];
                centre = &fits[1];
            }
            break;
        case 3:
            left   = &fits[0];
            centre = &fits[1];
            right  = &fits[2];
            break;
    }

    update_peak_result(&left_peak,   left);
    update_peak_result(&centre_peak, centre);
    update_peak_result(&right_peak,  right);
    update_peak_result_relative(&left_peak, &left_peak_relative);
    update_peak_result_relative(&right_peak, &right_peak_relative);
    synchrotron_tune = 0.5 * (
        left_peak_relative.delta_tune + right_peak_relative.delta_tune);

    if (centre == NULL)
        return TUNE_NO_PEAK;
    else
    {
        *tune = centre_peak.tune;
        /* For the reported phase use the entire model to compute this to give
         * us a slightly more realistic measurement. */
        *phase = 180 / M_PI *
            carg(eval_one_pole_model(peak_count, fits, creal(centre->b)));
        return TUNE_OK;
    }
}


static struct peak_fit_result first_fit;
static struct peak_fit_result second_fit;


/* Top level control of peak fitting and tune extraction.  Takes as given a list
 * of candidate peaks, and the quality of the rest of the result depends on the
 * quality of this initial list. */
static void process_peak_tune(
    unsigned int length,
    const struct channel_sweep *sweep, const double tune_scale[],
    const struct peak_info *info,
    unsigned int *status, double *tune, double *phase)
{
    /* Perform initial fit on raw peak ranges. */
    extract_peak_ranges(length, info, &first_fit);
    fit_peaks(sweep, tune_scale, &first_fit, false);

    /* Refine the fit. */
    extract_good_peaks(length, tune_scale, &first_fit, &second_fit);
    fit_peaks(sweep, tune_scale, &second_fit, true);

    /* Extract the final peaks in ascending order of frequency. */
    struct one_pole final_fits[MAX_PEAKS];
    fitted_peak_count = extract_sorted_fits(&second_fit, final_fits);

    /* Finally compute the three peaks and the associated tune. */
    *status = extract_peak_tune(fitted_peak_count, final_fits, tune, phase);
}


static void publish_peak_result(const char *prefix, struct peak_result *result)
{
    char buffer[40];
#define FORMAT(name) \
    (sprintf(buffer, "TUNE:PEAK:%s%s", prefix, name), buffer)

    PUBLISH_READ_VAR(ai, FORMAT(""),       result->tune);
    PUBLISH_READ_VAR(ai, FORMAT(":PHASE"), result->phase);
    PUBLISH_READ_VAR(ai, FORMAT(":AREA"),  result->area);
    PUBLISH_READ_VAR(ai, FORMAT(":WIDTH"), result->width);
    PUBLISH_READ_VAR(ai, FORMAT(":HEIGHT"), result->height);
    PUBLISH_READ_VAR(bi, FORMAT(":VALID"), result->valid);
#undef FORMAT
}

static void publish_peak_result_relative(
    const char *prefix, struct peak_result_relative *result)
{
    char buffer[40];
#define FORMAT(name) \
    (sprintf(buffer, "TUNE:PEAK:%s%s", prefix, name), buffer)

    PUBLISH_READ_VAR(ai, FORMAT(":DTUNE"),   result->delta_tune);
    PUBLISH_READ_VAR(ai, FORMAT(":DPHASE"),  result->delta_phase);
    PUBLISH_READ_VAR(ai, FORMAT(":RAREA"),   result->rel_area);
    PUBLISH_READ_VAR(ai, FORMAT(":RHEIGHT"), result->rel_height);
#undef FORMAT
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

static int peak_power_4[TUNE_LENGTH / 4];
static struct peak_info peak_info_16;
static struct peak_info peak_info_64;

enum { PEAK_16, PEAK_64 };
static unsigned int peak_select;

static struct peak_info *select_peak_info(void)
{
    switch (peak_select)
    {
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
    smooth_waveform_4(TUNE_LENGTH,    sweep->power, peak_power_4);
    smooth_waveform_4(TUNE_LENGTH/4,  peak_power_4, peak_info_16.power);
    smooth_waveform_4(TUNE_LENGTH/16, peak_info_16.power, peak_info_64.power);

    process_peak_info(&peak_info_16);
    process_peak_info(&peak_info_64);

    struct peak_info *peak_info = select_peak_info();
    process_peak_tune(
        length, sweep, tune_scale, peak_info, status, tune, phase);
}


#define PUBLISH_PEAK_FIT(name, value) \
    PUBLISH_WF_READ_VAR(char, name, sizeof(struct peak_fit_result), \
        (char *) *(struct peak_fit_result *[]) { &value })

bool initialise_tune_peaks(void)
{
    publish_peak_info(&peak_info_16, 16);
    publish_peak_info(&peak_info_64, 64);

    PUBLISH_WRITE_VAR_P(ao, "TUNE:PEAK:THRESHOLD", peak_fit_threshold);
    PUBLISH_WRITE_VAR_P(ao, "TUNE:PEAK:MINWIDTH", min_peak_width);
    PUBLISH_WRITE_VAR_P(ao, "TUNE:PEAK:MAXWIDTH", max_peak_width);
    PUBLISH_WRITE_VAR_P(ao, "TUNE:PEAK:FITERROR", max_fit_error);

    PUBLISH_WRITE_VAR_P(mbbo, "TUNE:PEAK:SEL", peak_select);

    PUBLISH_READ_VAR(ulongin, "TUNE:PEAK:COUNT", fitted_peak_count);

    PUBLISH_PEAK_FIT("TUNE:PEAK:FIRSTFIT", first_fit);
    PUBLISH_PEAK_FIT("TUNE:PEAK:SECONDFIT", second_fit);

    publish_peak_result("LEFT",   &left_peak);
    publish_peak_result("CENTRE", &centre_peak);
    publish_peak_result("RIGHT",  &right_peak);
    publish_peak_result_relative("LEFT",  &left_peak_relative);
    publish_peak_result_relative("RIGHT", &right_peak_relative);
    PUBLISH_READ_VAR(ai, "TUNE:PEAK:SYNCTUNE", synchrotron_tune);

    return true;
}
