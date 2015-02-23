/* Helper functions for tune measurement and curve fitting. */

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <complex.h>

#include "error.h"
#include "hardware.h"
#include "detector.h"

#include "tune_support.h"



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Tune helper functions. */

int find_max_val(unsigned int length, const int array[])
{
    int max_val = array[0];
    for (unsigned int i = 1; i < length; i ++)
        if (array[i] > max_val)
            max_val = array[i];
    return max_val;
}


/* We are given a waveform of points wf[] to which we wish to fit a quadratic
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
bool fit_quadratic(unsigned int length, const int wf[], double *result)
{
    /* Confusingly, here M[n] = M2n from the context above. */
    double M[3] = { length, 0, 0 };
    double Y[3] = { 0, 0, 0 };
    double centre = (length - 1) / 2.0;
    for (unsigned int i = 0; i < length; i ++)
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
    const double tune_scale[], const short wf_i[], const short wf_q[],
    double ix, double *tune, double *phase)
{
    double base;
    double frac = modf(ix, &base);
    int ix0 = (int) base;

    double harmonic;
    *tune = modf(INTERPOLATE(ix0, frac, tune_scale), &harmonic);
    *phase = 180 / M_PI *
        atan2(INTERPOLATE(ix0, frac, wf_q), INTERPOLATE(ix0, frac, wf_i));
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Fitting one-pole model to IQ. */


/* Fitting one pole filter to IQ data.  Given waveforms scale[] and wf[],
 * which we write here as s[] and iq[], this function computes complex
 * parameters a and b to minimise the fitting error for the model
 *
 *               a
 *      z(s) = -----    with nominal error term  en = z - iq
 *             s - b    ie, en[i] = z(s[i]) - iq[i] .
 *
 * As a matter of practicality, minimising this error term requires an expensive
 * least squares minimisation, so instead we fudge this by multiplying through
 * by (s - b) and a weighting factor w to get the fitted error term
 *
 *      e = w . (s - b) . en = w . (a + b . iq - iq . s) .
 *
 * We can minimise E = ||e||^2 = sum_i |e[i]|^2 by solving the overdetermined
 * matrix equation
 *
 *      W M x = W y     where
 *          M[i,.] = (1, iq[i]), x = [a; b], y[i] = s[i] . iq[i], W = diag(w).
 *
 * It turns out that for complex M, x, y, we can solve this by solving the fully
 * determined equation
 *
 *      M^H W M x = M^H W y
 *
 * This is easily multiplied out to produce the equations:
 *
 *      [ S(w)      S(w iq)     ] [a] = [ S(w s iq)     ]
 *      [ S(w iq)*  S(w |iq|^2) ] [b]   [ S(w s |iq|^2) ]
 *
 * and this is of course easy to solve for (a,b) by inverting the 2x2 matrix.
 *
 * Note however that the solution here minimises w.(s-b).en, and without a
 * compensating weight this solution gives too much emphasis to points further
 * away from the centre frequency b.  This can be fixed by performing the fit
 * twice: once with w = 1 and a second time with w = 1 / |s - b|^2; assuming the
 * first fit is acceptable, this second fit will refine the fit with more
 * emphasis on the points nearer the centre frequency.
 *
 * The inverse of M^H W M is
 *
 *      [a] = 1/D [ S(w |iq|^2)  -S(w iq) ] [ S(w s iq)     ]
 *      [b]       [ -S(w iq*)    S(w)     ] [ S(w s |iq|^2) ]
 * where
 *      D = S(2) S(2 |iq|^2) - |S(w iq)|^2
 */
static bool fit_one_pole(
    unsigned int length, const double scale[], const double complex iq[],
    const double *weights, struct one_pole *fit)
{
    /* Compute the components of M^H W M and M^H W y. */
    double complex S_w_iq = 0;          // S(w iq)
    double S_w_iq2 = 0;                 // S(w |iq|^2)
    double complex S_w_s_iq = 0;        // S(w s iq)
    double S_w_s_iq2 = 0;               // S(w s |iq|^2)

    double S_w = weights ? 0 : length;  // S(w)
    for (unsigned int i = 0; i < length; i ++)
    {
        double complex w_iq = iq[i];
        double w_iq2 = cabs2(w_iq);
        if (weights)
        {
            double w = weights[i];
            S_w += w;
            w_iq  *= w;
            w_iq2 *= w;
        }

        S_w_iq    += w_iq;
        S_w_iq2   += w_iq2;
        double s = scale[i];
        S_w_s_iq  += s * w_iq;
        S_w_s_iq2 += s * w_iq2;
    }

    double det = S_w * S_w_iq2 - cabs2(S_w_iq);
    return
        TEST_OK_(length >= 2, "Singular data to fit")  &&
        TEST_OK_(fabs(det) > S_w, "One pole fit failed")  &&
        DO_(
            fit->a = (S_w_iq2 * S_w_s_iq - S_w_iq * S_w_s_iq2) / det;
            fit->b = (S_w * S_w_s_iq2 - conj(S_w_iq) * S_w_s_iq) / det
        );
}


/* Computes the relative fit error between the given data and fit, computes the
 * error as:
 *                                  2
 *              SUM | data - model |
 *      error = ---------------------
 *                              2
 *                 SUM | model |
 *
 * It's looking like this might well be a reasonable estimate of fit quality. */
static double compute_fit_error(
    unsigned int length, const double scale[], const double complex iq[],
    const struct one_pole *fit)
{
    double error = 0;
    double sum = 0;
    for (unsigned int i = 0; i < length; i ++)
    {
        double complex model = peak_eval(fit, scale[i]);
        error += cabs2(iq[i] - model);
        sum   += cabs2(model);
    }
    return error / sum;
}


/* Given data to process (scale_in, wf_i, wf_q) together with the associated
 * power already computed, and a data range with a threshold, scan the selected
 * data set and extract all points with relative power greater than the given
 * threshold.  Returns the number of points extracted. */
static unsigned int extract_threshold_data(
    const struct peak_range *range, double threshold, const int power[],
    const double scale_in[], const short wf_i[], const short wf_q[],
    double scale_out[], double complex iq[])
{
    unsigned int count = 0;
    int maxval =
        find_max_val(range->right - range->left + 1, power + range->left);
    int minval = lround(threshold * maxval);
    for (unsigned int ix = range->left; ix <= range->right; ix ++)
        if (power[ix] >= minval)
        {
            scale_out[count] = scale_in[ix];
            iq[count] = wf_i[ix] + I * wf_q[ix];
            count += 1;
        }
    return count;
}


complex double eval_one_pole_model(
    unsigned int peak_count, const struct one_pole fits[], double s)
{
    complex double result = 0;
    for (unsigned int i = 0; i < peak_count; i ++)
        result += peak_eval(&fits[i], s);
    return result;
}


/* Take existing fits into account and adjust the data we're fitting. */
static void adjust_iq_with_model(
    unsigned int peak_count, const struct one_pole fits[],
    unsigned int peak_ix, bool refine_fit,
    unsigned int count, const double scale[], double complex iq[])
{
    /* Refine the data by subtracting existing fits from the data. */
    for (unsigned int j = 0; j < peak_count; j ++)
    {
        if (j == peak_ix)
        {
            /* If we're not refining then subsequent fits are unknown, so we
             * stop processing at this point, otherwise we'll go on and subtract
             * all the old first pass fits as well. */
            if (!refine_fit)
                break;
        }
        else
        {
            /* Subtract the model from the data. */
            const struct one_pole *fit = &fits[j];
            for (unsigned int i = 0; i < count; i ++)
                iq[i] -= peak_eval(fit, scale[i]);
        }
    }
}


/* The weight function here is 1/|z-b|^2 and helps to ensure a cleaner curve
 * fit.  The first 1/|z-b| factor helps cancel out a weighting error in the
 * model, and the second factor puts more emphasis on fitting the peak.
 *    Here we compute the weights if requested, using the given workspace which
 * is returned, otherwise NULL is returned. */
static const double *maybe_compute_weights(
    unsigned int count, bool compute, double weights[],
    const double scale[], const struct one_pole *fit)
{
    if (compute)
    {
        for (unsigned int i = 0; i < count; i ++)
            weights[i] = 1 / cabs2(scale[i] - fit->b);
        return weights;
    }
    else
        return NULL;
}


/* Given a list of data ranges fit a one-pole filter to each of the ranges.  The
 * fit process is repeated twice for each peak: the first time we perform an
 * unweighted fit to the residual data after subtracting previous fits, the
 * second time we refine the data by redoing the fit with weighting and
 * subtracting the best model from the data. */
unsigned int fit_multiple_peaks(
    unsigned int peak_count, double threshold, bool refine_fit,
    const double scale_in[], const short wf_i[], const short wf_q[],
    const int power[], const struct peak_range ranges[],
    struct one_pole fits[], double errors[])
{
    unsigned int peak_ix = 0;
    for (; peak_ix < peak_count; peak_ix ++)
    {
        const struct peak_range *range = &ranges[peak_ix];
        struct one_pole *fit = &fits[peak_ix];

        /* Extract a block of (frequency, iq) data to fit this peak to. */
        unsigned int max_count = range->right - range->left + 1;
        double scale[max_count];
        double complex iq[max_count];
        unsigned int count = extract_threshold_data(
            range, threshold, power, scale_in, wf_i, wf_q, scale, iq);

        /* Adjust the iq data block according to our current model knowledge. */
        adjust_iq_with_model(
            peak_count, fits, peak_ix, refine_fit, count, scale, iq);

        /* If performing fit refinement then we need a weights vector, so
         * compute this if necessary. */
        double weights[count];
        const double *fit_weights =
            maybe_compute_weights(count, refine_fit, weights, scale, fit);

        /* Perform the fit, if this fails discard this and all subsequent fit
         * candidates. */
        if (!fit_one_pole(count, scale, iq, fit_weights, fit))
            break;

        /* Finally compute the fit error. */
        errors[peak_ix] = compute_fit_error(count, scale, iq, fit);
    }
    return peak_ix;
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Waveform processing for supporting peak detection. */


/* Computes second derivative over given waveform with saturation. */
void compute_dd(unsigned int length, const int wf_in[], int wf_out[])
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


/* Hard-wired bin size of 4 to allow for optimisations. */
void smooth_waveform_4(unsigned int length, const int wf_in[], int wf_out[])
{
    for (unsigned int i = 0; i < length / 4; i ++)
    {
        int64_t accum = 0;
        for (unsigned int j = 0; j < 4; j ++)
            accum += wf_in[4*i + j];
        wf_out[i] = (int) ((accum + 2) >> 2);   // Rounded division by 4
    }
}
