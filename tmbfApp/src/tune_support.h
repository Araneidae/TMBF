/* Helper functions for tune measurement and curve fitting. */

struct peak_range { unsigned int left; unsigned int right; };
struct one_pole { double complex a; double complex b; complex double c; };


/* Returns |z|^2, really ought to be in standard C library. */
static inline double cabs2(double complex z)
{
    return creal(z)*creal(z) + cimag(z)*cimag(z);
}


/* Evaluation functions for one_pole model for extracting the centre frequency,
 * phase at that frequency, peak width (half-width at half power maximum), and
 * integrated peak power (missing a factor of pi, as vertical units are rather
 * arbitrary). */
static inline double peak_centre(const struct one_pole *fit)
{
    return creal(fit->b);
}

static inline double peak_phase(const struct one_pole *fit)
{
    return carg(-I * fit->a);
}

static inline double peak_width(const struct one_pole *fit)
{
    return -cimag(fit->b);
}

static inline double peak_area(const struct one_pole *fit)
{
    return cabs2(fit->a) / -cimag(fit->b);
}


/* Returns value of maximum element of array. */
int find_max_val(unsigned int length, const int array[]);

/* Returns centre from fitting quadratic to the given data, fails if the centre
 * is outside the waveform. */
bool fit_quadratic(unsigned int length, const int wf[], double *result);

/* Given a list of discovered peaks (ordered by size) performs sequential fits
 * of one pole filters to each peak.  The following parameters are passed:
 *
 * peak_count   Number of peaks to fit (normally up to 3)
 * refine_fit   Controls existing fits are used to refine data
 * scale_in     Frequency scale
 * wf_i, wf_q   Raw IQ detector data
 * ranges       List of ranges for each peak
 * fits         List of fitting results
 * errors       List of fit errors */
unsigned int fit_multiple_peaks(
    unsigned int peak_count, bool refine_fit,
    const double scale_in[], const short wf_i[], const short wf_q[],
    const struct peak_range ranges[],
    struct one_pole fits[], double errors[]);

/* Given index into sweep and the corresponding tune scale computes the tune and
 * phase for the given index. */
void index_to_tune(
    const double tune_scale[], const short wf_i[], const short wf_q[],
    double ix, double *tune, double *phase);

/* Evaluates one pole model at given frequency. */
complex double eval_one_pole_model(
    unsigned int peak_count, const struct one_pole fits[], double s);

/* Computes second derivative of input curve.  The waveform length is preserved,
 * so the two end points are set to zero. */
void compute_dd(unsigned int length, const int wf_in[], int wf_out[]);

/* Smooths and decimates waveform by factor of four.  Output waveform must be
 * length/4 points long. */
void smooth_waveform_4(unsigned int length, const int wf_in[], int wf_out[]);
