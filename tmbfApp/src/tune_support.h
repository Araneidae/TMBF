/* Helper functions for tune measurement and curve fitting. */

struct peak_range { unsigned int left; unsigned int right; };
struct one_pole { double complex a; double complex b; };


/* Returns |z|^2, really ought to be in standard C library. */
double cabs2(double complex z);

/* Returns value of maximum element of array. */
int find_max_val(unsigned int length, const int array[]);

/* Returns centre from fitting quadratic to the given data, fails if the centre
 * is outside the waveform. */
bool fit_quadratic(unsigned int length, const int wf[], double *result);

/* Decodes iq model in (a,b) format into corresponding peak height, width,
 * centre frequency and phase. */
void decode_one_pole(
    const struct one_pole *fit,
    double *height, double *width, double *centre, double *phase);

/* Given a list of discovered peaks (ordered by size) performs sequential fits
 * of one pole filters to each peak.  The following parameters are passed:
 *
 * peak_count   Number of peaks to fit (normally up to 3)
 * tune_scale   Frequency scale
 * wf_i, wf_q   Raw IQ detector data
 * power        |iq|^2, already calculated for convenience
 * ranges       List of ranges for each peak
 * fits         List of fitting results */
unsigned int fit_multiple_peaks(
    unsigned int peak_count, double threshold, bool refine_fit,
    const double tune_scale[], const short wf_i[], const short wf_q[],
    const int power[],
    const struct peak_range ranges[], struct one_pole fits[]);

/* Given index into sweep and the corresponding tune scale computes the tune and
 * phase for the given index. */
void index_to_tune(
    const double tune_scale[], const short wf_i[], const short wf_q[],
    double ix, double *tune, double *phase);


/* Computes second derivative of input curve.  The waveform length is preserved,
 * so the two end points are set to zero. */
void compute_dd(unsigned int length, const int wf_in[], int wf_out[]);

/* Smooths and decimates waveform by factor of four.  Output waveform must be
 * length/4 points long. */
void smooth_waveform_4(unsigned int length, const int wf_in[], int wf_out[]);
