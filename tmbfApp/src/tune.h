/* High level tune processing. */

struct sweep_info;
struct channel_sweep;

/* This is called each time a tune sweep completes.  All the waveforms generated
 * by the detector are passed through for detailed tune detection processing. */
void update_tune_sweep(const struct sweep_info *sweep_info, bool overflow);

/* To be called any time the tune setup has changed. */
void tune_setting_changed(void);


/* Returns value of maximum element of array. */
int find_max_val(int length, const int array[]);

/* Returns centre from fitting quadratic to the given data, fails if the centre
 * is outside the waveform. */
bool fit_quadratic(int length, const int wf[], double *result);

/* Given index into sweep and the corresponding tune scale computes the tune and
 * phase for the given index. */
void index_to_tune(
    const struct channel_sweep *sweep, const double tune_scale[],
    double ix, double *tune, double *phase);


/* Tune status enumerations used when reporting tune to users. */
enum tune_status {
    TUNE_INVALID,       // Not calculated
    TUNE_OK,            // Satisfactory result
    TUNE_NO_PEAK,       // Peak not found
    TUNE_EXTRA_PEAKS,   // Too many peaks found
    TUNE_BAD_FIT,       // Misshapen peak
    TUNE_OVERFLOW,      // Detector input overflow
    TUNE_RANGE,         // Alarm range error
};



bool initialise_tune(void);
