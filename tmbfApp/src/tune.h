/* High level tune processing. */

struct sweep_info;

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


/* This is called each time a tune sweep completes.  All the waveforms generated
 * by the detector are passed through for detailed tune detection processing. */
void update_tune_sweep(struct sweep_info *sweep_info, bool overflow);

/* Called by tune PLL on tune updates. */
void update_tune_pll_tune(bool tune_ok, double tune, double phase);

bool initialise_tune(void);
