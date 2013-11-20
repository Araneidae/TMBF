/* High level tune processing. */

struct sweep_info;

/* This is called each time a tune sweep completes.  All the waveforms generated
 * by the detector are passed through for detailed tune detection processing. */
void update_tune_sweep(const struct sweep_info *sweep_info, bool overflow);

/* To be called any time the tune setup has changed. */
void tune_setting_changed(void);

bool initialise_tune(void);
