/* Called on each successful tune sweep to update peak tune processing. */
void process_peaks(
    unsigned int length, const struct channel_sweep *sweep,
    const double *tune_scale);

bool initialise_tune_peaks(void);
