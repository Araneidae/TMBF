/* Called on each successful tune sweep to update peak tune processing. */
void measure_tune_peaks(
    unsigned int length, const struct channel_sweep *sweep,
    const double *tune_scale,
    unsigned int *status, double *tune, double *phase);

bool initialise_tune_peaks(void);
