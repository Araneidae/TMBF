/* Called once on startup to initialise tune following support. */
bool initialise_tune_follow(void);

/* Called when a buffer DEBUG update occurs. */
void update_tune_follow_debug(const int *buffer_raw);
