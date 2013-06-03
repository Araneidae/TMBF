/* Interface to large fast memory buffer. */


#define SAMPLES_PER_TURN    936

/* Initialises resources for access to DDR data. */
bool initialise_ddr(void (*ddr_trigger)(void));

void read_ddr_turns(ssize_t start, size_t turns, int16_t *result);
void read_ddr_bunch(ssize_t start, size_t bunch, size_t turns, int16_t *result);
