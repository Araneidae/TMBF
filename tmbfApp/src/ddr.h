/* Interface to large fast memory buffer. */


#define SAMPLES_PER_TURN    936

/* Initialises resources for access to DDR data. */
bool initialise_ddr(void (*ddr_trigger)(void));

/* Reads the given number of complete turns from the given offset from the
 * trigger point. */
void read_ddr_turns(ssize_t start, size_t turns, int16_t *result);

/* Reads the given number of turns for a complete bunch from the given offset
 * from the trigger point. */
void read_ddr_bunch(ssize_t start, size_t bunch, size_t turns, int16_t *result);
