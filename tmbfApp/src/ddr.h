/* Interface to large fast memory buffer. */


/* Initialises resources for access to DDR data. */
bool initialise_ddr(void);

/* Reads the given number of complete turns from the given offset from the
 * trigger point. */
void read_ddr_turns(ssize_t start, size_t turns, int16_t *result);

/* Reads the given number of turns for a complete bunch from the given offset
 * from the trigger point. */
void read_ddr_bunch(ssize_t start, size_t bunch, size_t turns, int16_t *result);

/* To be called regularly.  Returns true if a DDR trigger has been fully
 * received and is ready to be processed. */
bool poll_ddr_trigger(void);
