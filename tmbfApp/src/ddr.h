/* Interface to large fast memory buffer. */


/* Initialises resources for access to DDR data. */
bool initialise_ddr(void);

/* Reads the given number of complete turns from the given offset from the
 * trigger point.  Note: can fail if attempted while DDR capture still in
 * progress. */
bool read_ddr_turns(ssize_t start, size_t turns, int16_t *result);

/* Reads the given number of turns for a complete bunch from the given offset
 * from the trigger point.  As for read_ddr_turns, can fail if DDR busy. */
bool read_ddr_bunch(ssize_t start, size_t bunch, size_t turns, int16_t *result);
