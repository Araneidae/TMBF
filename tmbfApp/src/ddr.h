/* Interface to large fast memory buffer. */


/* Initialises resources for access to DDR data. */
bool initialise_ddr(void);

/* Reads the given number of complete turns from the given offset from the
 * trigger point. */
void read_ddr_turns(ssize_t start, size_t turns, int16_t *result);

/* Reads the given number of turns for a complete bunch from the given offset
 * from the trigger point. */
void read_ddr_bunch(ssize_t start, size_t bunch, size_t turns, int16_t *result);

/* Configures offset into DDR buffer, to be called in response to successful
 * call to hw_read_ddr_status. */
void set_ddr_offset(int ddr_offset);
