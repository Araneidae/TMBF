/* DDR buffer EPICS interface. */

bool initialise_ddr_epics(void);

/* Called when arming DDR: this is used to disable buffer updates. */
void prepare_ddr_buffer(void);

/* Called on explicit stop of DDR. */
void disarm_ddr_buffer(void);

/* To be called on successful DDR triggering. */
void process_ddr_buffer(void);

/* Called when sequencer has finished processing; this may be of interest to the
 * DDR buffer. */
void notify_ddr_seq_ready(void);
