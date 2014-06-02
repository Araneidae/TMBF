/* DDR buffer EPICS interface. */

bool initialise_ddr_epics(void);

/* To be called on successful DDR triggering. */
void process_ddr_buffer(void);

/* Called when arming DDR: this is used to disable buffer updates. */
void arming_ddr_buffer(void);
