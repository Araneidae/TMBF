/* DDR buffer EPICS interface. */

bool initialise_ddr_epics(void);

/* To be called on successful DDR triggering.  The one_shot flag indicates
 * whether this is part of a recurring trigger event. */
void process_ddr_buffer(bool one_shot);

/* Called when arming DDR: this is used to disable buffer updates. */
void arming_ddr_buffer(void);
