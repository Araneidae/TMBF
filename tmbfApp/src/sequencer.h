/* Sequencer sweep and detector control. */

bool initialise_sequencer(void);

/* Called immediately before arming the fast buffer. */
void prepare_fast_buffer(void);

/* Called immediately before arming the sequencer. */
void prepare_sequencer(void);

/* Called when the fast buffer has triggered. */
void process_fast_buffer(void);
