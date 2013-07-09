/* Sequencer sweep and detector control. */

bool initialise_sequencer(void);

/* Called to poll the fast buffer status. */
bool poll_buf_busy(void);

/* Called when the fast buffer has triggered. */
void process_fast_buffer(void);

/* Enable or disable sequencer trigger. */
void enable_seq_trigger(bool enable);
