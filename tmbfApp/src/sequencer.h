/* Sequencer sweep and detector control. */

bool initialise_sequencer(void);

/* Called when the fast buffer has triggered. */
void process_fast_buffer(void);

/* Called immediately before arming the sequencer. */
void prepare_sequencer(bool enable_sequencer);

/* Returns currently programmed sequencer table and current sequencer state
 * count. */
const struct seq_entry *read_sequencer_table(unsigned int *state_count);
