/* Detector and sweep control. */

#define TUNE_LENGTH    (RAW_BUF_DATA_LENGTH / 4)


/* Information about a single channel of sweep measurement. */
struct channel_sweep {
    short wf_i[TUNE_LENGTH];
    short wf_q[TUNE_LENGTH];
    int power[TUNE_LENGTH];
};

/* Full information about a successful detector sweep. */
struct sweep_info {
    int sweep_length;
    bool single_bunch_mode;
    double tune_scale[TUNE_LENGTH];
    /* Aggregate sweep info for the four individual channels below. */
    struct channel_sweep mean;
    /* Channel specific sweep info. */
    struct channel_sweep channels[4];
};



bool initialise_detector(void);

/* Called immediately before arming and triggering the sequencer so that the
 * hardware settings appropriate to the new scan can be programmed. */
void prepare_detector(bool settings_changed);

/* Converts frequency in tunes (cycles per turn) into fractions of phase advance
 * per clock cycle. */
unsigned int tune_to_freq(double tune);

/* Called on completion of buffer processing in IQ mode.  The packed I and Q
 * components are passed through for processing by the detector. */
void update_iq(
    const short buffer_low[], const short buffer_high[],
    unsigned int sequencer_pc, const struct seq_entry *sequencer_table);
