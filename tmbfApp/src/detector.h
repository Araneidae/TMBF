/* Detector and sweep control. */

#define TUNE_LENGTH    (BUF_DATA_LENGTH / 4)


struct seq_entry;

/* Information about a single channel of sweep measurement. */
struct channel_sweep {
    short wf_i[TUNE_LENGTH];
    short wf_q[TUNE_LENGTH];
    int power[TUNE_LENGTH];
};

/* Full information about a successful detector sweep. */
struct sweep_info {
    unsigned int sweep_length;
    bool single_bunch_mode;
    double tune_scale[TUNE_LENGTH];
    /* Aggregate sweep info for the four individual channels below. */
    struct channel_sweep mean;
    /* Channel specific sweep info. */
    struct channel_sweep channels[4];
};



bool initialise_detector(void);

/* Converts frequency in tunes (cycles per turn) into fractions of phase advance
 * per clock cycle. */
unsigned int tune_to_freq(double tune);

void compute_power(struct channel_sweep *sweep);

/* Called immediately before arming and triggering the sequencer so that the
 * hardware settings appropriate to the new scan can be programmed. */
void prepare_detector(
    bool settings_changed,
    unsigned int sequencer_pc, const struct seq_entry *sequencer_table,
    unsigned int super_count, const uint32_t offsets[]);

/* Called on completion of buffer processing in IQ mode.  The packed I and Q
 * components are passed through for processing by the detector. */
void update_iq(const short buffer_low[], const short buffer_high[]);

/* This is called as part of injection tune processing to forcibly update the
 * tune scale from outside. */
void inject_tune_scale(const double tune_scale[TUNE_LENGTH]);
