/* Detector and sweep control. */

bool initialise_detector(void);

/* Internally records any changes to sequencer settings.  Used to update the
 * detector frequency scale as necessary. */
void seq_settings_changed(void);

/* Converts frequency in tunes (cycles per turn) into fractions of phase advance
 * per clock cycle. */
unsigned int tune_to_freq(double tune);

/* Called on completion of buffer processing in IQ mode.  The packed I and Q
 * components are passed through for processing by the detector. */
void update_iq(const short buffer_low[], const short buffer_high[]);


#define TUNE_LENGTH    (RAW_BUF_DATA_LENGTH / 4)


/* Information about a single channel of sweep measurement. */
struct channel_sweep_info {
    short wf_i[TUNE_LENGTH];
    short wf_q[TUNE_LENGTH];
    int power[TUNE_LENGTH];
};

/* Full information about a successful detector sweep. */
struct sweep_info {
    int sweep_length;
    float tune_scale[TUNE_LENGTH];
    /* Aggregate sweep info for the four individual channels below. */
    struct channel_sweep_info mean;
    /* Channel specific sweep info. */
    struct channel_sweep_info channels[4];
};

