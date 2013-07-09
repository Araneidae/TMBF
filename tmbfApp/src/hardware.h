/* Abstraction of TMBF register interface.
 *
 * Mostly a very thin wrapper over the existing registers, largely designed
 * so that the hardware interface can be separated from the code that uses
 * it. */


/* Some standard array size definitions. */
#define MAX_FIR_COEFFS      9       // Length of feedback filter in taps
#define MAX_BUNCH_COUNT     936     // Bunches in a single turn
#define BUF_DATA_LENGTH     16384   // Points in internal fast buffer
#define MAX_SEQUENCER_COUNT 8       // Steps in sequencer
#define FIR_BANKS           4
#define BUNCH_BANKS         4

#define SAMPLES_PER_TURN    MAX_BUNCH_COUNT  //!!!!???? Duplicate to eliminate


/* To be called once at startup. */
bool initialise_hardware(void);


/* Returns version number. */
int hw_read_version(void);


/* * * * * * * * * * * * */
/* ADC: Data Input Stage */

/* Writes ADC offset corrections, one for each ADC channel. */
void hw_write_adc_offsets(short offsets[4]);

/* Reads ADC minimum and maximum values since last reading. */
void hw_read_adc_minmax(short min[MAX_BUNCH_COUNT], short max[MAX_BUNCH_COUNT]);


/* * * * * * * * * * * * * * * */
/* FIR: Filtering for Feedback */

/* Fixed gain of FIR output stage in 3dB steps. */
void hw_write_fir_gain(unsigned int gain);

/* FIR tap coefficients for selected bank.  Taps are signed 18-bit integers. */
void hw_write_fir_taps(int bank, int taps[MAX_FIR_COEFFS]);

/* Returns the number of coefficients in the FIR filters. */
int hw_read_fir_length(void);


/* * * * * * * * * * * * * */
/* DAC: Data Output Stage */

/* Reads DAC minimum and maximum values since last reading. */
void hw_read_dac_minmax(short min[MAX_BUNCH_COUNT], short max[MAX_BUNCH_COUNT]);

/* Output enable. */
void hw_write_dac_enable(unsigned int enable);

/* Output delay in 2ns steps up to 1023 steps. */
void hw_write_dac_delay(unsigned int delay);


/* * * * * * * * * * * * * * * * * * * * * * */
/* DDR: High Speed High Volume Data Capture */

/* Selects data to capture to DDR RAM, either DAC or unprocessed ADC. */
void hw_write_ddr_select(unsigned int select);

/* Arms DDR for data capture. */
void hw_write_ddr_arm(void);

/* Selects software or hardware trigger. */
void hw_write_ddr_trigger_select(unsigned int selection);

/* If software triggering selected then triggers DDR capture. */
void hw_write_ddr_soft_trigger(void);


/* * * * * * * * * * * * * * * * * * * * */
/* BUN: Bunch by Bunch Banked Selection */

struct bunch_entry {
    unsigned int bunch_gain;        // FIR gain for this bunch
    unsigned int output_select;     // Output selection for this bunch
    unsigned int fir_select;        // Filter selection
};

/* Writes the selected bunch entries to the selected bank. */
void hw_write_bun_entry(int bank, struct bunch_entry entries[MAX_BUNCH_COUNT]);

/* Enables bunch counter synchronsation. */
void hw_write_bun_sync(void);


/* * * * * * * * * * * * * * * * * */
/* BUF: High Speed Internal Buffer */

/* Selects data for capture to buffer. */
void hw_write_buf_select(unsigned int selection);

/* Enables capture starting on next trigger. */
void hw_write_buf_arm(void);

/* Selects internal or external triggering.  This trigger can also be used for
 * triggering the sequencer. */
void hw_write_buf_trigger_select(unsigned int selection);

/* If internal triggering selected triggers buffer data capture.  Also triggers
 * sequencer operation if armed. */
void hw_write_buf_soft_trigger(void);

/* Returns current buffer status: true if waiting for data, false if idle. */
bool hw_read_buf_status(void);

/* Reads buffer into two separate 16-bit arrays. */
void hw_read_buf_data(short low[BUF_DATA_LENGTH], short high[BUF_DATA_LENGTH]);


/* * * * * * * * * * * * * * * * * * * * * * */
/* NCO: Fixed Frequency Numerical Oscillator */

/* Sets fixed NCO generator frequency. */
void hw_write_nco_freq(uint32_t freq);

/* Sets fixed NCO generator output gain. */
void hw_write_nco_gain(unsigned int gain);


/* * * * * * * * * * * * * */
/* DET: Frequency Detector */

/* Switches detector between all bunch detection or individual bunch mode. */
void hw_write_det_mode(bool bunch_mode);

/* If bunch mode enabled selects which bunch will be detected in each of the
 * four concurrent channels. */
void hw_write_det_bunches(unsigned int bunch[4]);

/* Configures detector gain. */
void hw_write_det_gain(unsigned int gain);


/* * * * * * * * * * * * * * * * * * * * * */
/* SEQ: Programmed Bunch and Sweep Control */

struct seq_entry {
    unsigned int start_freq;        // NCO start frequency
    unsigned int delta_freq;        // Frequency step for sweep
    unsigned int dwell_time;        // Dwell time at each step
    unsigned int capture_count;     // Number of sweep points to capture
    unsigned int bunch_bank;        // Bunch bank selection
    unsigned int hom_gain;          // HOM output gain
    unsigned int wait_time;         // Extra wait time
};

/* Rewrites the sequencer table.  All entries must be present. */
void hw_write_seq_entries(struct seq_entry entries[MAX_SEQUENCER_COUNT]);

/* Returns current sequencer state. */
unsigned int hw_read_seq_state(void);

/* Resets sequencer. */
void hw_write_seq_reset(void);
