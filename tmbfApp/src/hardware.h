/* Abstraction of TMBF register interface.
 *
 * Mostly a very thin wrapper over the existing registers, largely designed
 * so that the hardware interface can be separated from the code that uses
 * it. */


/* Some standard array size definitions. */
#define BUNCHES_PER_TURN    936     // Bunches in a single turn
#define RAW_BUF_DATA_LENGTH 16384   // Points in internal fast buffer
#define MAX_SEQUENCER_COUNT 7       // Steps in sequencer, not counting state 0
#define FIR_BANKS           4
#define BUNCH_BANKS         4

/* Delay compensation on ADC/FIR/DAC buffer data can result in garbage in the
 * last turn's worth of the buffer, so don't include this in the published and
 * processed data length. */
#define BUF_DATA_LENGTH     (RAW_BUF_DATA_LENGTH - BUNCHES_PER_TURN)


/* To be called once at startup.  The given config_file contains hardware
 * offsets describing the FPGA behaviour. */
bool initialise_hardware(const char *config_file);


/* Returns version number. */
int hw_read_version(void);

/* All bits will be read into overflow_bits[], but only those bits selected in
 * read_bits[] will be updated and correctly reset. */
enum {
    OVERFLOW_FIR        = 0,    // Overflow in FIR gain control output
    OVERFLOW_DAC        = 1,    // Overflow in DAC multiplexor and scaling
    OVERFLOW_DAC_COMP   = 2,    // Overflow in DAC pre-emphasis filter
    OVERFLOW_ADC_LIMIT  = 3,    // ADC input exceeds configured limit

    OVERFLOW_IQ_FIR     = 4,    // Overflow in IQ detector FIR input
    OVERFLOW_IQ_ACC     = 5,    // Overflow in IQ detector accumulator
    OVERFLOW_IQ_SCALE   = 6,    // Overflow in IQ detector readout scaling

    PULSED_BIT_COUNT = 8
};
void hw_read_pulsed_bits(
    const bool read_bits[PULSED_BIT_COUNT],
    bool pulsed_bits[PULSED_BIT_COUNT]);

/* Only useful for testing: loops digital DAC output to digital ADC input. */
void hw_write_loopback_enable(bool loopback);

/* Only useful for testing: disables internal delay compensation. */
void hw_write_compensate_disable(bool disable);

/* Control front panel LED. */
void hw_write_front_panel_led(bool enable);

/* Returns clock dropout detection.  If true returned then the FPGA must be
 * reloaded to restore normal operation. */
bool hw_read_clock_dropout(void);


/* * * * * * * * * * * * */
/* ADC: Data Input Stage */

/* Writes ADC offset corrections, one for each ADC channel. */
void hw_write_adc_offsets(short offsets[4]);

/* Writes ADC compensation FIR. */
void hw_write_adc_filter(short taps[12]);

/* Reads ADC minimum and maximum values since last reading. */
void hw_read_adc_minmax(
    short min[BUNCHES_PER_TURN], short max[BUNCHES_PER_TURN]);

/* Sets low level initial ADC delay. */
void hw_write_adc_skew(unsigned int skew);

/* Writes limit for triggering ADC limit event. */
void hw_write_adc_limit(int limit);


/* * * * * * * * * * * * * * * */
/* FIR: Filtering for Feedback */

/* Fixed gain of FIR output stage in 3dB steps. */
void hw_write_fir_gain(unsigned int gain);

/* FIR tap coefficients for selected bank.  Taps are signed 18-bit integers. */
void hw_write_fir_taps(int bank, int taps[]);

/* Returns the number of coefficients in the FIR filters. */
int hw_read_fir_length(void);


/* * * * * * * * * * * * * */
/* DAC: Data Output Stage */

/* Reads DAC minimum and maximum values since last reading. */
void hw_read_dac_minmax(
    short min[BUNCHES_PER_TURN], short max[BUNCHES_PER_TURN]);

/* Output enable. */
void hw_write_dac_enable(bool enable);

/* Output delay in 2ns steps up to 1023 steps for DAC delay and output delay
 * from 0 to 2 (in 2ns steps) for preemphasis filter. */
void hw_write_dac_delay(unsigned int dac_delay, unsigned int preemph_delay);

/* Pre-emphasis filter. */
void hw_write_dac_preemph(short taps[3]);


/* * * * * * * * * * * * * * * * * * * * * * */
/* DDR: High Speed High Volume Data Capture */

/* Selects data to capture to DDR RAM, either DAC or unprocessed ADC. */
void hw_write_ddr_select(unsigned int select);

/* Starts data capture into DDR RAM.  Must be called before triggering. */
void hw_write_ddr_enable(void);

/* Returns delay (in clocks) from reference bunch to corresponding DDR readout.
 * This depends on the currently selected data source. */
int hw_read_ddr_delay(void);

/* Returns DDR trigger status together with current buffer offset. */
bool hw_read_ddr_status(int *offset);


/* * * * * * * * * * * * * * * * * * * * */
/* BUN: Bunch by Bunch Banked Selection */

struct bunch_entry {
    unsigned int bunch_gain;        // FIR gain for this bunch
    unsigned int output_select;     // Output selection for this bunch
    unsigned int fir_select;        // Filter selection
};

/* Writes the selected bunch entries to the selected bank. */
void hw_write_bun_entry(int bank, struct bunch_entry entries[BUNCHES_PER_TURN]);

/* Enables bunch counter synchronsation. */
void hw_write_bun_sync(void);

/* Configure zero bunch offset for bunch synchronisation. */
void hw_write_bun_zero_bunch(int bunch);

/* Returns zero if bunch counter not yet triggered, otherwise returns
 * sub-trigger phase bits. */
int hw_read_bun_trigger_phase(void);


/* * * * * * * * * * * * * * * * * */
/* BUF: High Speed Internal Buffer */

enum { BUF_SELECT_IQ = 1, BUF_SELECT_DEBUG = 4 };

/* Selects data for capture to buffer. */
void hw_write_buf_select(unsigned int selection);

/* Returns current buffer status: true if waiting for data or trigger, false if
 * idle. */
bool hw_read_buf_status(void);

/* Digest of buf_status above, returns true if buffer expects more data. */
bool hw_read_buf_busy(void);

/* Reads buffer into two separate 16-bit arrays. */
void hw_read_buf_data(
    int raw[RAW_BUF_DATA_LENGTH],
    short low[RAW_BUF_DATA_LENGTH], short high[RAW_BUF_DATA_LENGTH]);


/* * * * * * * * * * * * * * * * * * * * * * */
/* NCO: Fixed Frequency Numerical Oscillator */

/* Sets fixed NCO generator frequency. */
void hw_write_nco_freq(uint32_t freq);

/* Sets fixed NCO generator output gain. */
void hw_write_nco_gain(unsigned int gain);


/* * * * * * * * * * * * * */
/* DET: Frequency Detector */


enum { DET_IN_ADC = 0, DET_IN_FIR = 1 };

/* Switches detector between all bunch detection or individual bunch mode. */
void hw_write_det_mode(bool bunch_mode);

/* Switch between ADC and FIR input for detector. */
void hw_write_det_input_select(unsigned int input);

/* If bunch mode enabled selects which bunch will be detected in each of the
 * four concurrent channels. */
void hw_write_det_bunches(unsigned int bunch[4]);

/* Configures detector gain. */
void hw_write_det_gain(unsigned int gain);

/* Writes the detector window waveform. */
#define DET_WINDOW_LENGTH   1024
void hw_write_det_window(uint16_t window[DET_WINDOW_LENGTH]);

/* Returns measured detector phase delays (relative to a full turn of delay) for
 * ADC and FIR measurements, results are reported in bunches. */
void hw_read_det_delays(int *adc_delay, int *fir_delay);


/* * * * * * * * * * * * * * * * * * * * * */
/* FTUN: Fast Tune Following and Detector  */

#define FTUN_FIFO_SIZE  1023

enum { FTUN_IN_ADC = 0, FTUN_IN_FIR = 1 };

struct ftun_control {
    int dwell;                  // Dwell time in turns
    bool blanking;              // Blanking enable
    bool multibunch;            // Single or multibunch selector
    int bunch;                  // Bunch and channel selection for detector
    unsigned int input_select;  // ADC or FIR input (0 for ADC input)
    unsigned int det_gain;      // Detector gain
    int target_phase;           // Target phase for feedback
    unsigned int iir_rate;      // IIR feedback factor (1-2^N)
    int i_scale;                // Integral Scaling factor
    int p_scale;                // Proportional scaling factor
    int min_magnitude;          // Minimum magnitude for feedback
    int max_offset;             // Maximum frequency offset for feedback

    unsigned int iq_iir_rate;   // IIR rate for IQ readback
    unsigned int freq_iir_rate;
};

/* Writes given settings to FTUN. */
void hw_write_ftun_control(struct ftun_control *control);

/* Controls whether tune following is enabled. */
void hw_write_ftun_enable(bool enable);

/* Initiates tune following. */
void hw_write_ftun_start(void);

/* Enables triggered tune following start. */
void hw_write_ftun_arm(void);

/* Disarms triggered tune following start. */
void hw_write_ftun_disarm(void);

/* Returns current operational status of tune following: one of not running,
 * armed and waiting for trigger, or running. */
enum ftun_status { FTUN_STOPPED, FTUN_ARMED, FTUN_RUNNING };
enum ftun_status hw_read_ftun_status(void);

/* Read current tune following status into an array of bool. */
enum {
    // These five status bits record the current status
    FTUN_STAT_VAL,              // Integrated frequency offset out of range
    FTUN_STAT_MAG,              // Magnitude of signal too small
    FTUN_STAT_DET,              // Detector output overflow
    FTUN_STAT_ACC,              // Detector accumulator overflow
    FTUN_STAT_INP,              // FIR detector input overflow

    FTUN_STAT_RUNNING,          // Set if tune following is running

    // These five bits record the status when tune following was halted
    FTUN_STOP_VAL = 8,
    FTUN_STOP_MAG,
    FTUN_STOP_DET,
    FTUN_STOP_ACC,
    FTUN_STOP_INP,

    FTUN_BIT_COUNT = FTUN_STOP_INP + 1  // Number of bits in array
};
void hw_read_ftun_status_bits(bool status[FTUN_BIT_COUNT]);

/* Reads snapshot of current (filtered) IQ value. */
void hw_read_ftun_iq(int *I, int *Q);

/* Reads current frequency offset.  Returns true if frequency offset active,
 * false if NCO set to programmed frequency.  The frequency returned is filtered
 * with 12 extra bits of precision. */
bool hw_read_ftun_frequency(int *frequency);

/* Reads minimum and maximum IQ angle values since last reading, returns false
 * if no update in the interval. */
bool hw_read_ftun_i_minmax(int *min, int *max);
bool hw_read_ftun_q_minmax(int *min, int *max);

/* Reads up to FTUN_FIFO_SIZE words from the FTUN FIFO into buffer.  Sets the
 * dropout flag if the FTUN buffer overflowed and returns the number of words
 * read. */
size_t hw_read_ftun_buffer(int buffer[FTUN_FIFO_SIZE], bool *dropout);

/* Returns ADC and FIR detector delays. */
void hw_read_ftun_delays(int *adc_delay, int *fir_delay);


/* * * * * * * * * * * * * * * * * * * * * */
/* SEQ: Programmed Bunch and Sweep Control */

struct seq_entry {
    unsigned int start_freq;        // NCO start frequency
    unsigned int delta_freq;        // Frequency step for sweep
    unsigned int dwell_time;        // Dwell time at each step
    unsigned int capture_count;     // Number of sweep points to capture
    unsigned int bunch_bank;        // Bunch bank selection
    unsigned int hom_gain;          // HOM output gain
    unsigned int window_rate;       // Detector window advance frequency
    bool enable_window;             // Enable detector windowing
    bool write_enable;              // Enable data capture of sequence
    bool enable_blanking;           // Observe trigger holdoff control
    unsigned int holdoff;           // Detector holdoff
};

/* Rewrites the sequencer table.  All entries must be present. */
void hw_write_seq_entries(
    unsigned int bank0, struct seq_entry entries[MAX_SEQUENCER_COUNT]);

/* Programs sequencer program counter.  The sequencer will run the next time the
 * buffer is armed. */
void hw_write_seq_count(int sequencer_pc);

/* Configure sequencer state which will generate trigger. */
void hw_write_seq_trig_state(int state);

/* Returns current sequencer state. */
unsigned int hw_read_seq_state(void);

/* Returns true if sequencer busy, either waiting for trigger or running. */
bool hw_read_seq_status(void);

/* Resets sequencer. */
void hw_write_seq_reset(void);


/* * * * * * * * * * * * */
/* TRG: Trigger control. */

/* Simultaneously arm one or both of DDR and BUF. */
void hw_write_trg_arm(bool ddr, bool buf);

/* Simultaneously soft trigger one or both of DDR and BUF. */
void hw_write_trg_soft_trigger(bool ddr, bool buf);

/* Disarm pending trigger.  No effect if already triggered or not armed. */
void hw_write_trg_disarm(bool ddr, bool buf);

/* Configure DDR trigger delay. */
void hw_write_trg_ddr_delay(unsigned int ddr_delay);

/* Configure buffer trigger delay. */
void hw_write_trg_buf_delay(unsigned int buf_delay);

/* Configure blanking interval after external trigger in turns. */
void hw_write_trg_blanking(unsigned int trigger_blanking);

/* Configure trigger or SCLK connector as blanking source. */
void hw_write_trg_blanking_source(unsigned int source);

/* Arms next capture of trigger. */
void hw_write_trg_arm_raw_phase(void);

/* Configure DDR trigger source. */
void hw_write_trg_ddr_source(unsigned int source, bool blanking);

/* Returns raw phase bits from trigger. */
int hw_read_trg_raw_phase(void);
