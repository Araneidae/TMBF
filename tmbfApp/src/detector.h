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
