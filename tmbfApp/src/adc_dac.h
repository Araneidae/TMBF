/* ADC EPICS interface. */

bool initialise_adc_dac(void);

/* As part of bunch synchronisation we update ADC skew. */
void set_adc_skew(unsigned int skew);
