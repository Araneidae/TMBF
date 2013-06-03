/* Abstraction of TMBF register interface.
 *
 * Mostly a very thin wrapper over the existing registers, largely designed
 * so that the hardware interface can be separated from the code that uses
 * it. */


/* Some standard array size definitions. */
#define MAX_FIR_COEFFS      10
#define MAX_BUNCH_COUNT     936
#define MAX_DATA_LENGTH     16384



/* Register read access. */
#define DECLARE_REGISTER_R(name) \
    unsigned int read_##name(void)

/* Register write access. */
#define DECLARE_REGISTER_W(name) \
    void write_##name(unsigned int value)

/* Each configuration DECLARE_REGISTER has read and write access routines. */
#define DECLARE_REGISTER(name) \
    DECLARE_REGISTER_R(name); \
    DECLARE_REGISTER_W(name)

/* Pulses a control bit. */
#define DECLARE_PULSE(name) \
    void pulse_##name(void)


/* Some of these will not be published longer term, others will change... */

/* Registers corresponding to control register fields. */
DECLARE_REGISTER(CTRL_DAC_ENA);     // DAC output selection
DECLARE_REGISTER(CTRL_SOFT_TRIG);   // Writing this creates soft trigger event
DECLARE_REGISTER(CTRL_ARCHIVE);     // Select signal to write to fast buffers
DECLARE_REGISTER(CTRL_FIR_GAIN);    // Select gain of FIR output
DECLARE_REGISTER(CTRL_HOM_GAIN);    // Select gain for HOM oscillator output
DECLARE_REGISTER(CTRL_TRIG_SEL);    // Select soft or external trigger
DECLARE_REGISTER(CTRL_ARM_SEL);     // Select soft or external arming
DECLARE_REGISTER(CTRL_SOFT_ARM);    // Write to arm the soft trigger
DECLARE_REGISTER(CTRL_GROW_DAMP);   // Set to enable grow damp test
DECLARE_REGISTER(CTRL_DDR_INPUT);   // Select data to write to fast DDR memory
DECLARE_REGISTER(CTRL_CH_SELECT);   // Channel readout selection
DECLARE_REGISTER(CTRL_DDC_INPUT);   // Select input to DDC
DECLARE_REGISTER(CTRL_IQ_SCALE);    // DDC output gain
DECLARE_REGISTER(CTRL_BUNCH_SYNC);  // Bunch synchronisation

DECLARE_PULSE(CTRL_ARM_DDR);
DECLARE_PULSE(CTRL_TRIG_DDR);

/* Delay register fields. */
DECLARE_REGISTER(DELAY_DAC);        // DAC output delay (in 2ns intervals)
DECLARE_REGISTER(DELAY_TUNE_SWEEP); // Enable tune sweep
DECLARE_REGISTER(DELAY_GROW_DAMP);  // Configure delay for grow damp test

/* Other registers. */
DECLARE_REGISTER(BunchSelect);      // Bunch number in single bunch mode
DECLARE_REGISTER(DDC_dwellTime);    // DDC dwell time in 8ns units
DECLARE_REGISTER(NCO_frequency);    // Fixed frequency when HOM not scanning
DECLARE_REGISTER_R(FPGA_version);   // FPGA version number
DECLARE_REGISTER(SweepStartFreq);   // Sweep start frequency
DECLARE_REGISTER(SweepStopFreq);    // Sweep stop frequency
DECLARE_REGISTER(SweepStep);        // Sweep frequency advance
DECLARE_REGISTER(AdcOffAB);         // Hideous direct access to registers that
DECLARE_REGISTER(AdcOffCD);         // --- really needs to be wrapped!


/* Routines to read and write the FIR coefficients. */
void read_FIR_coeffs(int coeffs[MAX_FIR_COEFFS]);
void write_FIR_coeffs(const int coeffs[MAX_FIR_COEFFS]);

/* Read ADC minimum and maximum values. */
void read_ADC_MinMax(
    short int ADC_min[MAX_BUNCH_COUNT], short int ADC_max[MAX_BUNCH_COUNT]);

/* Read DAC minimum and maximum values. */
void read_DAC_MinMax(
    short int DAC_min[MAX_BUNCH_COUNT], short int DAC_max[MAX_BUNCH_COUNT]);


void read_DataSpace(
    short int HighData[MAX_DATA_LENGTH], short int LowData[MAX_DATA_LENGTH]);

/* The bunch-by-bunch gain and DAC settings share the same registers, but we
 * conceal this in the API below. */
void read_bunch_configs(
    short int bunch_gains[MAX_BUNCH_COUNT],
    short int bunch_dacs[MAX_BUNCH_COUNT],
    short int bunch_tempdacs[MAX_BUNCH_COUNT]);

void write_BB_gains(short int gains[MAX_BUNCH_COUNT]);
void write_BB_DACs(short int dacs[MAX_BUNCH_COUNT]);
void write_BB_TEMPDACs(short int dacs[MAX_BUNCH_COUNT]);


void set_softTrigger(void);
void set_bunchSync(void);



bool InitialiseHardware(void);


#undef DECLARE_REGISTER
