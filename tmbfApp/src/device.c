#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <math.h>

#include <iocsh.h>      // Needed for debug command

#include "GenericDevice.h"

//#include "tunes.h"
#include "device.h"


/* Hardware field definitions.  These are offset, length pairs designed to be
 * used with LiberaCtrl() and LiberaDelay(). */

#define CTRL_DAC_OUT        0, 2
#define CTRL_FIR_INVERT     2, 1
#define CTRL_SOFT_TRIG      3, 1
#define CTRL_ARCHIVE        4, 2
#define CTRL_FIR_GAIN       6, 4
#define CTRL_HOM_GAIN       10, 4
#define CTRL_TRIG_SEL       14, 1
#define CTRL_ARM_SEL        15, 1
#define CTRL_SOFT_ARM       16, 1
#define CTRL_GROW_DAMP      17, 1
#define CTRL_TEMP_DAC_OUT   18, 3
#define CTRL_DDR_INPUT      21, 1
#define CTRL_SET_PLANE      22, 1
#define CTRL_CH_SELECT      23, 2
#define CTRL_DDC_INPUT      25, 1
#define CTRL_BUNCH_MODE     26, 1
#define CTRL_IQ_SCALE       27, 3
#define CTRL_BUNCH_SYNC     30, 1

#define DELAY_DAC           0, 10
#define DELAY_TUNE_SWEEP    18, 1
#define DELAY_GROW_DAMP     24, 8



#define MAP_SIZE (1 << 14)
#define MAP_MASK (MAP_SIZE - 1)

#define ASSERT(e) if(!(e)) AssertFail(__FILE__, __LINE__)

void AssertFail(const char * FileName, int LineNumber)
{
    fprintf(stderr, "Error at line %d, file %s (%d) [%s]\n",
            LineNumber, FileName, errno, strerror(errno));
    exit(1);
}


#define TEST_IO(var, error, command, args...) \
    ( { \
        var = (command)(args); \
        if ((int) var == -1)  perror(error); \
        (int) var != -1; \
    } )

#define SQR(x)  ((x)*(x))



#define FF_CONFIG_ADDRESS       0x1402C000
#define FF_DATA_ADDRESS         0x14028000

typedef struct 
{
    unsigned int Ctrl;                  // 000  Main control register
    unsigned int Bunch;                 // 004  Bunch number selection
    unsigned int ProgClkVal;            // 008  DDC dwell time (in 8ns clocks)
    unsigned int Nco;                   // 00C  Fixed NCO output frequency
    unsigned int Status;                // 010  FPGA version number
    unsigned int Delay;                 // 014  DAC output delay (in 2ns steps)
    unsigned int SwpStartFreq;          // 018  Sweep start
    unsigned int SwpStopFreq;           // 01C  --- and stop frequecies
    int          Coeffs[12];            // 020  FIR coefficents
    unsigned int AdcOffAB;              // 050  Used to correct DC offsets
    unsigned int AdcOffCD;              // 054  --- between four ADC channels.
    unsigned int AdcChnSel;             // 058  Select ADC readout channel
    unsigned int DacChnSel;             // 05C  Select DAC readout channel
    unsigned int PhaseAdvStep;          // 060  Sweep frequency step
    int          DummyArry[231];        // 064  (padding)
    int          BB_Gain_Coeffs[256];   // 400  Bunch gain control
    int          BB_Adc_MinMax[256];    // 800  ADC readout
    int          BB_Dac_MinMax[256];    // C00  DAC readout
} FF_CONFIG_SPACE;

/* These three pointers directly overlay the FF memory. */
static FF_CONFIG_SPACE * ConfigSpace;
int * DataSpace;


static bool MapFastFeedbackMemory()
{
    int mem = open("/dev/mem", O_RDWR | O_SYNC);
    ASSERT(mem != -1);
    
    /* Map FF_CONFIG page */
    char * map_config_base = (char *) mmap(
        0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
        mem, FF_CONFIG_ADDRESS & ~MAP_MASK);
    ASSERT(map_config_base != (void *) -1);
    ConfigSpace = (FF_CONFIG_SPACE *)
        (map_config_base + (FF_CONFIG_ADDRESS & MAP_MASK));

    /* Map FF_DATA page */
    char * map_data_base = (char *) mmap(
        0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
        mem, FF_DATA_ADDRESS & ~MAP_MASK);
    ASSERT(map_data_base != (void *) -1);
    DataSpace = (int *) (map_data_base + (FF_DATA_ADDRESS & MAP_MASK));
 
    return 0;
}

static void generic_reg_read(Variant *v)
{
    *(v->buffer) = *((unsigned int *) v->usr); /*   ConfigSpace->Ctrl; */
}

static void generic_reg_write(Variant *v)
{
    *((unsigned int *) v->usr) = (unsigned int) *(v->buffer);
}

static void generic_buf_read(Variant *v)
{
    memcpy(v->buffer, (int *) v->usr, v->length * sizeof(int));
}

static void generic_buf_write(Variant *v)
{
    memcpy((int *) v->usr, v->buffer, v->length * sizeof(int));
}

/* Libera ctrl register write
 *
 * val is written into the bit-field of *reg of length specified by numbits
 * and with bit offset specified by startbit. */

static void LiberaConfigure(unsigned int * reg, unsigned int startbit,
    unsigned int numbits, unsigned int val)
{
    /* Create bit mask with the appropriate bits set. */
    unsigned int mask = ((1 << numbits) - 1) << startbit;
    /* Perform the register update: read the register, mask out the bits
     * we're changing, write the new field. */
    *reg = (*reg & ~mask) | ((val << startbit) & mask);
}

static void LiberaCtrl(
    unsigned int startbit, unsigned int numbits, unsigned int val)
{
    LiberaConfigure(&ConfigSpace->Ctrl, startbit, numbits, val);
}

static void LiberaDelay(
    unsigned int startbit, unsigned int numbits, unsigned int val)
{
    LiberaConfigure(&ConfigSpace->Delay, startbit, numbits, val);
}


#if 0
static void adc_minmax_read(Variant *v)
{
    int n, i;
    int * buf = (int *)v->buffer;
    volatile int * src = (volatile int *)v->usr;

    for (n = 0; n < 4; n++) {
        ConfigSpace->AdcChnSel = 2*n+1; /* Channel select and read enable */
        for (i = 0; i<234; i++) 
            buf[i*4+n] = src[i];    
    }
    ConfigSpace->AdcChnSel = 0;
}
#endif

static void dac_minmax_read(Variant *v)
{
    int n, i;
    int * buf = (int *)v->buffer;
    volatile int * src = (volatile int *)v->usr;

    for (n = 0; n < 4; n++) {
        ConfigSpace->DacChnSel = 2*n+1; /* Channel select and read enable */
        for (i = 0; i<234; i++) 
            buf[i*4+n] = src[i];    
    }
    ConfigSpace->DacChnSel = 0;
}

/*
 * Write BB gain&conf values as a single waveform
 */
static void bb_gain_and_dac_write(Variant *v)
{
    int * buf = (int *)v->buffer;
    volatile int * src = (volatile int *)v->usr;

    for (int n=0; n<4; n++) {
        LiberaCtrl(CTRL_CH_SELECT, n);
        for (int i=0; i<234; i++) 
            src[i] = buf[i*4+n];
    } 
}


static void bb_write_gain(Variant *v)
{
    float * gains = (float *) v->buffer;
    unsigned int * src = (unsigned int *)v->usr;

    for (int n=0; n<4; n++)
    {
        LiberaCtrl(CTRL_CH_SELECT, n);
        for (int i=0; i<234; i++)
        {
            float gain = gains[4*i + n];
            int gain_int =
                gain >= 1.0  ? 32767 :
                gain <= -1.0 ? -32767 : (int) (32767 * gain);
            int current = src[i];
            src[i] = (current & 0xffff0000) | (gain_int & 0x0000ffff);
        }
    }
}


static void bb_write_dac(Variant *v)
{
    unsigned short * dacs = (unsigned short *) v->buffer;
    unsigned int * src = (unsigned int *)v->usr;

    for (int n=0; n<4; n++)
    {
        LiberaCtrl(CTRL_CH_SELECT, n);
        for (int i=0; i<234; i++)
        {
            int current = src[i];
            src[i] = ((dacs[4*i + n] & 0x0003) << 16) | (current & 0x0000ffff);
        }
    }
}


/*
 * Read on-chip history buffer as a single waveform
 */
static void hb_buf_read(Variant *v)
{
    int * buf = (int *)v->buffer;
    volatile int * src = (volatile int *)v->usr;

    for (int n=0; n<4; n++) {
        LiberaCtrl(CTRL_CH_SELECT, n);
        for (int i=0; i<4096; i++) 
            buf[i*4+n] = src[i];
    }
}

/*
 * Read min and max ADC buffers individually as single waveforms
 */
typedef struct 
{
    short int upper;
    short int lower;
} packed_data;

packed_data minmax_buf[936];

/* 1-) ADC minbuf read trigger adc max_buf read via epics FLNK */
static void adc_minbuf_read(Variant *v)
{
    unsigned int n, i;
    short int * buf = (short int *)v->buffer;
    volatile int * src = (volatile int *)v->usr;

    for (n = 0; n < 4; n++) {
        /* Channel select and read enable for ADC */
        ConfigSpace->AdcChnSel = 2*n+1; 
        for (i = 0; i<234; i++) 
            (*(int *)&minmax_buf[i*4+n]) = src[i];    
    }
    ConfigSpace->AdcChnSel = 0;

    for(i = 0; i < v->length; ++i) {
        buf[i] = minmax_buf[i].lower;
    } 
}

/* 2-) ADC maxbuf triggers adc min max diff */
static void adc_maxbuf_read(Variant *v)
{
    unsigned int i;
    short int * buf = (short int *)v->buffer;

    for(i = 0; i < v->length; ++i) {
        buf[i] = minmax_buf[i].upper;
    } 
}

float mean_diff;
float var_diff; 

/* 3-) ADC diffbuf triggers mean and std calculations */
static void adc_diffbuf_read(Variant *v)
{
    short int * buf = (short int *)v->buffer;

    int sum_diff = 0;
    long long int sum_var = 0;

    for(unsigned int i = 0; i < v->length; ++i) {
        short int diff     = minmax_buf[i].lower - minmax_buf[i].upper;
        buf[i]    = diff; 
        sum_diff += diff;
        sum_var  += (int) diff*diff;
    }
    
    mean_diff = sum_diff / 936.0;
    var_diff = sum_var / 936.0 - mean_diff*mean_diff;
}

/* 4-) ADC diffbuf triggers mean and std calculations */
static void get_adcmean(Variant *v)
{
    v->buffer[0] = mean_diff;
}

static void get_adcstd(Variant *v)
{
    v->buffer[0] = var_diff;
}

/*
 * Read on-chip history buffer as a single waveform
 */
packed_data hb_buf[16384];

static void hb_buf_lower_read(Variant *v)
{
    unsigned int ctrl, dummy;
    unsigned int n, i;
    short int * buf = (short int *)v->buffer;
    volatile int * src = (volatile int *)v->usr;

    for (n=0; n<4; n++) {
        ctrl = ConfigSpace->Ctrl;              /* read ctrl register */
        ctrl &= ~(1<<23);                      /* clear channel_sel bits */
        ctrl &= ~(1<<24);    
        dummy = n<<23;
        ConfigSpace->Ctrl = ctrl | dummy;      /* set channel_sel bits */

        for (i=0; i<4096; i++) 
            (*(int *)&hb_buf[i*4+n]) = src[i];    
    }

    for(i = 0; i < v->length; ++i) {
        buf[i] = hb_buf[i].lower;
    } 
}

static void hb_buf_upper_read(Variant *v)
{
    unsigned int i;
    short int * buf = (short int *)v->buffer;

    for(i = 0; i < v->length; ++i) {
        buf[i] = hb_buf[i].upper;
    } 
}

/* Declares a function for writing to a control register. */
#define LIBERA_CTRL(function, register) \
    static void function(Variant *v) \
    { \
        LiberaCtrl(register, *(unsigned int *)v->buffer); \
    }

LIBERA_CTRL(set_dacout,         CTRL_DAC_OUT)
LIBERA_CTRL(set_tempdacout,     CTRL_TEMP_DAC_OUT)
LIBERA_CTRL(set_firgain,        CTRL_FIR_GAIN)
LIBERA_CTRL(set_homgain,        CTRL_HOM_GAIN)
LIBERA_CTRL(set_firinvert,      CTRL_FIR_INVERT)
LIBERA_CTRL(set_archive,        CTRL_ARCHIVE)
LIBERA_CTRL(set_chselect,       CTRL_CH_SELECT)
LIBERA_CTRL(set_trigsel,        CTRL_TRIG_SEL)
LIBERA_CTRL(set_armsel,         CTRL_ARM_SEL)
LIBERA_CTRL(set_ddcinput,       CTRL_DDC_INPUT)
LIBERA_CTRL(set_ddrinput,       CTRL_DDR_INPUT)
LIBERA_CTRL(set_bunchmode,      CTRL_BUNCH_MODE)
LIBERA_CTRL(set_growdampmode,   CTRL_GROW_DAMP)



static void set_tunesweepmode(Variant *v)
{
    unsigned int * buf = (unsigned int *)v->buffer;
    LiberaDelay(DELAY_TUNE_SWEEP, buf[0]);
}


static void set_dacdly(Variant *v)
{
    LiberaDelay(DELAY_DAC, (unsigned int)v->buffer[0]);
}

static void set_growdampperiod(Variant *v)
{
    LiberaDelay(DELAY_GROW_DAMP, (unsigned int)v->buffer[0]);
}

static void set_iqscale(Variant *v)
{
    LiberaCtrl(CTRL_IQ_SCALE, (unsigned int)v->buffer[0]);
}


/* Processing SOFTTRIG forces a soft triggered data capture. */
static void set_softtrig(Variant *v)
{
    /* The sequence is as follows:
     *  1. Select soft trigger
     *  2. Select soft arming
     *  3. Arm (by writing 1 then 0)
     *  4. Trigger (ditto) */
/*     LiberaCtrl(CTRL_TRIG_SEL,  0);  // Select soft trigger */
/*     LiberaCtrl(CTRL_ARM_SEL,   0);  // Select soft arm */

    LiberaCtrl(CTRL_SOFT_ARM,  0);
    usleep(1000);
    LiberaCtrl(CTRL_SOFT_ARM,  1);
    usleep(1000);
    LiberaCtrl(CTRL_SOFT_ARM,  0);
    usleep(1000);

    LiberaCtrl(CTRL_SOFT_TRIG, 0);
    usleep(1000);
    LiberaCtrl(CTRL_SOFT_TRIG, 1);
    usleep(1000);
    LiberaCtrl(CTRL_SOFT_TRIG, 0);
}

/* Processing SOFTARM configures hardware triggering (otherwise what's the
 * point?) */
static void set_softarm(Variant *v)
{
#if 1
    unsigned int * buf = (unsigned int *)v->buffer;
    LiberaCtrl(CTRL_SOFT_ARM, buf[0]);
#else
    LiberaCtrl(CTRL_TRIG_SEL,  1);  /* Select hard trigger */
    LiberaCtrl(CTRL_ARM_SEL,   1);  /* Select soft arm */
    LiberaCtrl(CTRL_SOFT_ARM,  0);  /* Force arm through a rising edge */
    LiberaCtrl(CTRL_SOFT_ARM,  1);
#endif
}

static void set_bunchsync(Variant *v)
{
#if 0
    unsigned  = (unsigned int *)v->buffer;
    LiberaCtrl(CTRL_BUNCH_SYNC, buf[0]);
#else
    LiberaCtrl(CTRL_BUNCH_SYNC,  0);
    usleep(1000);
    LiberaCtrl(CTRL_BUNCH_SYNC,  1);
    usleep(1000);
    LiberaCtrl(CTRL_BUNCH_SYNC,  0);
#endif       
}

static unsigned int fir_cycles = 1;
static unsigned int fir_length = 1;
static double fir_phase = 0;

#define MAX_FIR_LENGTH  9

static void set_fircoeffs()
{
    double tune = (double) fir_cycles / fir_length;    
    LiberaCtrl(CTRL_SET_PLANE, 1);

    /* Only work on the active part of the filter, leaving the beginning of
     * the filter padded with zeros. */
    int fir_start = MAX_FIR_LENGTH - fir_length;

    int coeffs[MAX_FIR_LENGTH];
    memset(coeffs, 0, sizeof(coeffs));

    /* Calculate FIR coeffs and the mean value.  We pad the beginning of the
     * buffer with zeros to ensure zero extra delay from the filter. */
    int sum = 0;
    double max_int = pow(2, 31) - 1;
    for (unsigned int i = 0; i < fir_length; i++)
    {
        int tap = (int) round(max_int * 
            sin(2*M_PI * (tune * (i+0.5) + fir_phase/360.0)));
        coeffs[i + fir_start] = tap;
        sum += tap;
    }
    
    /* Fixup the DC offset introduced by the residual sum.  Turns out that
     * this is generally quite miniscule (0, 1 or -1), so it doesn't hugely
     * matter where we put it... unless we're really unlucky (eg, tune==1),
     * in which case it really hardly matters! */
    coeffs[fir_start] -= sum;

    /* Write all the FIR coefficients.  */
    for (int i = 0; i < MAX_FIR_LENGTH; i++)
        ConfigSpace->Coeffs[i] = coeffs[i];
}

static void set_fircycles(Variant *v)
{
    fir_cycles = (unsigned int) v->buffer[0];
    set_fircoeffs();
}

static void set_firlength(Variant *v)
{
    unsigned int new_length = (unsigned int) v->buffer[0];
    if (new_length < 1  ||  MAX_FIR_LENGTH < new_length)
        printf("Invalid FIR length %d\n", new_length);
    else
    {
        fir_length = (unsigned int) v->buffer[0];
        set_fircoeffs();
    }
}

static void set_firphase(Variant *v)
{
    fir_phase = v->buffer[0];
    set_fircoeffs();
}



static unsigned int tune_to_freq(double tune)
{
    return (unsigned int) round(pow(2,33)*tune/936.0); 
}

static double freq_to_tune(unsigned int freq)
{
    return 936.0 * freq / pow(2, 33);
}

/* Set NCO */
static void set_homfreq (Variant *v)
{
    ConfigSpace->Nco = tune_to_freq(*(double *)v->buffer);
}

/* Set Sweep Start Freq */
static void set_sweepstartfreq (Variant *v)
{
    ConfigSpace->SwpStartFreq = tune_to_freq(*(double *)v->buffer);
}  

/* Set Sweep Stop Freq */
static void set_sweepstopfreq (Variant *v)
{
    ConfigSpace->SwpStopFreq = tune_to_freq(*(double *)v->buffer);
}

static void set_freq_step(Variant *v)
{
    ConfigSpace->PhaseAdvStep = tune_to_freq(*(double *)v->buffer);
}


static float ScaleWaveform[4096];

/* The DDC skew is a group delay in ns.  We translate this into a phase
 * advance of
 *
 *      phase = 0.5 * skew * 2pi * frequency / 936
 *
 * Here the frequency is in tunes, one revolution per turn, which we convert
 * into one revolution per bunch, or half a revolution per nanosecond (at
 * 500MHz RF frequency).
 *    For economy of calculation, we precompute the rotations as scaled
 * integers so that the final computation can be a simple integer
 * multiplication (one instruction when the compiler is in the right mood). */
static int DDC_skew = 0;
static int rotate_I[4096];  // 2**30 * cos(phase)
static int rotate_Q[4096];  // 2**30 * sin(phase)

static void update_tune_scale()
{
    double start = freq_to_tune(ConfigSpace->SwpStartFreq);
    double stop  = freq_to_tune(ConfigSpace->SwpStopFreq);
    double step  = freq_to_tune(ConfigSpace->PhaseAdvStep);

    double phase_scale = - DDC_skew / 936.0 / 2.0;
    double two30 = pow(2, 30);
    for (int i = 0; i < 4096; i++)
    {
        ScaleWaveform[i] = (float) (start + i * step);
        if (ScaleWaveform[i] > stop)
            ScaleWaveform[i] = stop;

        double cycle;
        double phase = 2 * M_PI * modf(
            phase_scale * ScaleWaveform[i], &cycle);
        rotate_I[i] = (int) round(two30 * cos(phase));
        rotate_Q[i] = (int) round(two30 * sin(phase));
    }
}

static void set_tune_scale(Variant *v)
{
    update_tune_scale();
    memcpy(v->buffer, ScaleWaveform, sizeof(ScaleWaveform));
}

void update_ddc_skew(Variant *v)
{
    DDC_skew = (int) v->buffer[0];
    update_tune_scale();
}




static void read_waveform(int *waveform, int length, Variant *v)
{
    int * buffer = (int *) v->buffer;
    for (int i = 0; i < length; i ++)
        buffer[i] = waveform[i];
}


inline int MulSS(int x, int y)
{
    unsigned int result, temp; 
    __asm__("smull   %1, %0, %2, %3" :
        "=&r"(result), "=&r"(temp) : "r"(x), "r"(y)); 
    return result; 
}


/* I & Q readings reduced from raw buffer and rotated. */
static int buffer_I[4096];
static int buffer_Q[4096];

static void update_IQ()
{
    for (int i = 0; i < 4096; i ++)
    {
        int I =
            hb_buf[4*i + 0].lower + hb_buf[4*i + 1].lower + 
            hb_buf[4*i + 2].lower + hb_buf[4*i + 3].lower;
        int Q =
            hb_buf[4*i + 0].upper + hb_buf[4*i + 1].upper + 
            hb_buf[4*i + 2].upper + hb_buf[4*i + 3].upper;
        int rI = rotate_I[i];
        int rQ = rotate_Q[i];
        
        buffer_I[i] = MulSS(I, rI) - MulSS(Q, rQ);
        buffer_Q[i] = MulSS(I, rQ) + MulSS(Q, rI);
    }
}

static void read_buffer_i(Variant *v)
{
    read_waveform(buffer_I, 4096, v);
}

static void read_buffer_q(Variant *v)
{
    read_waveform(buffer_Q, 4096, v);
}


static int PowerPeak;

static void compute_power(Variant *v)
{
    update_IQ();
    
    int * spectrum = (int *) v->buffer;
    
    /* Convert the extracted IQ waveform into a power spectrum.  After
     * accumulation and rotation, the I and Q values are signed 16 bit values
     * again, so can use them as they are. */
    for (int i = 0; i < 4096; i ++)
        spectrum[i] = SQR(buffer_I[i]) + SQR(buffer_Q[i]);
    
    /* Perform a peak search across the power spectrum, writing the index of
     * the peak into PowerPeak. */
    int PeakValue = 0;
    PowerPeak = 0;
    for (int i = 0; i < 4096; i ++)
    {
        if (spectrum[i] > PeakValue)
        {
            PeakValue = spectrum[i];
            PowerPeak = i;
        }
    }
}

static void compute_tune(Variant *v)
{
    double harmonic;
    v->buffer[0] = modf(ScaleWaveform[PowerPeak], &harmonic);
}


static int cumsum_I[4096];
static int cumsum_Q[4096];
static int cumsum_peak;

static void update_cumsum_IQ()
{
    int sum_i = 0;
    int sum_q = 0;
    for (int i = 0; i < 4096; i ++)
    {
        sum_i += buffer_I[i];
        cumsum_I[i] = sum_i;
        sum_q += buffer_Q[i];
        cumsum_Q[i] = sum_q;
    }

    /* Perform peak search across the cumulative sum. */
    cumsum_peak = 0;
    long long int PeakValue = 0;
    for (int i = 0; i < 4096; i ++)
    {
        long long int value =
            SQR((long long int) cumsum_I[i]) +
            SQR((long long int) cumsum_Q[i]);
        if (value > PeakValue)
        {
            PeakValue = value;
            cumsum_peak = i;
        }
    }
}

static void read_cumsum_i(Variant *v)
{
    update_cumsum_IQ();
    read_waveform(cumsum_I, 4096, v);
}

static void read_cumsum_q(Variant *v)
{
    read_waveform(cumsum_Q, 4096, v);
}

static void compute_cumsum_tune(Variant *v)
{
    double harmonic;
    v->buffer[0] = modf(ScaleWaveform[cumsum_peak], &harmonic);
}

static void compute_cumsum_phase(Variant *v)
{
    double phase = atan2(buffer_I[cumsum_peak], buffer_Q[cumsum_peak]);
    double cycle;
    v->buffer[0] = 360.0 * modf(
        fir_phase / 360.0 - phase / 2.0 / M_PI, &cycle);
}


static unsigned int extract_field(int start, int width, unsigned int value)
{
    return (value >> start) & ((1 << width) - 1);
}

static void dump_registers(const iocshArgBuf *args)
{
    unsigned int * registers = (unsigned int *) ConfigSpace;
    for (int i = 0; i < 4; i ++)
    {
        printf("%04x  ", 0x20 * i);
        for (int j = 0; j < 8; j ++)
            printf(" %08x", registers[8*i + j]);
        printf("\n");
    }

    unsigned int ctrl = ConfigSpace->Ctrl;
    printf("Ctrl: %d %d %d %d  %d %d %d %d  %d %d %d %d  %d %d %d %d  %d %d\n",
        extract_field(CTRL_DAC_OUT, ctrl),
        extract_field(CTRL_FIR_INVERT, ctrl),
        extract_field(CTRL_SOFT_TRIG, ctrl),
        extract_field(CTRL_ARCHIVE, ctrl),
        
        extract_field(CTRL_FIR_GAIN, ctrl),
        extract_field(CTRL_HOM_GAIN, ctrl),
        extract_field(CTRL_TRIG_SEL, ctrl),
        extract_field(CTRL_ARM_SEL, ctrl),
        
        extract_field(CTRL_SOFT_ARM, ctrl),
        extract_field(CTRL_GROW_DAMP, ctrl),
        extract_field(CTRL_TEMP_DAC_OUT, ctrl),
        extract_field(CTRL_DDR_INPUT, ctrl),
        
        extract_field(CTRL_SET_PLANE, ctrl),
        extract_field(CTRL_CH_SELECT, ctrl),
        extract_field(CTRL_DDC_INPUT, ctrl),
        extract_field(CTRL_BUNCH_MODE, ctrl),
        
        extract_field(CTRL_IQ_SCALE, ctrl),
        extract_field(CTRL_BUNCH_SYNC, ctrl));
    unsigned int delay = ConfigSpace->Delay;
    printf("Delay: %d %d %d\n",
        extract_field(DELAY_DAC, delay),
        extract_field(DELAY_TUNE_SWEEP, delay),
        extract_field(DELAY_GROW_DAMP, delay));
}

static const iocshFuncDef debugDef = { "d", 0, NULL };

bool InitialiseTunes()
{
    GenericRegister("SWPSTARTFREQ_S", set_sweepstartfreq, 0);
    GenericRegister("SWPSTOPFREQ_S", set_sweepstopfreq, 0);
    GenericRegister("SWPFREQSTEP_S", set_freq_step, 0);
    GenericRegister("TUNESCALE", set_tune_scale, 0);
    
    GenericRegister("DDC_I", read_buffer_i, 0);
    GenericRegister("DDC_Q", read_buffer_q, 0);
    GenericRegister("TUNEPOWER", compute_power, 0);
    GenericRegister("TUNE", compute_tune, 0);

    GenericRegister("CUMSUM_I", read_cumsum_i, 0);
    GenericRegister("CUMSUM_Q", read_cumsum_q, 0);
    GenericRegister("TUNECUMSUM", compute_cumsum_tune, 0);
    GenericRegister("TUNEPHASE", compute_cumsum_phase, 0);

    GenericRegister("DDCSKEW_S", update_ddc_skew, 0);

    return true;
}


int GenericInit()
{
    iocshRegister(&debugDef, dump_registers);

    MapFastFeedbackMemory();
    
    printf("Registering Generic Device functions\n");
    RegisterGenericHook(GenericGlobalLock);
    
    GenericRegister("BUNCH_W",   generic_reg_write, &ConfigSpace->Bunch);
    GenericRegister("BUNCH_R",   generic_reg_read, &ConfigSpace->Bunch);
    GenericRegister("READOUT_W", generic_reg_write, &ConfigSpace->ProgClkVal);
    GenericRegister("READOUT_R", generic_reg_read, &ConfigSpace->ProgClkVal);
    GenericRegister("NCO_W",     generic_reg_write, &ConfigSpace->Nco);
    GenericRegister("NCO_R",     generic_reg_read, &ConfigSpace->Nco);
    GenericRegister("STATUS_R",  generic_reg_read, &ConfigSpace->Status);
    GenericRegister("DELAY_W",   generic_reg_write, &ConfigSpace->Delay);
    GenericRegister("DELAY_R",   generic_reg_read, &ConfigSpace->Delay);
    GenericRegister("ADC_OFF_AB_W",
        generic_reg_write, &ConfigSpace->AdcOffAB);
    GenericRegister("ADC_OFF_AB_R",
        generic_reg_read, &ConfigSpace->AdcOffAB);
    GenericRegister("ADC_OFF_CD_W",
        generic_reg_write, &ConfigSpace->AdcOffCD);
    GenericRegister("ADC_OFF_CD_R",
        generic_reg_read, &ConfigSpace->AdcOffCD);
    GenericRegister("ARCBUF",    generic_buf_read,  DataSpace);
    GenericRegister("COEFFS_R",  generic_buf_read,  &ConfigSpace->Coeffs);
    GenericRegister("COEFFS_W",  generic_buf_write, &ConfigSpace->Coeffs);
    
    
    GenericRegister("BB_GAIN_COE_ALL_W",
        bb_gain_and_dac_write, &ConfigSpace->BB_Gain_Coeffs);
    GenericRegister("BB_GAINS_W", bb_write_gain, &ConfigSpace->BB_Gain_Coeffs);
    GenericRegister("BB_DACS_W",  bb_write_dac, &ConfigSpace->BB_Gain_Coeffs);
    
    GenericRegister("DAC_MINMAX_ALL_R",
        dac_minmax_read, &ConfigSpace->BB_Dac_MinMax);
    GenericRegister("ADC_MINBUF_R",
        adc_minbuf_read, &ConfigSpace->BB_Adc_MinMax);
    GenericRegister("ADC_MAXBUF_R",
        adc_maxbuf_read, &ConfigSpace->BB_Adc_MinMax);
    GenericRegister("ADC_DIFFBUF_R", adc_diffbuf_read, 0);
    GenericRegister("HB_BUF_R",      hb_buf_read, DataSpace);
    GenericRegister("HB_BUF_LOWER_R", hb_buf_lower_read, DataSpace);
    GenericRegister("HB_BUF_UPPER_R", hb_buf_upper_read, DataSpace);
    GenericRegister("DACOUT_S",  set_dacout,  0);
    GenericRegister("FIRGAIN_S", set_firgain, 0);
    GenericRegister("FIRINVERT_S", set_firinvert, 0);
    GenericRegister("HOMGAIN_S", set_homgain, 0);
    
    GenericRegister("FIRCYCLES_S", set_fircycles, 0);
    GenericRegister("FIRLENGTH_S", set_firlength, 0);
    GenericRegister("FIRPHASE_S", set_firphase, 0);
    
    GenericRegister("ARCHIVE_S", set_archive, 0);
    GenericRegister("CHSELECT_S", set_chselect, 0);
    GenericRegister("TRIGSEL_S", set_trigsel, 0);
    GenericRegister("ARMSEL_S", set_armsel, 0);
    GenericRegister("DDCINPUT_S", set_ddcinput, 0);
    GenericRegister("DDRINPUT_S", set_ddrinput, 0);
    GenericRegister("DACDLY_S", set_dacdly, 0);
    GenericRegister("HOMFREQ_S", set_homfreq, 0);

    GenericRegister("GROWDAMPMODE_S", set_growdampmode, 0);
    GenericRegister("GROWDAMPPERIOD_S", set_growdampperiod, 0);
    GenericRegister("BUNCHMODE_S", set_bunchmode, 0);
    GenericRegister("TEMPDACOUT_S",  set_tempdacout,  0);
    GenericRegister("IQSCALE_S",  set_iqscale,  0);
    GenericRegister("PROGCLKVAL_S",
        generic_reg_write, &ConfigSpace->ProgClkVal);
    GenericRegister("TUNESWEEPMODE_S", set_tunesweepmode, 0);
    GenericRegister("SOFTTRIG_S", set_softtrig, 0);
    GenericRegister("SOFTARM_S", set_softarm, 0);
    GenericRegister("BUNCHSYNC_S", set_bunchsync, 0);
    GenericRegister("ADCMEAN_R", get_adcmean, 0);
    GenericRegister("ADCSTD_R", get_adcstd, 0);

    return InitialiseTunes();
}
