#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <math.h>

#include "hardware.h"
#include "test_error.h"
#include "epics_device.h"

#include "device.h"




#define SQR(x)  ((x)*(x))




/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                       Bunch by Bunch Configuration                        */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* Write BB gain&conf values as a single waveform */


static void bb_write_gain(float *gains)
{
    short int int_gains[MAX_BUNCH_COUNT];
    for (int i = 0; i < MAX_BUNCH_COUNT; i++)
    {
        float gain = gains[i];
        int_gains[i] =
            gain >= 1.0  ? 32767 :
            gain <= -1.0 ? -32767 : (int) (32767 * gain);
    }
    write_BB_gains(int_gains);
}


// PUBLISH_SIMPLE_WAVEFORM(
//     int,   "BB_GAIN_COE_ALL_W", MAX_BUNCH_COUNT, bb_gain_and_dac_write)
PUBLISH_SIMPLE_WAVEFORM(float, "BB_GAINS_W", MAX_BUNCH_COUNT, bb_write_gain)
PUBLISH_SIMPLE_WAVEFORM(short, "BB_DACS_W",  MAX_BUNCH_COUNT, write_BB_DACs)




/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                            ADC Min/Max Buffer                             */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

static short int ADC_min_buf[MAX_BUNCH_COUNT];
static short int ADC_max_buf[MAX_BUNCH_COUNT];
static short int ADC_diff_buf[MAX_BUNCH_COUNT];
static float mean_diff;
static float var_diff; 

static void adc_minbuf_read(short *buffer)
{
    read_ADC_MinMax(ADC_min_buf, ADC_max_buf);

    int sum_diff = 0;
    long long int sum_var = 0;
    for (unsigned int i = 0; i < MAX_BUNCH_COUNT; ++i)
    {
        short int diff = ADC_max_buf[i] - ADC_min_buf[i];
        ADC_diff_buf[i] = diff; 
        sum_diff += diff;
        sum_var  += (int) diff*diff;
    }
    
    mean_diff = sum_diff / (float) MAX_BUNCH_COUNT;
    var_diff = sum_var / (float) MAX_BUNCH_COUNT - mean_diff*mean_diff;
    
    
    memcpy(buffer, ADC_min_buf, sizeof(ADC_min_buf));
}

PUBLISH_SIMPLE_WAVEFORM(
    short, "ADC_MINBUF_R", MAX_BUNCH_COUNT, adc_minbuf_read)
PUBLISH_READ_WAVEFORM(
    short, "ADC_MAXBUF_R", MAX_BUNCH_COUNT, ADC_max_buf)
PUBLISH_READ_WAVEFORM(
    short, "ADC_DIFFBUF_R", MAX_BUNCH_COUNT, ADC_diff_buf)
PUBLISH_VARIABLE_READ(ai, "ADCMEAN_R", mean_diff)
PUBLISH_VARIABLE_READ(ai, "ADCSTD_R",  var_diff)





/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                            ??????????????????                             */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



static short int hb_buf_lower[MAX_DATA_LENGTH];
static short int hb_buf_upper[MAX_DATA_LENGTH];

static void hb_buf_lower_read(short *buffer)
{
    read_DataSpace(hb_buf_lower, hb_buf_upper);
    memcpy(buffer, hb_buf_lower, sizeof(hb_buf_lower));
}


PUBLISH_SIMPLE_WAVEFORM(
    short, "HB_BUF_LOWER_R", MAX_DATA_LENGTH, hb_buf_lower_read)
PUBLISH_READ_WAVEFORM(short, "HB_BUF_UPPER_R", MAX_DATA_LENGTH, hb_buf_upper)


PUBLISH_METHOD("SOFTTRIG_S",  set_softTrigger)
PUBLISH_METHOD("BUNCHSYNC_S", set_bunchSync)



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                       Control of FIR coefficients                         */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

static unsigned int fir_cycles = 1;
static unsigned int fir_length = 1;
static double fir_phase = 0;

#define MAX_FIR_LENGTH  9

static void set_fircoeffs()
{
    double tune = (double) fir_cycles / fir_length;    

    /* Only work on the active part of the filter, leaving the beginning of
     * the filter padded with zeros.  This ensures zero extra delay from the
     * filter if the length is shorter than the filter length. */
    int fir_start = MAX_FIR_LENGTH - fir_length;

    /* Note that MAX_FIR_LENGTH < MAX_FIR_COEFFS.  We still have to write all
     * of the coefficients. */
    int coeffs[MAX_FIR_COEFFS];
    memset(coeffs, 0, sizeof(coeffs));

    /* Calculate FIR coeffs and the mean value. */
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
    write_FIR_coeffs(coeffs);
}

static bool set_fircycles(int cycles)
{
    fir_cycles = cycles;
    set_fircoeffs();
    return true;
}

static bool set_firlength(int new_length)
{
    if (new_length < 1  ||  MAX_FIR_LENGTH < new_length)
    {
        printf("Invalid FIR length %d\n", new_length);
        return false;
    }
    else
    {
        fir_length = new_length;
        set_fircoeffs();
        return true;
    }
}

static bool set_firphase(double new_phase)
{
    fir_phase = new_phase;
    set_fircoeffs();
    return true;
}


PUBLISH(longout, "FIRCYCLES_S", set_fircycles)
PUBLISH(longout, "FIRLENGTH_S", set_firlength)
PUBLISH(ao,      "FIRPHASE_S",  set_firphase)

/* Direct access to the FIR coefficients through register interface. */
PUBLISH_SIMPLE_WAVEFORM(int, "COEFFS_R", MAX_FIR_COEFFS, read_FIR_coeffs)
PUBLISH_SIMPLE_WAVEFORM(int, "COEFFS_W", MAX_FIR_COEFFS, write_FIR_coeffs)



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                            Tune Measurement                               */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* Display scale for tune response, also used for computing tune. */
static float ScaleWaveform[4096];

/* Waveforms for compensating IQ by DDC_skew.
 * 
 * The DDC skew is a group delay in ns.  We translate this into a phase
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

/* I & Q readings reduced from raw buffer and rotated to compensate for skew,
 * as above. */
static int buffer_I[4096];
static int buffer_Q[4096];

/* Index of raw power peak. */
static int PowerPeak;

static int cumsum_I[4096];
static int cumsum_Q[4096];
static int cumsum_peak;



static unsigned int tune_to_freq(double tune)
{
    return (unsigned int) round(pow(2,33)*tune/936.0); 
}

static double freq_to_tune(unsigned int freq)
{
    return 936.0 * freq / pow(2, 33);
}

/* Set NCO */
static void set_homfreq (double new_freq)
{
    write_NCO_frequency(tune_to_freq(new_freq));
}

/* Set Sweep Start Freq */
static void set_sweepstartfreq (double new_freq)
{
    write_SweepStartFreq(tune_to_freq(new_freq));
}  

/* Set Sweep Stop Freq */
static void set_sweepstopfreq (double new_freq)
{
    write_SweepStopFreq(tune_to_freq(new_freq));
}

static void set_freq_step(double new_freq)
{
    write_SweepStep(tune_to_freq(new_freq));
}



static void update_tune_scale()
{
    double start = freq_to_tune(read_SweepStartFreq());
    double stop  = freq_to_tune(read_SweepStopFreq());
    double step  = freq_to_tune(read_SweepStep());

    double phase_scale = - DDC_skew / 936.0 / 2.0;
    double two30 = (double) (1 << 30);
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

static void set_tune_scale(float * buffer)
{
    update_tune_scale();
    memcpy(buffer, ScaleWaveform, sizeof(ScaleWaveform));
}

static bool update_ddc_skew(int new_skew)
{
    DDC_skew = new_skew;
    update_tune_scale();
    return true;
}




static void read_waveform(const int *waveform, int length, int *buffer)
{
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


static void update_IQ()
{
    for (int i = 0; i < 4096; i ++)
    {
        int I =
            hb_buf_lower[4*i + 0] + hb_buf_lower[4*i + 1] + 
            hb_buf_lower[4*i + 2] + hb_buf_lower[4*i + 3];
        int Q =
            hb_buf_upper[4*i + 0] + hb_buf_upper[4*i + 1] + 
            hb_buf_upper[4*i + 2] + hb_buf_upper[4*i + 3];
        int rI = rotate_I[i];
        int rQ = rotate_Q[i];
        
        buffer_I[i] = MulSS(I, rI) - MulSS(Q, rQ);
        buffer_Q[i] = MulSS(I, rQ) + MulSS(Q, rI);
    }
}


static void compute_power(int *spectrum)
{
    update_IQ();
    
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

static bool compute_tune(double *tune)
{
    double harmonic;
    *tune = modf(ScaleWaveform[PowerPeak], &harmonic);
    return true;
}


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

static void read_cumsum_i(int *buffer)
{
    update_cumsum_IQ();
    read_waveform(cumsum_I, 4096, buffer);
}


static bool compute_cumsum_tune(double *tune)
{
    double harmonic;
    *tune = modf(ScaleWaveform[cumsum_peak], &harmonic);
    return true;
}

static bool compute_cumsum_phase(double *phase)
{
    *phase = 180.0 / M_PI *
        atan2(-buffer_Q[cumsum_peak], -buffer_I[cumsum_peak]);
    return true;
}


PUBLISH_SIMPLE_WRITE(ao, "HOMFREQ_S", set_homfreq)

PUBLISH_SIMPLE_WRITE(ao, "SWPSTARTFREQ_S", set_sweepstartfreq)
PUBLISH_SIMPLE_WRITE(ao, "SWPSTOPFREQ_S", set_sweepstopfreq)
PUBLISH_SIMPLE_WRITE(ao, "SWPFREQSTEP_S", set_freq_step)
PUBLISH_SIMPLE_WAVEFORM(float, "TUNESCALE", 4096, set_tune_scale)

PUBLISH_READ_WAVEFORM(int, "DDC_I", 4096, buffer_I)
PUBLISH_READ_WAVEFORM(int, "DDC_Q", 4096, buffer_Q)
PUBLISH_SIMPLE_WAVEFORM(int, "TUNEPOWER", 4096, compute_power)
PUBLISH(ai, "TUNE", compute_tune)

PUBLISH_SIMPLE_WAVEFORM(int, "CUMSUM_I", 4096, read_cumsum_i)
PUBLISH_READ_WAVEFORM(int, "CUMSUM_Q", 4096, cumsum_Q)
PUBLISH(ai, "TUNECUMSUM", compute_cumsum_tune)
PUBLISH(ai, "TUNEPHASE", compute_cumsum_phase)

PUBLISH(longout, "DDCSKEW_S", update_ddc_skew)



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                            ??????????????????                             */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */




/* Wrappers for hardware access functions to make them amenable to EPICS
 * export. */




#define PUBLISH_REGISTER_R(record, name, register) \
    PUBLISH_SIMPLE_READ (record, name, read_##register) 
#define PUBLISH_REGISTER_W(record, name, register) \
    PUBLISH_SIMPLE_WRITE(record, name, write_##register) 

/* Register access PVs have a very consistent style. */
#define GENERIC_REGISTER(pv_name, field) \
    PUBLISH_REGISTER_W(longout, pv_name "_W", field) \
    PUBLISH_REGISTER_R(longin,  pv_name "_R", field)

PUBLISH_REGISTER_W(mbbo,    "DACOUT_S",         CTRL_DAC_OUT)
PUBLISH_REGISTER_W(mbbo,    "FIRINVERT_S",      CTRL_FIR_INVERT)
PUBLISH_REGISTER_W(mbbo,    "ARCHIVE_S",        CTRL_ARCHIVE)
PUBLISH_REGISTER_W(mbbo,    "FIRGAIN_S",        CTRL_FIR_GAIN)
PUBLISH_REGISTER_W(mbbo,    "HOMGAIN_S",        CTRL_HOM_GAIN)
PUBLISH_REGISTER_W(mbbo,    "TRIGSEL_S",        CTRL_TRIG_SEL)
PUBLISH_REGISTER_W(mbbo,    "ARMSEL_S",         CTRL_ARM_SEL)
PUBLISH_REGISTER_W(mbbo,    "GROWDAMPMODE_S",   CTRL_GROW_DAMP)
PUBLISH_REGISTER_W(mbbo,    "TEMPDACOUT_S",     CTRL_TEMP_DAC_OUT)
PUBLISH_REGISTER_W(mbbo,    "DDRINPUT_S",       CTRL_DDR_INPUT)
PUBLISH_REGISTER_W(mbbo,    "CHSELECT_S",       CTRL_CH_SELECT)
PUBLISH_REGISTER_W(mbbo,    "DDCINPUT_S",       CTRL_DDC_INPUT)
PUBLISH_REGISTER_W(mbbo,    "BUNCHMODE_S",      CTRL_BUNCH_MODE)
PUBLISH_REGISTER_W(longout, "IQSCALE_S",        CTRL_IQ_SCALE)
PUBLISH_REGISTER_W(mbbo,    "SOFTARM_S",        CTRL_SOFT_ARM)

PUBLISH_REGISTER_W(longout, "DACDLY_S",         DELAY_DAC)
PUBLISH_REGISTER_W(longout, "GROWDAMPPERIOD_S", DELAY_GROW_DAMP)
PUBLISH_REGISTER_W(mbbo,    "TUNESWEEPMODE_S",  DELAY_TUNE_SWEEP)

PUBLISH_REGISTER_W(longout, "PROGCLKVAL_S",     DDC_dwellTime)

PUBLISH_REGISTER_R(longin,  "STATUS_R",         FPGA_version)

GENERIC_REGISTER("BUNCH",       BunchSelect)
GENERIC_REGISTER("NCO",         NCO_frequency)
GENERIC_REGISTER("ADC_OFF_AB",  AdcOffAB)
GENERIC_REGISTER("ADC_OFF_CD",  AdcOffCD)



#ifndef __DEFINE_EPICS__
#include "device.EPICS"
#endif

int GenericInit()
{
    printf("Registering Generic Device functions\n");
//    RegisterGenericHook(GenericGlobalLock);

    return PUBLISH_EPICS_DATA();
}
