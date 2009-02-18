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
static float ADC_mean_diff;
static float ADC_var_diff; 

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
    
    ADC_mean_diff = sum_diff / (float) MAX_BUNCH_COUNT;
    ADC_var_diff = sum_var / (float) MAX_BUNCH_COUNT - ADC_mean_diff*ADC_mean_diff;
    
    
    memcpy(buffer, ADC_min_buf, sizeof(ADC_min_buf));
}

PUBLISH_SIMPLE_WAVEFORM(
    short, "ADC_MINBUF_R", MAX_BUNCH_COUNT, adc_minbuf_read)
PUBLISH_READ_WAVEFORM(
    short, "ADC_MAXBUF_R", MAX_BUNCH_COUNT, ADC_max_buf)
PUBLISH_READ_WAVEFORM(
    short, "ADC_DIFFBUF_R", MAX_BUNCH_COUNT, ADC_diff_buf)
PUBLISH_VARIABLE_READ(ai, "ADCMEAN_R", ADC_mean_diff)
PUBLISH_VARIABLE_READ(ai, "ADCSTD_R",  ADC_var_diff)


/* copy paste of ADC buffer below, with rename to DAC, surely there's a better
 * way...*/

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                            DAC Min/Max Buffer                             */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

static short int DAC_min_buf[MAX_BUNCH_COUNT];
static short int DAC_max_buf[MAX_BUNCH_COUNT];
static short int DAC_diff_buf[MAX_BUNCH_COUNT];
static float DAC_mean_diff;
static float DAC_var_diff; 

static void dac_minbuf_read(short *buffer)
{
    read_DAC_MinMax(DAC_min_buf, DAC_max_buf);

    int sum_diff = 0;
    long long int sum_var = 0;
    for (unsigned int i = 0; i < MAX_BUNCH_COUNT; ++i)
    {
        short int diff = DAC_max_buf[i] - DAC_min_buf[i];
        DAC_diff_buf[i] = diff; 
        sum_diff += diff;
        sum_var  += (int) diff*diff;
    }
    
    DAC_mean_diff = sum_diff / (float) MAX_BUNCH_COUNT;
    DAC_var_diff = sum_var / (float) MAX_BUNCH_COUNT - DAC_mean_diff*DAC_mean_diff;
    
    
    memcpy(buffer, DAC_min_buf, sizeof(DAC_min_buf));
}

PUBLISH_SIMPLE_WAVEFORM(
    short, "DAC_MINBUF_R", MAX_BUNCH_COUNT, dac_minbuf_read)
PUBLISH_READ_WAVEFORM(
    short, "DAC_MAXBUF_R", MAX_BUNCH_COUNT, DAC_max_buf)
PUBLISH_READ_WAVEFORM(
    short, "DAC_DIFFBUF_R", MAX_BUNCH_COUNT, DAC_diff_buf)
PUBLISH_VARIABLE_READ(ai, "DACMEAN_R", DAC_mean_diff)
PUBLISH_VARIABLE_READ(ai, "DACSTD_R",  DAC_var_diff)




/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                        Selectable Buffer (ADC/DAC/I-Q)                    */
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

static void set_fircycles(int cycles)
{
    fir_cycles = cycles;
    set_fircoeffs();
}

static bool set_firlength(void *context, int new_length)
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

static void set_firphase(double new_phase)
{
    fir_phase = new_phase;
    set_fircoeffs();
}


PUBLISH_SIMPLE_WRITE(longout, "FIRCYCLES_S", set_fircycles)
PUBLISH(longout, "FIRLENGTH_S", set_firlength)
PUBLISH_SIMPLE_WRITE(ao,      "FIRPHASE_S",  set_firphase)

/* Direct access to the FIR coefficients through register interface. */
PUBLISH_SIMPLE_WAVEFORM(int, "COEFFS_R", MAX_FIR_COEFFS, read_FIR_coeffs)
PUBLISH_SIMPLE_WAVEFORM(int, "COEFFS_W", MAX_FIR_COEFFS, write_FIR_coeffs)



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
    return PUBLISH_EPICS_DATA();
}
