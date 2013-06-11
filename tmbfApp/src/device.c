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
#include "error.h"
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


static void publish_bb_control(void)
{
    PUBLISH_WF_ACTION_P(float, "BB_GAINS", MAX_BUNCH_COUNT, bb_write_gain);
    PUBLISH_WF_ACTION_P(short, "BB_DACS", MAX_BUNCH_COUNT, write_BB_DACs);
    PUBLISH_WF_ACTION_P(
        short, "BB_TEMPDACS", MAX_BUNCH_COUNT, write_BB_TEMPDACs);
}


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


static void publish_buffer(void)
{
    PUBLISH_WF_ACTION(
        short, "HB_BUF_LOWER", MAX_DATA_LENGTH, hb_buf_lower_read);
    PUBLISH_WF_READ_VAR(short, "HB_BUF_UPPER", MAX_DATA_LENGTH, hb_buf_upper);

    PUBLISH_ACTION("SOFTTRIG",  set_softTrigger);
    PUBLISH_ACTION("BUNCHSYNC", set_bunchSync);
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                       Control of FIR coefficients                         */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

static unsigned int fir_cycles = 1;
static unsigned int fir_length = 1;
static double fir_phase = 0;


static void set_fircoeffs(void)
{
    double tune = (double) fir_cycles / fir_length;

    /* Only work on the active part of the filter, leaving the beginning of
     * the filter padded with zeros.  This ensures zero extra delay from the
     * filter if the length is shorter than the filter length. */
    int fir_start = MAX_FIR_COEFFS - fir_length;

    /* Note that MAX_FIR_COEFFS < MAX_FIR_COEFFS.  We still have to write all
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

static bool set_firlength(int new_length)
{
    if (new_length < 1  ||  MAX_FIR_COEFFS < new_length)
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


static void publish_fir(void)
{
    PUBLISH_WRITER_P(longout, "FIRCYCLES", set_fircycles);
    PUBLISH_WRITER_B_P(longout, "FIRLENGTH", set_firlength);
    PUBLISH_WRITER_P(ao,      "FIRPHASE",  set_firphase);

    /* Direct access to the FIR coefficients through register interface. */
    PUBLISH_WF_ACTION(int, "COEFFS", MAX_FIR_COEFFS, write_FIR_coeffs);
    PUBLISH_WF_ACTION(int, "COEFFSRB", MAX_FIR_COEFFS, read_FIR_coeffs);
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                            ??????????????????                             */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


static EPICS_STRING version_string;



/* Persistent writable register.  All associated records must be marked with
 * PINI='YES' to ensure the initial loaded state is written back. */
#define PUBLISH_REGISTER_P(record, name, register) \
    PUBLISH_WRITER_P(record, name, write_##register)


static void publish_registers(void)
{
    PUBLISH_REGISTER_P(mbbo,    "ARCHIVE",        CTRL_ARCHIVE);
    PUBLISH_REGISTER_P(mbbo,    "FIRGAIN",        CTRL_FIR_GAIN);
    PUBLISH_REGISTER_P(mbbo,    "HOMGAIN",        CTRL_HOM_GAIN);
    PUBLISH_REGISTER_P(mbbo,    "TRIGSEL",        CTRL_TRIG_SEL);
    PUBLISH_REGISTER_P(mbbo,    "ARMSEL",         CTRL_ARM_SEL);
    PUBLISH_REGISTER_P(mbbo,    "GROWDAMPMODE",   CTRL_GROW_DAMP);
    PUBLISH_REGISTER_P(mbbo,    "CHSELECT",       CTRL_CH_SELECT);
    PUBLISH_REGISTER_P(mbbo,    "DDCINPUT",       CTRL_DDC_INPUT);
    PUBLISH_REGISTER_P(ulongout, "IQSCALE",        CTRL_IQ_SCALE);
    PUBLISH_REGISTER_P(mbbo,    "SOFTARM",        CTRL_SOFT_ARM);

    PUBLISH_REGISTER_P(ulongout, "GROWDAMPPERIOD", DELAY_GROW_DAMP);
    PUBLISH_REGISTER_P(mbbo,    "TUNESWEEPMODE",  DELAY_TUNE_SWEEP);

    PUBLISH_REGISTER_P(ulongout, "PROGCLKVAL",     DDC_dwellTime);

    PUBLISH_REGISTER_P(ulongout, "BUNCH",          BunchSelect);
    PUBLISH_REGISTER_P(ulongout, "NCO",            NCO_frequency);

    PUBLISH_READER(ulongin, "STATUS", read_FPGA_version);
    PUBLISH_READ_VAR(stringin, "VERSION", version_string);
}



bool GenericInit(void)
{
    copy_epics_string(&version_string, TMBF_VERSION);

    publish_bb_control();
    publish_buffer();
    publish_fir();
    publish_registers();
    return true;
}
