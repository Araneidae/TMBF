/* EPICS interface to ADC readout and control. */

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>

#include "error.h"
#include "hardware.h"
#include "epics_device.h"

#include "adc.h"

/* Need to share these with device.c */
#define PUBLISH_REGISTER_P(record, name, register) \
    PUBLISH_SIMPLE_WRITE_INIT( \
        record, name, write_##register, read_##register, .persist = true)


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* ADC Min/Max Buffer */

static short int min_buf[MAX_BUNCH_COUNT];
static short int max_buf[MAX_BUNCH_COUNT];
static short int diff_buf[MAX_BUNCH_COUNT];
static float mean_diff;
static float var_diff;
static int max_max;


#define MAX(a, b) ((a) > (b) ? (a) : (b))

static void read_adc_minmax(void)
{
    read_ADC_MinMax(min_buf, max_buf);

    int sum_diff = 0;
    long long int sum_var = 0;
    max_max = 0;
    for (unsigned int i = 0; i < MAX_BUNCH_COUNT; ++i)
    {
        short int diff = max_buf[i] - min_buf[i];
        diff_buf[i] = diff;
        sum_diff += diff;
        sum_var  += (int) diff*diff;
        max_max = MAX(max_max, abs(max_buf[i]));
        max_max = MAX(max_max, abs(min_buf[i]));
    }

    mean_diff = sum_diff / (float) MAX_BUNCH_COUNT;
    var_diff  = sum_var / (float) MAX_BUNCH_COUNT - mean_diff*mean_diff;
}


PUBLISH_METHOD("ADC:SCAN", read_adc_minmax)

PUBLISH_READ_WAVEFORM(
    short, "ADC:MINBUF", MAX_BUNCH_COUNT, min_buf)
PUBLISH_READ_WAVEFORM(
    short, "ADC:MAXBUF", MAX_BUNCH_COUNT, max_buf)
PUBLISH_READ_WAVEFORM(
    short, "ADC:DIFFBUF", MAX_BUNCH_COUNT, diff_buf)
PUBLISH_VARIABLE_READ(ai, "ADC:MEAN", mean_diff)
PUBLISH_VARIABLE_READ(ai, "ADC:STD",  var_diff)
PUBLISH_VARIABLE_READ(longin, "ADC:MAX", max_max)


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* DAC Min/Max Buffer */
/* Currently a cut and paste copy of ADC above, but will be unified after rework
 * of the EPICS interface. */

static short int dac_min_buf[MAX_BUNCH_COUNT];
static short int dac_max_buf[MAX_BUNCH_COUNT];
static short int dac_diff_buf[MAX_BUNCH_COUNT];
static float dac_mean_diff;
static float dac_var_diff;
static int dac_max_max;



static void read_dac_minmax(void)
{
    read_DAC_MinMax(dac_min_buf, dac_max_buf);

    int sum_diff = 0;
    long long int sum_var = 0;
    dac_max_max = 0;
    for (unsigned int i = 0; i < MAX_BUNCH_COUNT; ++i)
    {
        short int diff = dac_max_buf[i] - dac_min_buf[i];
        dac_diff_buf[i] = diff;
        sum_diff += diff;
        sum_var  += (int) diff*diff;
        dac_max_max = MAX(dac_max_max, abs(dac_max_buf[i]));
        dac_max_max = MAX(dac_max_max, abs(dac_min_buf[i]));
    }

    dac_mean_diff = sum_diff / (float) MAX_BUNCH_COUNT;
    dac_var_diff =
        sum_var / (float) MAX_BUNCH_COUNT - dac_mean_diff*dac_mean_diff;
}


PUBLISH_METHOD("DAC:SCAN", read_dac_minmax)

PUBLISH_READ_WAVEFORM(
    short, "DAC:MINBUF", MAX_BUNCH_COUNT, dac_min_buf)
PUBLISH_READ_WAVEFORM(
    short, "DAC:MAXBUF", MAX_BUNCH_COUNT, dac_max_buf)
PUBLISH_READ_WAVEFORM(
    short, "DAC:DIFFBUF", MAX_BUNCH_COUNT, dac_diff_buf)
PUBLISH_VARIABLE_READ(ai, "DAC:MEAN", dac_mean_diff)
PUBLISH_VARIABLE_READ(ai, "DAC:STD",  dac_var_diff)
PUBLISH_VARIABLE_READ(longin, "DAC:MAX", dac_max_max)


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* ADC offset control. */

static void set_offsets(const short *offsets)
{
    write_AdcOffAB((offsets[1] << 16) + (offsets[0] & 0xFFFF));
    write_AdcOffCD((offsets[3] << 16) + (offsets[2] & 0xFFFF));
}

PUBLISH_SIMPLE_WAVEFORM(short, "ADC:OFFSET", 4, set_offsets, .persist = true)


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* DAC output control. */

PUBLISH_REGISTER_P(mbbo, "DAC:ENABLE", CTRL_DAC_ENA)
PUBLISH_REGISTER_P(longout, "DAC:DELAY",  DELAY_DAC)


#ifndef __DEFINE_EPICS__
#include "adc.EPICS"
#endif

bool initialise_adc(void)
{
    return PUBLISH_EPICS_DATA();
}
