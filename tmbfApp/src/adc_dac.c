/* EPICS interface to ADC and DAC readout and control. */

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

#include "adc_dac.h"



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Min/Max Buffer */

struct min_max {
    void (*read)(short *min_buf, short *max_buf);
    short int min_buf[MAX_BUNCH_COUNT];
    short int max_buf[MAX_BUNCH_COUNT];
    short int diff_buf[MAX_BUNCH_COUNT];
    double mean_diff;
    double var_diff;
    int max_max;
};


#define MAX(a, b) ((a) > (b) ? (a) : (b))

/* Reads min & max waveforms from selected hardware and computes the remaining
 * waveforms and values of interest. */
static bool read_minmax(void *context, const bool *ignore)
{
    struct min_max *min_max = context;

    min_max->read(min_max->min_buf, min_max->max_buf);

    int sum_diff = 0;
    long long int sum_var = 0;
    int max_max = 0;
    for (unsigned int i = 0; i < MAX_BUNCH_COUNT; ++i)
    {
        short int diff = min_max->max_buf[i] - min_max->min_buf[i];
        min_max->diff_buf[i] = diff;
        sum_diff += diff;
        sum_var  += (int) diff * diff;
        max_max = MAX(max_max, abs(min_max->max_buf[i]));
        max_max = MAX(max_max, abs(min_max->min_buf[i]));
    }

    min_max->max_max = max_max;
    min_max->mean_diff = sum_diff / (double) MAX_BUNCH_COUNT;
    min_max->var_diff  =
        sum_var  / (double) MAX_BUNCH_COUNT -
        min_max->mean_diff * min_max->mean_diff;
    return true;
}

static void publish_minmax(const char *name, struct min_max *min_max)
{
    char buffer[20];
#define FORMAT(record_name) \
    (sprintf(buffer, "%s:%s", name, record_name), buffer)

    PUBLISH(bo, FORMAT("SCAN"), .write = read_minmax, .context = min_max);

    PUBLISH_WF_READ_VAR(
        short, FORMAT("MINBUF"), MAX_BUNCH_COUNT, min_max->min_buf);
    PUBLISH_WF_READ_VAR(
        short, FORMAT("MAXBUF"), MAX_BUNCH_COUNT, min_max->max_buf);
    PUBLISH_WF_READ_VAR(
        short, FORMAT("DIFFBUF"), MAX_BUNCH_COUNT, min_max->diff_buf);

    PUBLISH_READ_VAR(ai,      FORMAT("MEAN"), min_max->mean_diff);
    PUBLISH_READ_VAR(ai,      FORMAT("STD"),  min_max->var_diff);
    PUBLISH_READ_VAR(longin,  FORMAT("MAX"),  min_max->max_max);
#undef FORMAT
}


static struct min_max adc_min_max = { .read = read_ADC_MinMax };
static struct min_max dac_min_max = { .read = read_DAC_MinMax };



/* ADC offset control. */
static void set_offsets(short *offsets)
{
    write_AdcOffAB((offsets[1] << 16) + (offsets[0] & 0xFFFF));
    write_AdcOffCD((offsets[3] << 16) + (offsets[2] & 0xFFFF));
}


bool initialise_adc_dac(void)
{
    /* Common min/max PVs. */
    publish_minmax("ADC", &adc_min_max);
    publish_minmax("DAC", &dac_min_max);

    /* Offset control for ADC. */
    PUBLISH_WF_ACTION_P(short, "ADC:OFFSET", 4, set_offsets);

    /* Direct register control for DAC. */
    PUBLISH_WRITER_P(mbbo, "DAC:ENABLE", write_CTRL_DAC_ENA);
    PUBLISH_WRITER_P(ulongout, "DAC:DELAY", write_DELAY_DAC);

    return true;
}
