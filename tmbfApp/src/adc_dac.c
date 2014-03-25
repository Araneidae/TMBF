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
    short int min_buf[BUNCHES_PER_TURN];
    short int max_buf[BUNCHES_PER_TURN];
    short int diff_buf[BUNCHES_PER_TURN];
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
    for (unsigned int i = 0; i < BUNCHES_PER_TURN; ++i)
    {
        short int diff = min_max->max_buf[i] - min_max->min_buf[i];
        min_max->diff_buf[i] = diff;
        sum_diff += diff;
        sum_var  += (int) diff * diff;
        max_max = MAX(max_max, abs(min_max->max_buf[i]));
        max_max = MAX(max_max, abs(min_max->min_buf[i]));
    }

    min_max->max_max = max_max;
    min_max->mean_diff = sum_diff / (double) BUNCHES_PER_TURN;
    min_max->var_diff  =
        sum_var  / (double) BUNCHES_PER_TURN -
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
        short, FORMAT("MINBUF"), BUNCHES_PER_TURN, min_max->min_buf);
    PUBLISH_WF_READ_VAR(
        short, FORMAT("MAXBUF"), BUNCHES_PER_TURN, min_max->max_buf);
    PUBLISH_WF_READ_VAR(
        short, FORMAT("DIFFBUF"), BUNCHES_PER_TURN, min_max->diff_buf);

    PUBLISH_READ_VAR(ai,      FORMAT("MEAN"), min_max->mean_diff);
    PUBLISH_READ_VAR(ai,      FORMAT("STD"),  min_max->var_diff);
    PUBLISH_READ_VAR(longin,  FORMAT("MAX"),  min_max->max_max);
#undef FORMAT
}


static struct min_max adc_min_max = { .read = hw_read_adc_minmax };
static struct min_max dac_min_max = { .read = hw_read_dac_minmax };


/* The ADC skew and DAC delay interact with each other so that the aggregate
 * delay is always constant.  We handle this by subtracting the ADC skew from
 * the DAC delay actually written. */

static unsigned int adc_skew;       // 0 to 3
static unsigned int dac_delay;      // 0 to 935
static unsigned int preemph_delay;  // 0 to 2
static struct epics_record *adc_skew_pv;

static void write_dac_delays(void)
{
    hw_write_dac_delay(dac_delay + 3 - adc_skew, preemph_delay);
}

static void write_dac_delay(unsigned int delay)
{
    dac_delay = delay;
    write_dac_delays();
}

static void write_preemph_delay(unsigned int delay)
{
    preemph_delay = delay;
    write_dac_delays();
}

void set_adc_skew(unsigned int skew)
{
    adc_skew = skew;
    hw_write_adc_skew(skew);
    write_dac_delays();
    trigger_record(adc_skew_pv, 0, NULL);
}


bool initialise_adc_dac(void)
{
    /* Common min/max PVs. */
    publish_minmax("ADC", &adc_min_max);
    publish_minmax("DAC", &dac_min_max);

    /* Offset control for ADC. */
    PUBLISH_WF_ACTION_P(short, "ADC:OFFSET", 4, hw_write_adc_offsets);
    adc_skew_pv = PUBLISH_READ_VAR_I(mbbi, "ADC:DELAY", adc_skew);
    PUBLISH_WRITER_P(longout, "ADC:LIMIT", hw_write_adc_limit);

    /* Direct register control for DAC. */
    PUBLISH_WRITER_P(bo, "DAC:ENABLE", hw_write_dac_enable);
    PUBLISH_WRITER_P(ulongout, "DAC:DELAY", write_dac_delay);

    /* Pre-emphasis filter interface. */
    PUBLISH_WF_ACTION_P(short, "DAC:PREEMPH", 3, hw_write_dac_preemph);
    PUBLISH_WRITER_P(mbbo, "DAC:PREEMPH:DELAY", write_preemph_delay);

    return true;
}
