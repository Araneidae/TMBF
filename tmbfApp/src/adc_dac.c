/* EPICS interface to ADC and DAC readout and control. */

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include "error.h"
#include "hardware.h"
#include "epics_device.h"
#include "numeric.h"
#include "tmbf.h"

#include "adc_dac.h"



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Min/Max Buffer */

struct min_max {
    /* Support for scaling min and max buffers values to floating point for
     * display. */
    int max_value;
    uint32_t scaling;
    int shift;

    /* Function for reading min/max buffer. */
    void (*read)(short *min_buf, short *max_buf);
    /* Values exported through EPICS. */
    float min_buf[BUNCHES_PER_TURN];
    float max_buf[BUNCHES_PER_TURN];
    float diff_buf[BUNCHES_PER_TURN];
    double mean_diff;
    double var_diff;
    double max_max;

    /* Support for filter coefficient update. */
    void (*write_filter)(int taps[]);
    int tap_count;
};


#define MAX(a, b) ((a) > (b) ? (a) : (b))

/* Reads min & max waveforms from selected hardware and computes the remaining
 * waveforms and values of interest. */
static bool read_minmax(void *context, const bool *ignore)
{
    struct min_max *min_max = context;

    short min_buf[BUNCHES_PER_TURN];
    short max_buf[BUNCHES_PER_TURN];
    min_max->read(min_buf, max_buf);

    int sum_diff = 0;
    long long int sum_var = 0;
    int max_max = 0;
    uint32_t scaling = min_max->scaling;
    int shift = min_max->shift;
    for (unsigned int i = 0; i < BUNCHES_PER_TURN; ++i)
    {
        short int diff = max_buf[i] - min_buf[i];
        min_max->diff_buf[i] = diff;
        sum_diff += diff;
        sum_var  += (int) diff * diff;
        max_max = MAX(max_max, abs(max_buf[i]));
        max_max = MAX(max_max, abs(min_buf[i]));

        fixed_to_single(min_buf[i], &min_max->min_buf[i], scaling, shift);
        fixed_to_single(max_buf[i], &min_max->max_buf[i], scaling, shift);
        fixed_to_single(diff, &min_max->diff_buf[i], scaling, shift);
    }

    min_max->max_max = (double) max_max / min_max->max_value;
    min_max->mean_diff =
        sum_diff / (double) BUNCHES_PER_TURN / min_max->max_value;
    min_max->var_diff  =
        sum_var  / (double) BUNCHES_PER_TURN /
            min_max->max_value / min_max->max_value -
        min_max->mean_diff * min_max->mean_diff;
    return true;
}

static void publish_minmax(
    const char *name, struct min_max *min_max, int max_value)
{
    min_max->max_value = max_value;
    compute_scaling(1.0 / max_value, &min_max->scaling, &min_max->shift);

    char buffer[20];
#define FORMAT(record_name) \
    (sprintf(buffer, "%s:%s", name, record_name), buffer)

    PUBLISH(bo, FORMAT("SCAN"), .write = read_minmax, .context = min_max);

    PUBLISH_WF_READ_VAR(
        float, FORMAT("MINBUF"), BUNCHES_PER_TURN, min_max->min_buf);
    PUBLISH_WF_READ_VAR(
        float, FORMAT("MAXBUF"), BUNCHES_PER_TURN, min_max->max_buf);
    PUBLISH_WF_READ_VAR(
        float, FORMAT("DIFFBUF"), BUNCHES_PER_TURN, min_max->diff_buf);

    PUBLISH_READ_VAR(ai, FORMAT("MEAN"), min_max->mean_diff);
    PUBLISH_READ_VAR(ai, FORMAT("STD"),  min_max->var_diff);
    PUBLISH_READ_VAR(ai, FORMAT("MAX"),  min_max->max_max);
#undef FORMAT
}


/* Support for writing waveforms with clipping. */
static void write_filter_coeffs(void *context, float coeffs[], size_t *length)
{
    struct min_max *min_max = context;
    int taps[min_max->tap_count];
    float_array_to_int(min_max->tap_count, coeffs, taps, 16, 1);
    min_max->write_filter(taps);
    *length = min_max->tap_count;
}


static struct min_max adc_min_max = {
    .read = hw_read_adc_minmax,
    .write_filter = hw_write_adc_filter,
    .tap_count = 12
};
static struct min_max dac_min_max = {
    .read = hw_read_dac_minmax,
    .write_filter = hw_write_dac_preemph,
    .tap_count = 3
};


/* The ADC skew and DAC delay interact with each other so that the aggregate
 * delay is always constant.  We handle this by subtracting the ADC skew from
 * the DAC delay actually written. */

static unsigned int adc_skew;       // 0 to 3
static unsigned int dac_delay;      // 0 to 935
static struct epics_record *adc_skew_pv;

static void write_dac_delays(void)
{
    hw_write_dac_delay(dac_delay + 3 - adc_skew);
}

static void write_dac_delay(unsigned int delay)
{
    dac_delay = delay;
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
    publish_minmax("ADC", &adc_min_max, 1 << 15);
    publish_minmax("DAC", &dac_min_max, 1 << 13);

    /* Offset control for ADC. */
    PUBLISH_WF_ACTION_P(short, "ADC:OFFSET", 4, hw_write_adc_offsets);
    PUBLISH_WAVEFORM(float, "ADC:FILTER", 12, write_filter_coeffs,
        .context = &adc_min_max, .persist = true);
    PUBLISH_WRITER_P(mbbo, "ADC:FILTER:DELAY", hw_write_adc_filter_delay);
    adc_skew_pv = PUBLISH_READ_VAR_I(mbbi, "ADC:SKEW", adc_skew);
    PUBLISH_WRITER_P(longout, "ADC:LIMIT", hw_write_adc_limit);

    /* Direct register control for DAC. */
    PUBLISH_WRITER_P(bo, "DAC:ENABLE", hw_write_dac_enable);
    PUBLISH_WRITER_P(ulongout, "DAC:DELAY", write_dac_delay);

    /* Pre-emphasis filter interface. */
    PUBLISH_WAVEFORM(float, "DAC:PREEMPH", 3, write_filter_coeffs,
        .context = &dac_min_max, .persist = true);
    PUBLISH_WRITER_P(mbbo, "DAC:PREEMPH:DELAY", hw_write_dac_filter_delay);

    return true;
}
