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
#include "epics_extra.h"
#include "numeric.h"
#include "tmbf.h"

#include "adc_dac.h"



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* ADC/DAC context */

/* Definitions shared between ADC and DAC control, including both min/max
 * buffers and the compensation filters. */
struct adc_dac {
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
    double max_diff;

    /* Support for filter coefficient update. */
    void (*write_filter)(int taps[]);
    void (*write_delay)(unsigned int delay);
    size_t tap_count;
    const float *default_taps;
};


#define MAX(a, b) ((a) > (b) ? (a) : (b))

/* Reads min & max waveforms from selected hardware and computes the remaining
 * waveforms and values of interest. */
static bool read_minmax(void *context, const bool *ignore)
{
    struct adc_dac *adc_dac = context;

    short min_buf[BUNCHES_PER_TURN];
    short max_buf[BUNCHES_PER_TURN];
    adc_dac->read(min_buf, max_buf);

    int sum_diff = 0;
    long long int sum_var = 0;
    int max_max = 0;
    int max_diff = 0;
    uint32_t scaling = adc_dac->scaling;
    int shift = adc_dac->shift;
    for (unsigned int i = 0; i < BUNCHES_PER_TURN; ++i)
    {
        int diff = max_buf[i] - min_buf[i];
        sum_diff += diff;
        sum_var  += diff * diff;
        max_max = MAX(max_max, abs(max_buf[i]));
        max_max = MAX(max_max, abs(min_buf[i]));
        max_diff = MAX(max_diff, diff);

        fixed_to_single(min_buf[i], &adc_dac->min_buf[i], scaling, shift);
        fixed_to_single(max_buf[i], &adc_dac->max_buf[i], scaling, shift);
        fixed_to_single(diff, &adc_dac->diff_buf[i], scaling, shift);
    }

    adc_dac->max_max = (double) max_max / adc_dac->max_value;
    adc_dac->max_diff = (double) max_diff / adc_dac->max_value;
    adc_dac->mean_diff =
        (double) sum_diff / BUNCHES_PER_TURN / adc_dac->max_value;
    adc_dac->var_diff  =
        (double) sum_var / BUNCHES_PER_TURN /
            adc_dac->max_value / adc_dac->max_value -
        adc_dac->mean_diff * adc_dac->mean_diff;
    return true;
}



/* Support for writing waveforms with clipping. */
static void write_filter_coeffs(void *context, float coeffs[], size_t *length)
{
    struct adc_dac *adc_dac = context;
    int taps[adc_dac->tap_count];
    float_array_to_int(adc_dac->tap_count, coeffs, taps, 16, 1);
    adc_dac->write_filter(taps);
    *length = adc_dac->tap_count;
}


static void init_wf_taps(void *context, float *taps, size_t *length)
{
    struct adc_dac *adc_dac = context;
    ASSERT_OK(adc_dac->tap_count == *length);
    for (size_t i = 0; i < adc_dac->tap_count; i ++)
        taps[i] = adc_dac->default_taps[i];
}

static bool init_group_delay(void *context, unsigned int *value)
{
    *value = 1;
    return true;
}

/* Publishes all common PVs associated with ADC and DAC interfaces.  This
 * includes both the min/max interface and the compensation filter. */
static void publish_adc_dac(const char *name, struct adc_dac *adc_dac)
{
    compute_scaling(
        1 / (float) adc_dac->max_value, &adc_dac->scaling, &adc_dac->shift);

    char buffer[20];
#define FORMAT(record_name) \
    (sprintf(buffer, "%s:%s", name, record_name), buffer)

    PUBLISH(bo, FORMAT("SCAN"), .write = read_minmax, .context = adc_dac);

    PUBLISH_WF_READ_VAR(
        float, FORMAT("MINBUF"), BUNCHES_PER_TURN, adc_dac->min_buf);
    PUBLISH_WF_READ_VAR(
        float, FORMAT("MAXBUF"), BUNCHES_PER_TURN, adc_dac->max_buf);
    PUBLISH_WF_READ_VAR(
        float, FORMAT("DIFFBUF"), BUNCHES_PER_TURN, adc_dac->diff_buf);

    PUBLISH_READ_VAR(ai, FORMAT("MEAN"), adc_dac->mean_diff);
    PUBLISH_READ_VAR(ai, FORMAT("STD"),  adc_dac->var_diff);
    PUBLISH_READ_VAR(ai, FORMAT("MAX"),  adc_dac->max_max);
    PUBLISH_READ_VAR(ai, FORMAT("MAXDIFF"),  adc_dac->max_diff);

    /* Input or output filter control and compensation. */
    PUBLISH_WAVEFORM(float, FORMAT("FILTER"),
        adc_dac->tap_count, write_filter_coeffs,
        .context = adc_dac, .persist = true, .init = init_wf_taps);
    PUBLISH_WRITER_P(mbbo, FORMAT("FILTER:DELAY"),
        adc_dac->write_delay, .init = init_group_delay);
#undef FORMAT
}


static struct adc_dac adc_context = {
    .max_value = 1 << 15,
    .read = hw_read_adc_minmax,
    .write_filter = hw_write_adc_filter,
    .write_delay = hw_write_adc_filter_delay,
    .tap_count = 12,
    .default_taps = (float []) { 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0 }
};

static struct adc_dac dac_context = {
    .max_value = 1 << 13,
    .read = hw_read_dac_minmax,
    .write_filter = hw_write_dac_preemph,
    .write_delay = hw_write_dac_filter_delay,
    .tap_count = 3,
    .default_taps = (float []) { 0, 1, 0 }
};


/* The ADC skew and DAC delay interact with each other so that the aggregate
 * delay is always constant.  We handle this by subtracting the ADC skew from
 * the DAC delay actually written. */

static unsigned int adc_skew;       // 0 to 3
static unsigned int dac_delay;      // 0 to 935
static struct in_epics_record_mbbi *adc_skew_pv;

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
    WRITE_IN_RECORD(mbbi, adc_skew_pv, skew);
}


static void write_adc_limit(double limit)
{
    hw_write_adc_limit(lround(limit * (1 << 15)));
}


bool initialise_adc_dac(void)
{
    /* Common min/max PVs. */
    publish_adc_dac("ADC", &adc_context);
    publish_adc_dac("DAC", &dac_context);

    PUBLISH_WF_ACTION_P(int, "ADC:OFFSET", 4, hw_write_adc_offsets);
    adc_skew_pv = PUBLISH_IN_VALUE_I(mbbi, "ADC:SKEW");
    PUBLISH_WRITER_P(ao, "ADC:LIMIT", write_adc_limit);

    PUBLISH_WRITER_P(bo, "DAC:ENABLE", hw_write_dac_enable);
    PUBLISH_WRITER_P(ulongout, "DAC:DELAY", write_dac_delay);

    return true;
}
