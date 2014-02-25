/* Tune following. */

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <string.h>
#include <math.h>

#include "error.h"
#include "epics_device.h"
#include "epics_extra.h"
#include "hardware.h"
#include "numeric.h"

#include "tune_follow.h"

/* Control parameters written through EPICS. */
static struct ftun_control ftun_control;
static float *forward_taps;
static float *feedback_taps;
static int iir_order;

/* Waveforms extracted from raw debug buffer. */
#define DATA_LENGTH     (RAW_BUF_DATA_LENGTH / 4)
static int16_t debug_i[DATA_LENGTH];
static int16_t debug_q[DATA_LENGTH];
static int32_t debug_mag[DATA_LENGTH];
static float debug_angle[DATA_LENGTH];
static float debug_filter[DATA_LENGTH];
static struct epics_interlock *debug_interlock;

/* Scaling to convert buffer in units of 2^18 per revolution into degrees. */
static uint32_t angle_scaling;
static int angle_scaling_shift;


void update_tune_follow_debug(const int *buffer_raw)
{
    interlock_wait(debug_interlock);
    for (int i = 0; i < DATA_LENGTH; i ++)
    {
        int j = 4 * i;
        debug_i[i]      = buffer_raw[j] & 0xFFFF;
        debug_q[i]      = buffer_raw[j] >> 16;
        fixed_to_single(
            (buffer_raw[j+1] >> 16) << 2, &debug_angle[i],
            angle_scaling, angle_scaling_shift);
        debug_mag[i]    = buffer_raw[j+1] & 0xFFFF;
        fixed_to_single(
            buffer_raw[j+2], &debug_filter[i],
            angle_scaling, angle_scaling_shift);
    }
    interlock_signal(debug_interlock, NULL);
}


static void write_ftun_control(void)
{
    hw_write_ftun_control(&ftun_control);
}

static void write_ftun_iir_taps(void)
{
    int forward_taps_hw[iir_order + 1];
    int feedback_taps_hw[iir_order];
    for (int i = 0; i <= iir_order; i ++)
        forward_taps_hw[i] = (int) round(forward_taps[i] * (1 << 14));
    for (int i = 0; i < iir_order; i ++)
        feedback_taps_hw[i] = (int) round(feedback_taps[i] * (1 << 14));
    hw_write_ftun_iir_taps(forward_taps_hw, feedback_taps_hw);
}


bool initialise_tune_follow(void)
{
    iir_order = hw_read_ftun_iir_order();
    forward_taps = calloc(iir_order + 1, sizeof(float));
    feedback_taps = calloc(iir_order, sizeof(float));
    // Convert angle in units of 360 degrees = 2^18 into degrees.
    compute_scaling(360.0 / (1 << 18), &angle_scaling, &angle_scaling_shift);

    PUBLISH_ACTION("FTUN:CONTROL", write_ftun_control);
    PUBLISH_WRITE_VAR_P(longout, "FTUN:DWELL", ftun_control.dwell);
    PUBLISH_WRITE_VAR_P(longout, "FTUN:BUNCH", ftun_control.bunch);
    PUBLISH_WRITE_VAR_P(bo,      "FTUN:MULTIBUNCH", ftun_control.multibunch);
    PUBLISH_WRITE_VAR_P(mbbo,    "FTUN:INPUT", ftun_control.input_select);
    PUBLISH_WRITE_VAR_P(mbbo,    "FTUN:GAIN",  ftun_control.det_gain);

    PUBLISH_ACTION("FTUN:SET_TAPS", write_ftun_iir_taps);
    PUBLISH_WF_WRITE_VAR_P(float, "FTUN:TAPS:A", iir_order + 1, forward_taps);
    PUBLISH_WF_WRITE_VAR_P(float, "FTUN:TAPS:B", iir_order, feedback_taps);

    PUBLISH_WF_READ_VAR(short, "FTUN:DEBUG:I", DATA_LENGTH, debug_i);
    PUBLISH_WF_READ_VAR(short, "FTUN:DEBUG:Q", DATA_LENGTH, debug_q);
    PUBLISH_WF_READ_VAR(int, "FTUN:DEBUG:MAG", DATA_LENGTH, debug_mag);
    PUBLISH_WF_READ_VAR(float, "FTUN:DEBUG:ANGLE",  DATA_LENGTH, debug_angle);
    PUBLISH_WF_READ_VAR(float, "FTUN:DEBUG:FILTER", DATA_LENGTH, debug_filter);
    debug_interlock = create_interlock(
        "FTUN:DEBUG:TRIG", "FTUN:DEBUG:DONE", false);

    return true;
}
