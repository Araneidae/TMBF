/* Tune following. */

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <math.h>
#include <pthread.h>

#include "error.h"
#include "epics_device.h"
#include "epics_extra.h"
#include "hardware.h"
#include "numeric.h"
#include "detector.h"

#include "tune_follow.h"


#define FTUN_FREQ_LENGTH 1024


/* Control parameters written through EPICS. */
static struct ftun_control ftun_control;
static double target_phase;
static double max_offset;

/* Tune following. */
static struct epics_interlock *ftun_interlock;
/* The following buffers and state are needed to properly fill freq_wf and
 * manage the flow of data from the hardware. */
static float freq_wf[FTUN_FREQ_LENGTH];         // Tune fraction PV
static int raw_freq_wf[FTUN_FREQ_LENGTH];       // Raw integer version of pv
static int freq_wf_buffer[FTUN_FREQ_LENGTH];    // Data in process of being read
static size_t buffer_wf_length;                 // Number of samples in buffer
/* Data dropout detection if FTUN FIFO overruns. */
static bool dropout_seen;
static bool data_dropout;

/* Waveforms extracted from raw debug buffer. */
#define DATA_LENGTH     (RAW_BUF_DATA_LENGTH / 4)
static int16_t debug_i[DATA_LENGTH];
static int16_t debug_q[DATA_LENGTH];
static int32_t debug_mag[DATA_LENGTH];
static float debug_angle[DATA_LENGTH];
static float debug_filter[DATA_LENGTH];
static float debug_deltaf[DATA_LENGTH];
static int16_t debug_status[DATA_LENGTH];
static struct epics_interlock *debug_interlock;

/* NCO control. */
static uint32_t nco_freq;           // Base NCO frequency (in hardware units)
static uint32_t nco_freq_fraction;  // Fractional NCO freq in hardware units

/* Scaling to convert buffer in units of 2^18 per revolution into degrees. */
static uint32_t angle_scaling;
static int angle_scaling_shift;
/* Scaling to convert frequency delta into fractional tune. */
static uint32_t freq_scaling;
static int freq_scaling_shift;

/* Status readback data. */
static bool status_array[FTUN_BIT_COUNT];
static double current_angle;
static int current_magnitude;


void update_tune_follow_debug(const int *buffer_raw)
{
    interlock_wait(debug_interlock);
    for (int i = 0; i < DATA_LENGTH; i ++)
    {
        int j = 4 * i;
        /* IQ data from detector. */
        debug_i[i]      = buffer_raw[j] & 0xFFFF;
        debug_q[i]      = buffer_raw[j] >> 16;
        /* IQ angle in degrees. */
        int angle = ((buffer_raw[j+1] >> 16) << 2) - ftun_control.target_phase;
        fixed_to_single(
            (angle << 14) >> 14,
            &debug_angle[i], angle_scaling, angle_scaling_shift);
        /* IQ magnitude. */
        debug_mag[i]    = buffer_raw[j+1] & 0xFFFF;
        /* Filtered IQ angle. */
        fixed_to_single(
            buffer_raw[j+2], &debug_filter[i],
            angle_scaling, angle_scaling_shift);
        /* Frequency offset in tunes. */
        fixed_to_single(
            ((buffer_raw[j+3] & 0x3FFFF) << 14) >> 14, &debug_deltaf[i],
            freq_scaling, freq_scaling_shift);
        /* Point by point status. */
        debug_status[i] = buffer_raw[j+3] >> 18;
    }
    interlock_signal(debug_interlock, NULL);
}


static void write_ftun_control(void)
{
    ftun_control.target_phase = (int) round(pow(2, 18) / 360.0 * target_phase);
    ftun_control.max_offset = tune_to_freq(max_offset);
    hw_write_ftun_control(&ftun_control);
}


static void write_ftun_start(void)
{
    hw_write_nco_freq(nco_freq);
    hw_write_ftun_start();
}


static void write_nco_freq(double tune)
{
    nco_freq = tune_to_freq(tune);
    nco_freq_fraction = tune_to_freq(tune - round(tune));
    hw_write_nco_freq(nco_freq);
}

static double read_nco_freq(void)
{
    return BUNCHES_PER_TURN / pow(2, 32) * hw_read_nco_freq();
}

static void read_ftun_status(void)
{
    hw_read_ftun_status(status_array);
    int angle;
    hw_read_ftun_angle_mag(&angle, &current_magnitude);
    current_angle = 360.0 / pow(2, 18) * angle;
}


static void process_ftun_buffer(int *buffer, size_t read_count, bool dropout)
{
    /* Copy what we can to the raw buffer. */
    size_t copied = FTUN_FREQ_LENGTH - buffer_wf_length;
    if (copied > read_count)
        copied = read_count;
    memcpy(&freq_wf_buffer[buffer_wf_length], buffer, copied * sizeof(int));
    buffer_wf_length += copied;

    if (dropout)
        dropout_seen = true;

    /* If the raw buffer is full then we can transfer it and update. */
    if (buffer_wf_length == FTUN_FREQ_LENGTH)
    {
        interlock_wait(ftun_interlock);
        memcpy(raw_freq_wf, freq_wf_buffer, FTUN_FREQ_LENGTH * sizeof(int));
        for (size_t i = 0; i < FTUN_FREQ_LENGTH; i ++)
            fixed_to_single(nco_freq_fraction + freq_wf_buffer[i],
                &freq_wf[i], freq_scaling, freq_scaling_shift);
        data_dropout = dropout_seen;
        interlock_signal(ftun_interlock, NULL);

        buffer_wf_length = 0;

        dropout_seen = false;
    }

    /* Copy any residue into the raw buffer. */
    size_t residue = read_count - copied;
    memcpy(&freq_wf_buffer[buffer_wf_length],
        &buffer[copied], residue * sizeof(int));
    buffer_wf_length += residue;
}

static void *poll_tune_follow(void *context)
{
    while (true)
    {
        int ftun_buffer[FTUN_FIFO_SIZE];
        bool dropout;
        size_t read_count = hw_read_ftun_buffer(ftun_buffer, &dropout);
        if (read_count > 0)
            process_ftun_buffer(ftun_buffer, read_count, dropout);

        usleep(100000); // 10 Hz polling
    }
    return NULL;
}


bool initialise_tune_follow(void)
{
    /* Convert angle in units of 360 degrees = 2^18 into degrees. */
    compute_scaling(360.0 / (1 << 18), &angle_scaling, &angle_scaling_shift);
    /* Convert frequency in units of 2^32 revolutions into fractional tune. */
    compute_scaling(BUNCHES_PER_TURN / pow(2, 32),
        &freq_scaling, &freq_scaling_shift);

    /* Control PVs for setting tune follow parameters. */
    PUBLISH_ACTION("FTUN:CONTROL", write_ftun_control);
    PUBLISH_WRITE_VAR_P(longout, "FTUN:DWELL",  ftun_control.dwell);
    PUBLISH_WRITE_VAR_P(bo,      "FTUN:BLANKING", ftun_control.blanking);
    PUBLISH_WRITE_VAR_P(bo,      "FTUN:MULTIBUNCH", ftun_control.multibunch);
    PUBLISH_WRITE_VAR_P(longout, "FTUN:BUNCH",  ftun_control.bunch);
    PUBLISH_WRITE_VAR_P(mbbo,    "FTUN:INPUT",  ftun_control.input_select);
    PUBLISH_WRITE_VAR_P(mbbo,    "FTUN:GAIN",   ftun_control.det_gain);
    PUBLISH_WRITE_VAR_P(mbbo,    "FTUN:IIR",    ftun_control.iir_rate);
    PUBLISH_WRITE_VAR_P(ao,      "FTUN:TARGET", target_phase);
    PUBLISH_WRITE_VAR_P(longout, "FTUN:SCALE",  ftun_control.feedback_scale);
    PUBLISH_WRITE_VAR_P(longout, "FTUN:MINMAG", ftun_control.min_magnitude);
    PUBLISH_WRITE_VAR_P(ao,      "FTUN:MAXDELTA", max_offset);

    /* Tune following action. */
    PUBLISH_ACTION("FTUN:START", write_ftun_start);
    PUBLISH_ACTION("FTUN:STOP", hw_write_ftun_stop);

    PUBLISH_READ_VAR(bi, "FTUN:FREQ:DROPOUT", data_dropout);
    PUBLISH_WF_READ_VAR(float, "FTUN:FREQ", FTUN_FREQ_LENGTH, freq_wf);
    PUBLISH_WF_READ_VAR(int, "FTUN:RAWFREQ", FTUN_FREQ_LENGTH, raw_freq_wf);
    ftun_interlock = create_interlock("FTUN:TRIG", "FTUN:DONE", false);

    /* Debug PVs available when fast buffer in debug mode. */
    PUBLISH_WF_READ_VAR(short, "FTUN:DEBUG:I", DATA_LENGTH, debug_i);
    PUBLISH_WF_READ_VAR(short, "FTUN:DEBUG:Q", DATA_LENGTH, debug_q);
    PUBLISH_WF_READ_VAR(int, "FTUN:DEBUG:MAG", DATA_LENGTH, debug_mag);
    PUBLISH_WF_READ_VAR(float, "FTUN:DEBUG:ANGLE",  DATA_LENGTH, debug_angle);
    PUBLISH_WF_READ_VAR(float, "FTUN:DEBUG:FILTER", DATA_LENGTH, debug_filter);
    PUBLISH_WF_READ_VAR(float, "FTUN:DEBUG:DELTAF", DATA_LENGTH, debug_deltaf);
    PUBLISH_WF_READ_VAR(short, "FTUN:DEBUG:STATUS", DATA_LENGTH, debug_status);
    debug_interlock = create_interlock(
        "FTUN:DEBUG:TRIG", "FTUN:DEBUG:DONE", false);

    /* Status bits. */
    PUBLISH_ACTION("FTUN:STAT:SCAN", read_ftun_status);
    PUBLISH_READ_VAR(bi, "FTUN:STAT:INP", status_array[FTUN_STAT_INP]);
    PUBLISH_READ_VAR(bi, "FTUN:STAT:ACC", status_array[FTUN_STAT_ACC]);
    PUBLISH_READ_VAR(bi, "FTUN:STAT:DET", status_array[FTUN_STAT_DET]);
    PUBLISH_READ_VAR(bi, "FTUN:STAT:MAG", status_array[FTUN_STAT_MAG]);
    PUBLISH_READ_VAR(bi, "FTUN:STAT:VAL", status_array[FTUN_STAT_VAL]);
    PUBLISH_READ_VAR(bi, "FTUN:STOP:INP", status_array[FTUN_STOP_INP]);
    PUBLISH_READ_VAR(bi, "FTUN:STOP:ACC", status_array[FTUN_STOP_ACC]);
    PUBLISH_READ_VAR(bi, "FTUN:STOP:DET", status_array[FTUN_STOP_DET]);
    PUBLISH_READ_VAR(bi, "FTUN:STOP:MAG", status_array[FTUN_STOP_MAG]);
    PUBLISH_READ_VAR(bi, "FTUN:STOP:VAL", status_array[FTUN_STOP_VAL]);
    PUBLISH_READ_VAR(bi, "FTUN:ACTIVE", status_array[FTUN_STAT_RUNNING]);
    PUBLISH_READ_VAR(ai, "FTUN:ANGLE", current_angle);
    PUBLISH_READ_VAR(longin, "FTUN:MAG", current_magnitude);

    /* NCO control. */
    PUBLISH_WRITER_P(ao,   "NCO:FREQ", write_nco_freq);
    PUBLISH_WRITER_P(mbbo, "NCO:GAIN", hw_write_nco_gain);
    PUBLISH_READER(ai,     "NCO:FREQ", read_nco_freq);

    pthread_t thread_id;
    return TEST_0(pthread_create(&thread_id, NULL, poll_tune_follow, NULL));
}
