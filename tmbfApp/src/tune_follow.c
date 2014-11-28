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
#include "tune.h"

#include "tune_follow.h"


#define FTUN_FREQ_LENGTH 4096


/* Control parameters written through EPICS. */
static struct ftun_control ftun_control;
static double target_phase;
static double max_offset;

/* IIR time constants. */
static double filter_iir_tc;
static double iq_iir_tc;
static double freq_iir_tc;

/* Frequency dependent delay compensation as an angle. */
static double delay_offset_degrees;

/* Tune following. */
static struct epics_interlock *ftun_interlock;
/* The following buffers and state are needed to properly fill freq_wf and
 * manage the flow of data from the hardware. */
static float freq_wf[FTUN_FREQ_LENGTH];         // Tune fraction PV
static int raw_freq_wf[FTUN_FREQ_LENGTH];       // Raw integer version of pv
static int freq_wf_buffer[FTUN_FREQ_LENGTH];    // Data in process of being read
static size_t buffer_wf_length;                 // Number of samples in buffer
static double mean_nco_frequency;
/* Data dropout detection if FTUN FIFO overruns. */
static bool dropout_seen;
static bool data_dropout;

/* Waveforms extracted from raw debug buffer. */
#define DATA_LENGTH     (BUF_DATA_LENGTH / 4)
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
enum { FTUN_ACTIVE_STOPPED, FTUN_ACTIVE_ARMED, FTUN_ACTIVE_RUNNING };
static unsigned int ftun_status;
static int current_i;
static int current_q;
static double current_angle;
static double delta_angle;
static int current_magnitude;

/* Min/max info. */
static int minimum_i;
static int maximum_i;
static int minimum_q;
static int maximum_q;
static double iq_variation;



/* Frequency dependent phase offset adjustment. */
static double closed_loop_delay = 1;


void update_tune_follow_debug(const int *buffer_raw)
{
    interlock_wait(debug_interlock);
    for (int i = 0; i < DATA_LENGTH; i ++)
    {
        int j = 4 * i;
        /* IQ data from detector. */
        debug_i[i]      = (int16_t) (buffer_raw[j] & 0xFFFF);
        debug_q[i]      = (int16_t) (buffer_raw[j] >> 16);
        /* IQ angle in degrees. */
        fixed_to_single(
            (int) (buffer_raw[j+1] & 0xFFFF0000) >> 14,
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
        debug_status[i] = (int16_t) (buffer_raw[j+3] >> 18);
    }
    interlock_signal(debug_interlock, NULL);
}


static double update_iir_tc(unsigned int iir_rate, unsigned int interval)
{
    if (iir_rate == 0)
        return 0;
    else
    {
        double dwell_time = ftun_control.dwell / 533.830;   // One turn in ms
        return dwell_time / -log(1 - 1./(1 << (iir_rate * interval)));
    }
}


static void update_delay_offset(void)
{
    int adc_delay, fir_delay;
    hw_read_ftun_delays(&adc_delay, &fir_delay);
    int delay =
        ftun_control.input_select == FTUN_IN_ADC ? adc_delay : fir_delay;
    delay += BUNCHES_PER_TURN;
    delay += lround(closed_loop_delay * BUNCHES_PER_TURN);
    delay_offset_degrees =
        360.0 / pow(2, 32) * (double) (int) (nco_freq * (uint32_t) delay);
}


/* Fold arbitrary angle (in degrees) into the range +- 180 degrees. */
static double wrap_angle(double angle)
{
    if (angle > 180)
        return angle - 360;
    else if (angle < -180)
        return angle + 360;
    else
        return angle;
}


static void write_ftun_control(void)
{
    update_delay_offset();

    ftun_control.target_phase = lround(
        pow(2, 18) / 360.0 * wrap_angle(target_phase + delay_offset_degrees));
    ftun_control.max_offset = tune_to_freq(max_offset);
    hw_write_ftun_control(&ftun_control);

    filter_iir_tc = update_iir_tc(ftun_control.iir_rate, 1);
    iq_iir_tc     = update_iir_tc(ftun_control.iq_iir_rate, 2);
    freq_iir_tc   = update_iir_tc(ftun_control.freq_iir_rate, 2);
}


static void write_ftun_start(void)
{
    hw_write_nco_freq(nco_freq);

    hw_write_ftun_disarm();
    hw_write_ftun_enable(true);
    hw_write_ftun_start();
}

static void write_ftun_arm(void)
{
    hw_write_nco_freq(nco_freq);

    hw_write_ftun_enable(true);
    hw_write_ftun_arm();
}

static void write_ftun_stop(void)
{
    hw_write_ftun_disarm();
    hw_write_ftun_enable(false);
}


static void write_nco_freq(double tune)
{
    nco_freq = tune_to_freq(tune);
    nco_freq_fraction = tune_to_freq(tune - floor(tune));
    hw_write_nco_freq(nco_freq);
}

static double read_nco_freq(void)
{
    double result = BUNCHES_PER_TURN / pow(2, 32) * nco_freq;
    int offset;
    if (hw_read_ftun_frequency(&offset))
        result += BUNCHES_PER_TURN / pow(2, 44) * offset;

    return result;
}

#define SQR(x)  ((x) * (x))

static void update_iq_angle_mag(void)
{
    hw_read_ftun_iq(&current_i, &current_q);

    /* Compute magnitude and angle. */
    current_angle = wrap_angle(
        180.0 / M_PI * atan2(current_q, current_i) - delay_offset_degrees);
    /* There is a tricky detail in this calculation.  When IQ saturates one of
     * the possible limit values is (I,Q) = (-2^15,-2^15) with I^2+Q^2 = 2^31,
     * which is too large for an int, hence the casts to unsigned int. */
    current_magnitude = (int) sqrt(
        (unsigned int) SQR(current_i) + (unsigned int) SQR(current_q));

    delta_angle = wrap_angle(current_angle - target_phase);
}

static void update_minmax(void)
{
    int min, max;
    if (hw_read_ftun_i_minmax(&min, &max))
    {
        minimum_i = min;
        maximum_i = max;
    }
    if (hw_read_ftun_q_minmax(&min, &max))
    {
        minimum_q = min;
        maximum_q = max;
    }

    /* Estimate variation from peak to peak movement. */
    double raw_variation = sqrt(
        (SQR((double) maximum_i - minimum_i) +
         SQR((double) maximum_q - minimum_q)) / 2);
    if (current_magnitude > 0)
        iq_variation = raw_variation / current_magnitude;
    else
        iq_variation = raw_variation;
}

static void read_ftun_status(void)
{
    hw_read_ftun_status_bits(status_array);
    ftun_status = hw_read_ftun_status();

    update_iq_angle_mag();
    update_minmax();

    if (ftun_status != FTUN_ACTIVE_RUNNING)
        update_tune_pll_tune(false, 0, 0);
}


static double compute_mean_frequency(void)
{
    double mean = 0;
    for (size_t i = 0; i < buffer_wf_length; i ++)
        mean += freq_wf[i];
    return mean / buffer_wf_length;
}


static void update_ftun_buffer(void)
{
    interlock_wait(ftun_interlock);
    memcpy(raw_freq_wf, freq_wf_buffer, FTUN_FREQ_LENGTH * sizeof(int));
    for (size_t i = 0; i < buffer_wf_length; i ++)
        fixed_to_single((int) nco_freq_fraction + freq_wf_buffer[i],
            &freq_wf[i], freq_scaling, freq_scaling_shift);
    for (size_t i = buffer_wf_length; i < FTUN_FREQ_LENGTH; i ++)
        freq_wf[i] = freq_wf[buffer_wf_length - 1];
    data_dropout = dropout_seen;
    interlock_signal(ftun_interlock, NULL);

    mean_nco_frequency = compute_mean_frequency();
    buffer_wf_length = 0;

    dropout_seen = false;

    update_tune_pll_tune(true, mean_nco_frequency, current_angle);
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

    /* Update output buffer either when the raw buffer is full or when it is
     * non-empty and we've stopped running. */
    /* If the raw buffer is full then we can transfer it and update. */
    if (buffer_wf_length == FTUN_FREQ_LENGTH  ||
        (buffer_wf_length > 0  &&  hw_read_ftun_status() != FTUN_RUNNING))
        update_ftun_buffer();

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
        process_ftun_buffer(ftun_buffer, read_count, dropout);

        usleep(10000); // 100 Hz polling
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
    PUBLISH_WRITE_VAR_P(longout, "FTUN:KI",     ftun_control.i_scale);
    PUBLISH_WRITE_VAR_P(longout, "FTUN:KP",     ftun_control.p_scale);
    PUBLISH_WRITE_VAR_P(longout, "FTUN:MINMAG", ftun_control.min_magnitude);
    PUBLISH_WRITE_VAR_P(ao,      "FTUN:MAXDELTA", max_offset);
    PUBLISH_WRITE_VAR_P(mbbo,    "FTUN:IQ:IIR", ftun_control.iq_iir_rate);
    PUBLISH_WRITE_VAR_P(mbbo,    "FTUN:FREQ:IIR", ftun_control.freq_iir_rate);
    PUBLISH_WRITE_VAR_P(ao,      "FTUN:LOOP:DELAY", closed_loop_delay);

    /* IIR time constants updated when IIR controls set. */
    PUBLISH_READ_VAR(ai, "FTUN:IIR:TC",         filter_iir_tc);
    PUBLISH_READ_VAR(ai, "FTUN:IQ:IIR:TC",      iq_iir_tc);
    PUBLISH_READ_VAR(ai, "FTUN:FREQ:IIR:TC",    freq_iir_tc);

    PUBLISH_READ_VAR(ai, "FTUN:PHASE:OFFSET",   delay_offset_degrees);

    /* Tune following action. */
    PUBLISH_ACTION("FTUN:START", write_ftun_start);
    PUBLISH_ACTION("FTUN:ARM", write_ftun_arm);
    PUBLISH_ACTION("FTUN:STOP", write_ftun_stop);

    PUBLISH_READ_VAR(bi, "FTUN:FREQ:DROPOUT", data_dropout);
    PUBLISH_WF_READ_VAR(float, "FTUN:FREQ", FTUN_FREQ_LENGTH, freq_wf);
    PUBLISH_WF_READ_VAR(int, "FTUN:RAWFREQ", FTUN_FREQ_LENGTH, raw_freq_wf);
    PUBLISH_READ_VAR(ai, "NCO:FREQ:MEAN", mean_nco_frequency);
    ftun_interlock = create_interlock("FTUN", false);

    /* Debug PVs available when fast buffer in debug mode. */
    PUBLISH_WF_READ_VAR(short, "FTUN:DEBUG:I", DATA_LENGTH, debug_i);
    PUBLISH_WF_READ_VAR(short, "FTUN:DEBUG:Q", DATA_LENGTH, debug_q);
    PUBLISH_WF_READ_VAR(int, "FTUN:DEBUG:MAG", DATA_LENGTH, debug_mag);
    PUBLISH_WF_READ_VAR(float, "FTUN:DEBUG:ANGLE",  DATA_LENGTH, debug_angle);
    PUBLISH_WF_READ_VAR(float, "FTUN:DEBUG:FILTER", DATA_LENGTH, debug_filter);
    PUBLISH_WF_READ_VAR(float, "FTUN:DEBUG:DELTAF", DATA_LENGTH, debug_deltaf);
    PUBLISH_WF_READ_VAR(short, "FTUN:DEBUG:STATUS", DATA_LENGTH, debug_status);
    debug_interlock = create_interlock("FTUN:DEBUG", false);

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
    PUBLISH_READ_VAR(mbbi, "FTUN:ACTIVE", ftun_status);
    PUBLISH_READ_VAR(ai, "FTUN:ANGLE", current_angle);
    PUBLISH_READ_VAR(ai, "FTUN:ANGLEDELTA", delta_angle);
    PUBLISH_READ_VAR(longin, "FTUN:MAG", current_magnitude);

    PUBLISH_READ_VAR(longin, "FTUN:I", current_i);
    PUBLISH_READ_VAR(longin, "FTUN:Q", current_q);
    PUBLISH_READ_VAR(longin, "FTUN:I:MIN", minimum_i);
    PUBLISH_READ_VAR(longin, "FTUN:I:MAX", maximum_i);
    PUBLISH_READ_VAR(longin, "FTUN:Q:MIN", minimum_q);
    PUBLISH_READ_VAR(longin, "FTUN:Q:MAX", maximum_q);
    PUBLISH_READ_VAR(ai, "FTUN:IQ:VAR", iq_variation);

    /* NCO control. */
    PUBLISH_WRITER_P(ao,   "NCO:FREQ", write_nco_freq);
    PUBLISH_WRITER_P(mbbo, "NCO:GAIN", hw_write_nco_gain);
    PUBLISH_READER(ai,     "NCO:FREQ", read_nco_freq);

    pthread_t thread_id;
    return TEST_0(pthread_create(&thread_id, NULL, poll_tune_follow, NULL));
}
