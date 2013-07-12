/* Detector and sweep control. */

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include "error.h"
#include "hardware.h"
#include "epics_device.h"
#include "epics_extra.h"
#include "sequencer.h"
#include "numeric.h"

#include "detector.h"



#define TUNE_LENGTH    (BUF_DATA_LENGTH / 4)


/* Detector bunches: one bunch for each operating channel. */
static unsigned int detector_bunches[4];

/* The tune scale is used both for presentation and as part of the tune
 * calculation, so we hang onto it here. */
static float tune_scale[TUNE_LENGTH];
static int scale_length;
/* Each time a sequencer setting changes we'll need to recompute the scale. */
static bool tune_scale_needs_refresh = true;
static struct epics_record *tune_scale_pv;

/* Computed I/Q waveforms.  One pair for each channel plus an aggregate across
 * all four channels. */
static short wf_i[4][TUNE_LENGTH];
static short wf_q[4][TUNE_LENGTH];
static short wf_i_mean[TUNE_LENGTH];
static short wf_q_mean[TUNE_LENGTH];
static struct epics_interlock *iq_trigger;


/* Waveforms for compensating IQ by group_delay.
 *
 * The DDC skew is a group delay in ns.  We translate this into a phase
 * advance of
 *
 *      phase = 0.5 * skew * 2pi * frequency / 936
 *
 * Here the frequency is in tunes, one revolution per turn, which we convert
 * into one revolution per bunch, or half a revolution per nanosecond (at
 * 500MHz RF frequency).
 *    For economy of calculation, we precompute the rotations as scaled
 * integers so that the final computation can be a simple integer
 * multiplication (one instruction when the compiler is in the right mood). */
static int group_delay = 0;
static int rotate_I[TUNE_LENGTH];  // 2**30 * cos(phase)
static int rotate_Q[TUNE_LENGTH];  // 2**30 * sin(phase)

/* Helper constants for fast tune scale and rotation waveform calculations
 * corresponding to multiplication by 936*2^-33. */
static uint32_t wf_scaling;
static int wf_shift;



unsigned int tune_to_freq(double tune)
{
    return (unsigned int) round(tune * (pow(2, 33) / (double) MAX_BUNCH_COUNT));
}


static void store_one_tune_freq(int freq, int ix)
{
    fixed_to_single(freq, &tune_scale[ix], wf_scaling, wf_shift);
    cos_sin(-freq * group_delay, &rotate_I[ix], &rotate_Q[ix]);
}

/* Computes frequency scale directly from sequencer settings.  Triggered
 * whenever the sequencer state changes. */
static void update_det_scale(void)
{
    unsigned int state;
    const struct seq_entry *sequencer_table = read_sequencer_table(&state);

    int ix = 0;
    unsigned int f0 = 0;
    for ( ; state > 0  &&  ix < TUNE_LENGTH; state --)
    {
        const struct seq_entry *entry = &sequencer_table[state];
        f0 = entry->start_freq;
        for (unsigned int i = 0;
             i < entry->capture_count  &&  ix < TUNE_LENGTH; i ++)
        {
            store_one_tune_freq(f0, ix++);
            f0 += entry->delta_freq;
        }
    }

    /* Pad the rest of the scale.  The last frequency is a good a choice as any,
     * anything that goes here is invalid. */
    scale_length = ix;
    while (ix < TUNE_LENGTH)
        store_one_tune_freq(f0, ix++);

    tune_scale_needs_refresh = false;
}


void seq_settings_changed(void)
{
    tune_scale_needs_refresh = true;
}


static void set_group_delay(int delay)
{
    group_delay = delay;
    seq_settings_changed(); // Not really true, but does enough to get by
}


/* This is called when IQ data has been read into the fast buffer.  Process it
 * according to our current configuration into separate IQ wavforms.  One
 * separate I/Q value is extracted from each channel and rotated to compensate
 * for the precomputed group delay, and an average is also stored. */
void update_iq(const short buffer_low[], const short buffer_high[])
{
printf("update_iq\n");
    if (tune_scale_needs_refresh)
    {
        update_det_scale();
        trigger_record(tune_scale_pv, 0, NULL);
    }

    interlock_wait(iq_trigger);

    for (int i = 0; i < scale_length; i ++)
    {
        int rot_I = rotate_I[i];
        int rot_Q = rotate_Q[i];
        int I_sum = 0, Q_sum = 0;
        for (int channel = 0; channel < 4; channel ++)
        {
            int raw_I = buffer_low[4 * i + channel];
            int raw_Q = buffer_high[4 * i + channel];
            int I = MulSS(raw_I, rot_I) - MulSS(raw_Q, rot_Q);
            int Q = MulSS(raw_I, rot_Q) - MulSS(raw_Q, rot_I);
            wf_i[channel][i] = I;
            wf_q[channel][i] = Q;
            I_sum += I;
            Q_sum += Q;
        }
        wf_i_mean[i] = I_sum;
        wf_q_mean[i] = Q_sum;
    }

    /* Pad rest of results with zeros: none of this data is valid! */
    for (int i = scale_length; i < TUNE_LENGTH; i ++)
    {
        for (int channel = 0; channel < 4; channel ++)
        {
            wf_i[channel][i] = 0;
            wf_q[channel][i] = 0;
        }
        wf_i_mean[i] = 0;
        wf_q_mean[i] = 0;
    }

    interlock_signal(iq_trigger, NULL);
}


static void write_det_bunches(void)
{
    hw_write_det_bunches(detector_bunches);
}


static void write_nco_freq(double tune)
{
    hw_write_nco_freq(tune_to_freq(tune));
}


static void publish_bunch(int bunch)
{
    char buffer[20];
#define FORMAT(name) \
    (sprintf(buffer, "DET:%s%d", name, bunch), buffer)

    PUBLISH_WRITE_VAR_P(ulongout, FORMAT("BUNCH"), detector_bunches[bunch]);
    PUBLISH_WF_READ_VAR(short, FORMAT("I:"), TUNE_LENGTH, wf_i[bunch]);
    PUBLISH_WF_READ_VAR(short, FORMAT("Q:"), TUNE_LENGTH, wf_q[bunch]);

#undef FORMAT
}



bool initialise_detector(void)
{
    PUBLISH_WRITER_P(bo, "DET:MODE", hw_write_det_mode);
    PUBLISH_WRITER_P(mbbo, "DET:GAIN", hw_write_det_gain);
    PUBLISH_WRITER_P(longout, "DET:SKEW", set_group_delay);

    for (int i = 0; i < 4; i ++)
        publish_bunch(i);
    PUBLISH_WF_READ_VAR(short, "DET:I:M", TUNE_LENGTH, wf_i_mean);
    PUBLISH_WF_READ_VAR(short, "DET:Q:M", TUNE_LENGTH, wf_q_mean);
    iq_trigger = create_interlock("DET:TRIG", "DET:DONE", false);

    PUBLISH_ACTION("DET:WBUNCH", write_det_bunches);

    tune_scale_pv = PUBLISH_WF_READ_VAR_I(
        float, "DET:SCALE", TUNE_LENGTH, tune_scale);

    PUBLISH_WRITER_P(ao,   "NCO:FREQ", write_nco_freq);
    PUBLISH_WRITER_P(mbbo, "NCO:GAIN", hw_write_nco_gain);

    /* Initialise the scaling constants so that
     *  wf_scaling * 2^wf_shift = 936 * 2^-33. */
    compute_scaling(MAX_BUNCH_COUNT, &wf_scaling, &wf_shift);
    wf_shift -= 33;     // Divide by 2^33.

    return true;
}
