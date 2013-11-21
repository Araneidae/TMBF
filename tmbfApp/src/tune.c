/* More advanced tune handling.  This processes the IQ waveforms after the
 * detector module has handled them.  This separation is so that more complex
 * and experimental tune detection can be separated from the core code. */

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
#include "detector.h"
#include "sequencer.h"

#include "tune.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Tune measurement. */

/* Converts a tune, detected as an index into the tune sweep waveform, into the
 * corresponding tune frequency offset and phase. */
static void index_to_tune(
    const struct channel_sweep *sweep, const float *tune_scale,
    int ix, double *tune, double *phase)
{
    double harmonic;
    *tune = modf(tune_scale[ix], &harmonic);
    *phase = 180.0 / M_PI * atan2(sweep->wf_q[ix], sweep->wf_i[ix]);
}


enum tune_status {
    TUNE_INVALID,
    TUNE_OK
};


/* Returns index of maximum element of array. */
static int find_max(int length, const int *array)
{
    int max_val = array[0];
    int max_ix = 0;
    for (int i = 1; i < length; i ++)
        if (array[i] > max_val)
        {
            max_val = array[i];
            max_ix = i;
        }
    return max_ix;
}


static epicsAlarmSeverity measure_tune(
    int length, const struct channel_sweep *sweep,
    const float *tune_scale, bool overflow,
    unsigned int *tune_status, double *tune, double *phase)
{
    /* Very crude algorithm:
     *  1.  Find peak
     *  2.  Slice data to 1/3 of peak height and look for contiguous and well
     *      separated blocks above this threshold.
     *  3.  If one block found, use it, if two blocks found take second peak.
     * Refinements include:
     *  -   Median filter on incoming data
     *  -   Quadratic fit for peak detect
     *  -   Knockout waveform for spike removal. */
    int peak_ix = find_max(length, sweep->power);

    index_to_tune(sweep, tune_scale, peak_ix, tune, phase);
    *tune_status = TUNE_INVALID;
    return epicsSevInvalid;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Control. */

/* Trigger interlock for record update. */
static struct epics_interlock *tune_trigger;

/* Tune settings. */
static int harmonic;            // Base frequency for tune sweep
static double centre_tune;      // Centre of tune sweep
static double half_range;       // Min/max tune setting
static int selected_bunch;        // Selected single bunch

/* Waveforms from last detector sweep. */
static struct channel_sweep sweep;

/* Tune measurements. */
static double measured_tune;    // Tune measurement
static double measured_phase;   // and associated phase
static struct epics_record *measured_tune_rec;
static struct epics_record *measured_phase_rec;
static unsigned int tune_status;


static void update_iq_power(const struct sweep_info *sweep_info)
{
    const struct channel_sweep *channel = sweep_info->single_bunch_mode ?
        &sweep_info->channels[selected_bunch % 4] :
        &sweep_info->mean;
    memcpy(&sweep, channel, sizeof(sweep));
}


void update_tune_sweep(const struct sweep_info *sweep_info, bool overflow)
{
    interlock_wait(tune_trigger);
    update_iq_power(sweep_info);
    epicsAlarmSeverity severity = measure_tune(
        sweep_info->sweep_length, &sweep, sweep_info->tune_scale, overflow,
        &tune_status, &measured_tune, &measured_phase);
    trigger_record(measured_tune_rec, severity, NULL);
    trigger_record(measured_phase_rec, severity, NULL);
    interlock_signal(tune_trigger, NULL);
}


static bool tune_setting;
static struct epics_record *tune_setting_rec;

static void set_tune_setting(bool setting)
{
    if (tune_setting != setting)
    {
        tune_setting = setting;
        trigger_record(tune_setting_rec, 0, NULL);
    }
}

void tune_setting_changed(void)
{
    set_tune_setting(false);
}


static void set_bunch_control(void)
{
    /* Configure bank 1 for waveform operation. */
    WRITE_NAMED_RECORD(bo, "BUN:1:USEWF", true);

    /* Copy FIR and gain settings from bank 0 to bank 1. */
    char fir_wf[BUNCHES_PER_TURN];
    int gain_wf[BUNCHES_PER_TURN];
    READ_NAMED_RECORD_WF(char, "BUN:0:FIRWF", fir_wf, BUNCHES_PER_TURN);
    READ_NAMED_RECORD_WF(int,  "BUN:0:GAINWF", gain_wf, BUNCHES_PER_TURN);
    WRITE_NAMED_RECORD_WF(char, "BUN:1:FIRWF_S", fir_wf, BUNCHES_PER_TURN);
    WRITE_NAMED_RECORD_WF(int,  "BUN:1:GAINWF_S", gain_wf, BUNCHES_PER_TURN);

    bool single_bunch_mode = READ_NAMED_RECORD_VALUE(bo, "DET:MODE");

    char out_wf[BUNCHES_PER_TURN];
    if (single_bunch_mode)
    {
        READ_NAMED_RECORD_WF(char, "BUN:0:OUTWF", out_wf, BUNCHES_PER_TURN);
        out_wf[selected_bunch] = 4;
    }
    else
        memset(out_wf, 4, BUNCHES_PER_TURN);
    WRITE_NAMED_RECORD_WF(char, "BUN:1:OUTWF_S", out_wf, BUNCHES_PER_TURN);
}


/* When the user wishes to use the tune settings this function will force the
 * sequencer and detector to use the settings configured here. */
static void set_tune_settings(void)
{
    /* Configure appropriate channel for selected single bunch.  Do this whether
     * we're currently in single or multi bunch mode. */
    char bunch_channel[20];
    sprintf(bunch_channel, "DET:BUNCH%d", selected_bunch % 4);
    WRITE_NAMED_RECORD(ulongout, bunch_channel, selected_bunch / 4);

    /* Force buffer to IQ and ensure the buffer is enabled. */
    WRITE_NAMED_RECORD(mbbo, "BUF:SELECT", SELECT_IQ);
    WRITE_NAMED_RECORD(bo,   "TRG:SEQ:ENA", true);

    /* Configure the sequencer with the selected tune range.  Force count and
     * capture to sensible values and set the sequencer PC to 1. */
    double centre = harmonic + centre_tune;
    WRITE_NAMED_RECORD(ao,       "SEQ:1:START_FREQ", centre - half_range);
    WRITE_NAMED_RECORD(ao,       "SEQ:1:END_FREQ", centre + half_range);
    WRITE_NAMED_RECORD(bo,       "SEQ:1:CAPTURE", true);
    WRITE_NAMED_RECORD(mbbo,     "SEQ:1:BANK", 1);
    WRITE_NAMED_RECORD(ulongout, "SEQ:1:COUNT", 4096);
    WRITE_NAMED_RECORD(ulongout, "SEQ:PC", 1);

    /* Configure the bunch control as a copy of bank 0, but with sweep enabled
     * for output. */
    set_bunch_control();

    /* Let the user know that the settings are now valid. */
    set_tune_setting(true);
}


bool initialise_tune(void)
{
    tune_trigger = create_interlock("TUNE:TRIG", "TUNE:DONE", false);
    PUBLISH_WRITE_VAR_P(longout, "TUNE:HARMONIC", harmonic);
    PUBLISH_WRITE_VAR_P(ao, "TUNE:CENTRE", centre_tune);
    PUBLISH_WRITE_VAR_P(ao, "TUNE:RANGE", half_range);
    PUBLISH_WRITE_VAR_P(longout, "TUNE:BUNCH", selected_bunch);

    PUBLISH_WF_READ_VAR(short, "TUNE:I", TUNE_LENGTH, sweep.wf_i);
    PUBLISH_WF_READ_VAR(short, "TUNE:Q", TUNE_LENGTH, sweep.wf_q);
    PUBLISH_WF_READ_VAR(int, "TUNE:POWER", TUNE_LENGTH, sweep.power);

    PUBLISH_READ_VAR(mbbi, "TUNE:STATUS", tune_status);
    measured_tune_rec  = PUBLISH_READ_VAR(ai, "TUNE:TUNE",  measured_tune);
    measured_phase_rec = PUBLISH_READ_VAR(ai, "TUNE:PHASE", measured_phase);

    PUBLISH_ACTION("TUNE:CHANGED", tune_setting_changed);
    tune_setting_rec = PUBLISH_READ_VAR_I(bi, "TUNE:SETTING", tune_setting);
    PUBLISH_ACTION("TUNE:SET", set_tune_settings);

    trigger_record(tune_setting_rec, 0, NULL);
    return true;
}
