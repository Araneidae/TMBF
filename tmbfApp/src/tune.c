/* More advanced tune handling.  This processes the IQ waveforms after the
 * detector module has handled them.  This separation is so that more complex
 * and experimental tune detection can be separated from the core code. */

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <string.h>
#include <complex.h>
#include <math.h>

#include "error.h"
#include "epics_device.h"
#include "epics_extra.h"
#include "hardware.h"
#include "detector.h"
#include "sequencer.h"
#include "tune_support.h"
#include "tune_peaks.h"

#include "tune.h"


/* Slightly digested detector sweep information needed for tune measurement. */
struct tune_sweep_info {
    unsigned int sweep_length;
    double *tune_scale;
    struct channel_sweep *sweep;
};


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Tune measurement. */

static double threshold_fraction = 0.3;
static unsigned int min_block_separation = 20;
static unsigned int min_block_length = 20;


/* Searches given waveform for blocks meeting the peak detection criteria,
 * returns number of blocks found.  Only MAX_BLOCKS blocks will be searched for,
 * returns MAX_BLOCKS if at least this many blocks found. */
#define MAX_BLOCKS  3
struct block { unsigned int start; unsigned int end; };
static unsigned int find_blocks(
    unsigned int length, const int wf[], int threshold,
    struct block blocks[MAX_BLOCKS])
{
    unsigned int count = 0;

    struct block dummy = { 0, 0 };
    struct block *last_block = &dummy;
    unsigned int ix = 0;
    while (count < MAX_BLOCKS)
    {
        struct block *block = &blocks[count];

        /* Search for start of block. */
        while (ix < length  &&  wf[ix] < threshold)
            ix += 1;
        if (ix >= length)  break;
        block->start = ix;

        /* Search for end of block. */
        while (ix < length  &&  wf[ix] >= threshold)
            ix += 1;
        block->end = ix;

        /* Assess what we've found.  Block must be long enough and sufficiently
         * separated from the previous block. */
        bool sep_ok = block->start - last_block->end >= min_block_separation;
        bool len_ok = block->end - block->start >= min_block_length;
        if (sep_ok  &&  len_ok)
        {
            last_block = block;
            count += 1;
        }
    }

    return count;
}


static enum tune_status find_peak(
    const int wf[], struct block *block, double *peak_ix)
{
    double result;
    if (fit_quadratic(block->end - block->start, wf + block->start, &result))
    {
        *peak_ix = block->start + result;
        return TUNE_OK;
    }
    else
        return TUNE_BAD_FIT;
}


static unsigned int find_high_ix(
    const struct block *blocks, unsigned int count, const double tune_scale[])
{
    unsigned int low_ix  = (blocks[0].start + blocks[0].end) / 2;
    unsigned int high_ix = (blocks[count-1].start + blocks[count-1].end) / 2;
    if (tune_scale[low_ix] < tune_scale[high_ix])
        return count - 1;
    else
        return 0;
}

static void measure_tune_basic(
    unsigned int length, const struct channel_sweep *sweep,
    const double tune_scale[],
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
    int peak_val = find_max_val(length, sweep->power);
    int threshold = (int) (threshold_fraction * peak_val);
    struct block blocks[MAX_BLOCKS];
    unsigned int block_count =
        find_blocks(length, sweep->power, threshold, blocks);

    double tune_ix = 0;
    switch (block_count)
    {
        case 0:
            *tune_status = TUNE_NO_PEAK;
            break;
        case MAX_BLOCKS:
            *tune_status = TUNE_EXTRA_PEAKS;
            break;
        case 1:
            *tune_status = find_peak(sweep->power, &blocks[0], &tune_ix);
            break;
        case 2:
        {
            // Take the peak with the higher frequency
            unsigned int block_ix = find_high_ix(blocks, 2, tune_scale);
            *tune_status = find_peak(sweep->power, &blocks[block_ix], &tune_ix);
            break;
        }
    }

    if (*tune_status == TUNE_OK)
        index_to_tune(
            tune_scale, sweep->wf_i, sweep->wf_q, tune_ix, tune, phase);
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Generic tune results. */

/* As these two values are used to compute the status of each tune update we
 * place them here. */
static double centre_tune;      // Centre of tune sweep
static double half_range;       // Min/max tune setting
static double alarm_range;      // Alarm range to test


struct tune_result_value {
    unsigned int status;
    double tune;
    double phase;
    epicsAlarmSeverity severity;
};

struct tune_result {
    struct tune_result_value value;
    struct epics_record *tune_pv;
    struct epics_record *phase_pv;
};


static void set_tune_result(
    struct tune_result_value *result,
    unsigned int status, double tune, double phase)
{
    result->status = status;
    if (status == TUNE_OVERFLOW)
        result->severity = epicsSevMajor;
    else
    {
        result->tune = tune;
        result->phase = phase;
        if (status == TUNE_OK)
        {
            /* Check for tune within alarm range. */
            if (fabs(tune - centre_tune) >= alarm_range)
            {
                result->status = TUNE_RANGE;
                result->severity = epicsSevMinor;
            }
            else
                result->severity = epicsSevNone;
        }
        else
            result->severity = epicsSevInvalid;
    }
}


/* Updates tune results with result of calling given tune measurement function
 * on the given tune sweep data.  Nothing is called if detector measurement
 * overflowed. */
static void compute_tune_result(
    bool overflow,
    unsigned int length, const struct channel_sweep *sweep,
    const double tune_scale[], struct tune_result *result,
    void (*measure_tune)(
        unsigned int length, const struct channel_sweep *sweep,
        const double tune_scale[],
        unsigned int *tune_status, double *tune, double *phase))
{
    if (overflow)
        set_tune_result(&result->value, TUNE_OVERFLOW, 0, 0);
    else
    {
        unsigned int status;
        double phase, tune;
        measure_tune(length, sweep, tune_scale, &status, &tune, &phase);
        set_tune_result(&result->value, status, tune, phase);
    }

    trigger_record(result->tune_pv,  result->value.severity, NULL);
    trigger_record(result->phase_pv, result->value.severity, NULL);
}


/* Copy core tune results from selected source to destination including
 * associated record severity. */
static void copy_tune_result(
    struct tune_result_value *src, struct tune_result *dest)
{
    dest->value = *src;
    trigger_record(dest->tune_pv,  dest->value.severity, NULL);
    trigger_record(dest->phase_pv, dest->value.severity, NULL);
}


static void publish_tune_result(struct tune_result *result, const char *prefix)
{
    char buffer[40];
#define FORMAT(field) (sprintf(buffer, "%s:%s", prefix, field), buffer)
    PUBLISH_READ_VAR(mbbi, FORMAT("STATUS"), result->value.status);
    result->tune_pv  =
        PUBLISH_READ_VAR(ai, FORMAT("TUNE"), result->value.tune);
    result->phase_pv =
        PUBLISH_READ_VAR(ai, FORMAT("PHASE"), result->value.phase);
#undef FORMAT
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Control. */

/* Trigger interlock for record update. */
static struct epics_interlock *tune_trigger;

/* Tune settings. */
static int harmonic;            // Base frequency for tune sweep
static bool reverse_tune;       // Set to sweep tune backwards
static unsigned int selected_bunch; // Selected single bunch

/* Waveforms from last detector sweep. */
static struct channel_sweep sweep;
static float phase_waveform[TUNE_LENGTH];
static int cumsum_i[TUNE_LENGTH];
static int cumsum_q[TUNE_LENGTH];
static double mean_power;
static int max_power;

/* Tune measurements. */
static struct tune_result tune_result_selected;
static struct tune_result tune_result_basic;
static struct tune_result tune_result_peaks;
static struct tune_result_value tune_result_pll;
enum { SELECT_TUNE_BASIC, SELECT_TUNE_PEAKS, SELECT_TUNE_PLL };
static unsigned int selected_tune_result;
static struct epics_interlock *tune_result;


/* Extracts tune sweep info from detector sweep info taking our channel selectio
 * into account. */
static void extract_sweep_info(
    struct tune_sweep_info *tune_sweep,
    struct sweep_info *sweep_info)
{
    tune_sweep->sweep_length = sweep_info->sweep_length;
    tune_sweep->tune_scale = sweep_info->tune_scale;
    tune_sweep->sweep = sweep_info->single_bunch_mode ?
        &sweep_info->channels[selected_bunch % 4] : &sweep_info->mean;
}


static void update_iq_power(const struct tune_sweep_info *tune_sweep)
{
    /* Take copy of selected sweep so we can publish selection specific PVs for
     * I, Q and power. */
    memcpy(&sweep, tune_sweep->sweep, sizeof(sweep));

    /* Update the total and max power statistics. */
    double total_power = 0;
    max_power = 0;
    for (unsigned int i = 0; i < tune_sweep->sweep_length; i ++)
    {
        int power = tune_sweep->sweep->power[i];
        total_power += power;
        if (power > max_power)
            max_power = power;
    }
    mean_power = total_power / tune_sweep->sweep_length;
}


static void update_phase_wf(void)
{
    for (int i = 0; i < TUNE_LENGTH; i ++)
        phase_waveform[i] =
            180.0F / (float) M_PI * atan2f(sweep.wf_q[i], sweep.wf_i[i]);
}


static void update_cumsum(const struct tune_sweep_info *tune_sweep)
{
    int sum_i = 0, sum_q = 0;
    for (unsigned int i = 0; i < tune_sweep->sweep_length; i ++)
    {
        sum_i += sweep.wf_i[i];
        sum_q += sweep.wf_q[i];
        cumsum_i[i] = sum_i;
        cumsum_q[i] = sum_q;
    }

    /* Pad the rest of the waveform with repeats of the last point. */
    for (unsigned int i = tune_sweep->sweep_length; i < TUNE_LENGTH; i ++)
    {
        cumsum_i[i] = sum_i;
        cumsum_q[i] = sum_q;
    }
}


static struct tune_result_value *select_tune_result(void)
{
    switch (selected_tune_result)
    {
        default:
        case SELECT_TUNE_BASIC: return &tune_result_basic.value;
        case SELECT_TUNE_PEAKS: return &tune_result_peaks.value;
        case SELECT_TUNE_PLL:   return &tune_result_pll;
    }
}

static void update_tune_result(void)
{
    interlock_wait(tune_result);
    copy_tune_result(select_tune_result(), &tune_result_selected);
    interlock_signal(tune_result, NULL);
}


static void do_tune_sweep(
    const struct tune_sweep_info *tune_sweep, bool overflow)
{
    interlock_wait(tune_trigger);

    update_iq_power(tune_sweep);
    update_phase_wf();
    update_cumsum(tune_sweep);

    compute_tune_result(
        overflow,
        tune_sweep->sweep_length, &sweep, tune_sweep->tune_scale,
        &tune_result_basic, measure_tune_basic);

//     interlock_signal(tune_trigger, NULL);

    /* Warning: At present we have nested EPICS interlock updates.  This is
     * necessary because the PEAK tune result is updated on the tune_trigger
     * interlock, but we need to complete the processing of measure_tune_peaks
     * first.  A bit of gentle refactoring is in order to avoid this. */
    compute_tune_result(
        overflow,
        tune_sweep->sweep_length, &sweep, tune_sweep->tune_scale,
        &tune_result_peaks, measure_tune_peaks);

    interlock_signal(tune_trigger, NULL);

    if (selected_tune_result != SELECT_TUNE_PLL)
        update_tune_result();
}


/* Called every time the tune PLL code has a new tune.  If the tune result is
 * configured to use tune PLL then we use this result. */
void update_tune_pll_tune(bool tune_ok, double tune, double phase)
{
    set_tune_result(
        &tune_result_pll, tune_ok ? TUNE_OK : TUNE_INVALID, tune, phase);

    /* Ignore repeated announcements that we have no results. */
    bool repeat = !tune_ok  &&  tune_result_pll.status != TUNE_OK;
    if (selected_tune_result == SELECT_TUNE_PLL  &&  !repeat)
        update_tune_result();
}


static void set_selected_result(unsigned int selection)
{
    selected_tune_result = selection;
    if (check_epics_ready())
        update_tune_result();
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Test data injection support. */

/* Fake data for test injection. */
static struct tune_sweep_info injection_info = {
    .sweep_length = TUNE_LENGTH,
    .tune_scale = (double [TUNE_LENGTH]) {},
    .sweep = &(struct channel_sweep) {}
};


void update_tune_sweep(struct sweep_info *sweep_info, bool overflow)
{
    struct tune_sweep_info tune_sweep;
    extract_sweep_info(&tune_sweep, sweep_info);
    do_tune_sweep(&tune_sweep, overflow);

    /* After performing a normal tune sweep update the injected sweep tune scale
     * so that things match by default. */
    memcpy(injection_info.tune_scale, tune_sweep.tune_scale,
        sizeof(double) * TUNE_LENGTH);
}


/* Inject given test data as power sweep by forcing call to update_tune_sweep
 * with synthetic sweep info structure. */
static void inject_test_data(void)
{
    inject_tune_scale(injection_info.tune_scale);
    compute_power(injection_info.sweep);
    do_tune_sweep(&injection_info, false);
}


static void publish_inject_pvs(void)
{
    PUBLISH_WF_WRITE_VAR(short, "TUNE:INJECT:I", TUNE_LENGTH,
        injection_info.sweep->wf_i);
    PUBLISH_WF_WRITE_VAR(short, "TUNE:INJECT:Q", TUNE_LENGTH,
        injection_info.sweep->wf_q);
    PUBLISH_WF_WRITE_VAR(double, "TUNE:INJECT:S", TUNE_LENGTH,
        injection_info.tune_scale);
    PUBLISH_ACTION("TUNE:INJECT", inject_test_data);

    /* Create sensible initial value for tune scale. */
    for (int i = 0; i < TUNE_LENGTH; i ++)
        injection_info.tune_scale[i] = (double) i / TUNE_LENGTH;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


static struct in_epics_record_bi *tune_setting;

static void tune_setting_changed(void)
{
    WRITE_IN_RECORD(bi, tune_setting, false);
}


static void set_bunch_control(void)
{
    /* Copy FIR and gain settings from bank 0 to bank 1. */
    char fir_wf[BUNCHES_PER_TURN];
    float gain_wf[BUNCHES_PER_TURN];
    READ_NAMED_RECORD_WF(char,  "BUN:0:FIRWF_S", fir_wf, BUNCHES_PER_TURN);
    READ_NAMED_RECORD_WF(float, "BUN:0:GAINWF_S", gain_wf, BUNCHES_PER_TURN);
    WRITE_NAMED_RECORD_WF(char,  "BUN:1:FIRWF_S", fir_wf, BUNCHES_PER_TURN);
    WRITE_NAMED_RECORD_WF(float, "BUN:1:GAINWF_S", gain_wf, BUNCHES_PER_TURN);

    bool single_bunch_mode = READ_NAMED_RECORD(bo, "DET:MODE");

    char out_wf[BUNCHES_PER_TURN];
    if (single_bunch_mode)
    {
        READ_NAMED_RECORD_WF(char, "BUN:0:OUTWF_S", out_wf, BUNCHES_PER_TURN);
        out_wf[selected_bunch] = 4;
    }
    else
        memset(out_wf, 4, BUNCHES_PER_TURN);
    WRITE_NAMED_RECORD_WF(char, "BUN:1:OUTWF_S", out_wf, BUNCHES_PER_TURN);
}


/* Clips frequency to range 0..BUNCHES_PER_TURN (maximum possible mode). */
static double clip_freq_range(double frequency)
{
    if (frequency < 0)
        return 0;
    else if (frequency > BUNCHES_PER_TURN)
        return BUNCHES_PER_TURN;
    else
        return frequency;
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

    /* Force buffer to IQ and configure triggering into standard mode. */
    WRITE_NAMED_RECORD(mbbo, "BUF:SELECT", BUF_SELECT_IQ);
    WRITE_NAMED_RECORD(mbbo, "TRG:SEQ:SEL", SEQ_TRIG_BUF);
    WRITE_NAMED_RECORD(bo, "TRG:BUF:SEL", true);    // Hardware trigger
    WRITE_NAMED_RECORD(bo, "TRG:BUF:MODE", true);   // Automatic retrigger

    /* Configure triggering on external trigger only. */
    WRITE_NAMED_RECORD(bo, "TRG:BUF:EXT:EN", true);
    WRITE_NAMED_RECORD(bo, "TRG:BUF:ADC:EN", false);
    WRITE_NAMED_RECORD(bo, "TRG:BUF:SCLK:EN", false);

    /* Configure the sequencer with the selected tune range.  Force count and
     * capture to sensible values and set the sequencer PC to 1. */
    double centre = harmonic + centre_tune;
    WRITE_NAMED_RECORD(ulongout, "SEQ:1:COUNT", 4096);
    WRITE_NAMED_RECORD(ao, "SEQ:1:START_FREQ", clip_freq_range(
        centre + (reverse_tune ? + half_range : - half_range)));
    WRITE_NAMED_RECORD(ao, "SEQ:1:END_FREQ", clip_freq_range(
        centre + (reverse_tune ? - half_range : + half_range)));
    WRITE_NAMED_RECORD(bo,       "SEQ:1:CAPTURE", true);
    WRITE_NAMED_RECORD(mbbo,     "SEQ:1:BANK", 1);
    WRITE_NAMED_RECORD(ulongout, "SEQ:PC", 1);

    /* Effectively disable the super sequencer by setting it to 1 and resetting
     * the offsets waveform. */
    WRITE_NAMED_RECORD(ulongout, "SEQ:SUPER:COUNT", 0);
    WRITE_NAMED_RECORD(bo, "SEQ:SUPER:RESET", true);

    /* Configure the bunch control as a copy of bank 0, but with sweep enabled
     * for output. */
    set_bunch_control();

    /* Let the user know that the settings are now valid. */
    WRITE_IN_RECORD(bi, tune_setting, true);
}


bool initialise_tune(void)
{
    tune_trigger = create_interlock("TUNE", false);
    PUBLISH_WRITE_VAR_P(longout, "TUNE:HARMONIC", harmonic);
    PUBLISH_WRITE_VAR_P(ao, "TUNE:CENTRE", centre_tune);
    PUBLISH_WRITE_VAR_P(ao, "TUNE:RANGE", half_range);
    PUBLISH_WRITE_VAR_P(bo, "TUNE:DIRECTION", reverse_tune);
    PUBLISH_WRITE_VAR_P(ao, "TUNE:ALARM", alarm_range);
    PUBLISH_WRITE_VAR_P(ulongout, "TUNE:BUNCH", selected_bunch);

    PUBLISH_WF_READ_VAR(short, "TUNE:I", TUNE_LENGTH, sweep.wf_i);
    PUBLISH_WF_READ_VAR(short, "TUNE:Q", TUNE_LENGTH, sweep.wf_q);
    PUBLISH_WF_READ_VAR(int, "TUNE:POWER", TUNE_LENGTH, sweep.power);
    PUBLISH_WF_READ_VAR(float, "TUNE:PHASEWF", TUNE_LENGTH, phase_waveform);
    PUBLISH_READ_VAR(ai, "TUNE:MEANPOWER", mean_power);
    PUBLISH_READ_VAR(longin, "TUNE:MAXPOWER", max_power);
    PUBLISH_WF_READ_VAR(int, "TUNE:CUMSUMI", TUNE_LENGTH, cumsum_i);
    PUBLISH_WF_READ_VAR(int, "TUNE:CUMSUMQ", TUNE_LENGTH, cumsum_q);

    publish_tune_result(&tune_result_selected, "TUNE");
    publish_tune_result(&tune_result_basic, "TUNE:BASIC");
    publish_tune_result(&tune_result_peaks, "PEAK");
    PUBLISH_WRITER_P(mbbo, "TUNE:SELECT", set_selected_result);
    tune_result = create_interlock("TUNE:RESULT", false);

    PUBLISH_ACTION("TUNE:CHANGED", tune_setting_changed);
    tune_setting = PUBLISH_IN_VALUE_I(bi, "TUNE:SETTING");
    PUBLISH_ACTION("TUNE:SET", set_tune_settings);

    /* Control parameters for tune measurement algorithm. */
    PUBLISH_WRITE_VAR_P(ao, "TUNE:THRESHOLD", threshold_fraction);
    PUBLISH_WRITE_VAR_P(ulongout, "TUNE:BLK:SEP", min_block_separation);
    PUBLISH_WRITE_VAR_P(ulongout, "TUNE:BLK:LEN", min_block_length);

    publish_inject_pvs();

    return true;
}
