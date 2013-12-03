/* Sequencer sweep and detector control. */

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
#include "detector.h"

#include "sequencer.h"


enum { HOM_GAIN_OFF = 8 };

/* These are the sequencer states for states 1 to 7.  State 0 is special and
 * is not handled in this array. */
struct sequencer_bank {
    double start_freq;
    double delta_freq;
    unsigned int dwell_time;
    unsigned int capture_count;
    unsigned int bunch_bank;
    unsigned int hom_gain;
    unsigned int holdoff;
    bool enable_window;
    bool write_enable;
    struct epics_record *delta_freq_rec;
    struct epics_record *end_freq_rec;
};

/* Sequencer state as currently seen through EPICS. */
static struct sequencer_bank banks[MAX_SEQUENCER_COUNT];
static unsigned int bank0;

/* Sequencer state as currently written to hardware, or as will be written at
 * start of next trigger. */
static struct seq_entry current_sequencer[MAX_SEQUENCER_COUNT];

/* Fast buffer interface. */
static struct epics_interlock *buffer_trigger;
static short buffer_low[RAW_BUF_DATA_LENGTH];
static short buffer_high[RAW_BUF_DATA_LENGTH];
static unsigned int buf_select;
static bool capture_iq;     // Set from buf_select when written to hardware

/* Sequencer program counter. */
static unsigned int sequencer_pc;

/* Number of IQ points to be captured by sequencer. */
static struct epics_interlock *info_trigger;
static int capture_count;       // Number of IQ points to capture
static int sequencer_duration;  // Total duration of sequence

static bool settings_changed;


static bool write_end_freq(void *context, const double *value)
{
    struct sequencer_bank *bank = context;
    bank->delta_freq = (*value - bank->start_freq) / bank->capture_count;
    WRITE_OUT_RECORD(ao, bank->delta_freq_rec, bank->delta_freq, false);
    return true;
}


static void publish_bank(int ix, struct sequencer_bank *bank)
{
    char buffer[20];
#define FORMAT(name) \
    (sprintf(buffer, "SEQ:%d:%s", ix + 1, name), buffer)

    PUBLISH_WRITE_VAR_P(ao, FORMAT("START_FREQ"), bank->start_freq);
    bank->delta_freq_rec =
        PUBLISH_WRITE_VAR_P(ao, FORMAT("STEP_FREQ"), bank->delta_freq);
    PUBLISH_WRITE_VAR_P(ulongout, FORMAT("DWELL"), bank->dwell_time);
    PUBLISH_WRITE_VAR_P(ulongout, FORMAT("COUNT"), bank->capture_count);
    PUBLISH_WRITE_VAR_P(mbbo, FORMAT("BANK"), bank->bunch_bank);
    PUBLISH_WRITE_VAR_P(mbbo, FORMAT("GAIN"), bank->hom_gain);
    PUBLISH_WRITE_VAR_P(bo, FORMAT("ENWIN"), bank->enable_window);
    PUBLISH_WRITE_VAR_P(bo, FORMAT("CAPTURE"), bank->write_enable);
    PUBLISH_WRITE_VAR_P(ulongout, FORMAT("HOLDOFF"), bank->holdoff);

    bank->end_freq_rec =
        PUBLISH(ao, FORMAT("END_FREQ"), write_end_freq, .context = bank);
#undef FORMAT
}


/* Compute number of captured points, to be called when the sequencer state or
 * program counter changes. */
static void update_capture_count(void)
{
    interlock_wait(info_trigger);
    capture_count = 0;
    sequencer_duration = 0;
    for (unsigned int i = 1; i <= sequencer_pc; i ++)
    {
        struct sequencer_bank *bank = &banks[i - 1];
        if (bank->write_enable)
            capture_count += bank->capture_count;
        sequencer_duration +=
            bank->capture_count * (bank->dwell_time + bank->holdoff);
    }
    interlock_signal(info_trigger, NULL);

    /* The sequencer will need to know about this in due course. */
    settings_changed = true;
}


/* Converts internal sequencer state into format suitable for writing to
 * hardware and updates hardware. */
static void write_seq_state(void)
{
    for (int i = 0; i < MAX_SEQUENCER_COUNT; i ++)
    {
        const struct sequencer_bank *bank = &banks[i];
        struct seq_entry *entry = &current_sequencer[i];
        entry->start_freq = tune_to_freq(bank->start_freq);
        entry->delta_freq = tune_to_freq(bank->delta_freq);
        entry->dwell_time = bank->dwell_time;
        entry->capture_count = bank->capture_count;
        entry->bunch_bank = bank->bunch_bank;
        entry->hom_gain = bank->hom_gain;
        entry->hom_enable = bank->hom_gain < HOM_GAIN_OFF;
        entry->enable_window = bank->enable_window;
        entry->write_enable = bank->write_enable;
        entry->holdoff = bank->holdoff;
        entry->window_rate =
            (unsigned int) lround((pow(2, 32) / 234) / bank->dwell_time);
    }
    hw_write_seq_entries(bank0, current_sequencer);
}


static void update_seq_state(void)
{
    /* Update all the end frequencies. */
    for (int i = 0; i < MAX_SEQUENCER_COUNT; i ++)
    {
        struct sequencer_bank *bank = &banks[i];
        double end_freq =
            bank->start_freq + bank->capture_count * bank->delta_freq;
        WRITE_OUT_RECORD(ao, bank->end_freq_rec, end_freq, false);
    }

    /* Update overall information about sequence. */
    update_capture_count();
}


/* Called before arming the sequencer: now is the time to configure the hardware
 * for operation. */
void prepare_sequencer(bool enable_sequencer)
{
    capture_iq = buf_select == SELECT_IQ  &&  capture_count > 0;
    hw_write_buf_select(buf_select);
    hw_write_seq_count(enable_sequencer ? sequencer_pc : 0);
    if (enable_sequencer)
    {
        write_seq_state();
        prepare_detector(settings_changed);
        settings_changed = false;
    }
}


static void set_state0_bunch_bank(unsigned int bank)
{
    bank0 = bank;
    hw_write_seq_entries(bank0, current_sequencer);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* This will be called when the fast buffer is triggered. */
void process_fast_buffer(void)
{
    interlock_wait(buffer_trigger);
    hw_read_buf_data(buffer_low, buffer_high);
    interlock_signal(buffer_trigger, NULL);

    if (capture_iq)
        update_iq(buffer_low, buffer_high, sequencer_pc, current_sequencer);
}


static void write_seq_count(unsigned int count)
{
    sequencer_pc = count;
    update_capture_count();
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


bool initialise_sequencer(void)
{
    PUBLISH_WRITER_P(mbbo, "SEQ:0:BANK", set_state0_bunch_bank);

    for (int i = 0; i < MAX_SEQUENCER_COUNT; i ++)
        publish_bank(i, &banks[i]);
    PUBLISH_ACTION("SEQ:WRITE", update_seq_state);
    PUBLISH_WRITER_P(ulongout, "SEQ:PC", write_seq_count);
    PUBLISH_READER(ulongin, "SEQ:PC", hw_read_seq_state);
    PUBLISH_ACTION("SEQ:RESET", hw_write_seq_reset);

    info_trigger = create_interlock("SEQ:INFO:TRIG", "SEQ:INFO:DONE", false);
    PUBLISH_READ_VAR(longin, "SEQ:LENGTH", capture_count);
    PUBLISH_READ_VAR(longin, "SEQ:DURATION", sequencer_duration);

    /* Fast buffer configuration settings. */
    buffer_trigger = create_interlock("BUF:TRIG", "BUF:DONE", false);
    PUBLISH_WF_READ_VAR(short, "BUF:WFA", BUF_DATA_LENGTH, buffer_low);
    PUBLISH_WF_READ_VAR(short, "BUF:WFB", BUF_DATA_LENGTH, buffer_high);
    PUBLISH_WRITE_VAR_P(mbbo, "BUF:SELECT", buf_select);

    return true;
}
