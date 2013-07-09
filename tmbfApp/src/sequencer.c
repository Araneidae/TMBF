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

#include "sequencer.h"


/* These are the sequencer states for states 1 to 7.  State 0 is special and
 * is not handled in this array. */
struct sequencer_bank {
    double start_freq;
    double delta_freq;
    unsigned int dwell_time;
    unsigned int capture_count;
    unsigned int bunch_bank;
    unsigned int hom_gain;
};

static struct sequencer_bank banks[MAX_SEQUENCER_COUNT - 1];
static unsigned int state0_bunch_bank;



static void publish_bank(int ix, struct sequencer_bank *bank)
{
    char buffer[20];
#define FORMAT(name) \
    (sprintf(buffer, "SEQ:%d:%s", ix + 1, name), buffer)

    PUBLISH_WRITE_VAR_P(ao, FORMAT("START_FREQ"), bank->start_freq);
    PUBLISH_WRITE_VAR_P(ao, FORMAT("STEP_FREQ"), bank->delta_freq);
    PUBLISH_WRITE_VAR_P(ulongout, FORMAT("DWELL"), bank->dwell_time);
    PUBLISH_WRITE_VAR_P(ulongout, FORMAT("COUNT"), bank->capture_count);
    PUBLISH_WRITE_VAR_P(mbbo, FORMAT("BANK"), bank->bunch_bank);
    PUBLISH_WRITE_VAR_P(mbbo, FORMAT("GAIN"), bank->hom_gain);

#undef FORMAT
}


static unsigned int tune_to_freq(double tune)   // !!! share this
{
    return (unsigned int) round(pow(2,33) * tune / 936.0);
}

static void write_seq_state(void)
{
    struct seq_entry entries[MAX_SEQUENCER_COUNT] = {
        [0] = { .bunch_bank = state0_bunch_bank }};
    for (int i = 1; i < MAX_SEQUENCER_COUNT; i ++)
    {
        struct sequencer_bank *bank = &banks[i - 1];
        struct seq_entry *entry = &entries[i];
        entry->start_freq = tune_to_freq(bank->start_freq);
        entry->delta_freq = tune_to_freq(bank->delta_freq);
        entry->dwell_time = bank->dwell_time - 1;
        entry->capture_count = bank->capture_count;
        entry->bunch_bank = bank->bunch_bank;
        entry->hom_gain = bank->hom_gain;
        entry->wait_time = 0;
    }
    hw_write_seq_entries(entries);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* Fast buffer interface. */

static struct epics_interlock *buffer_trigger;
static short buffer_low[BUF_DATA_LENGTH];
static short buffer_high[BUF_DATA_LENGTH];
static unsigned int buf_select;

/* Sequencer enable and program counter. */
static unsigned int sequencer_pc;
static bool sequencer_enable;


/* This will be called when the fast buffer is triggered. */
void process_fast_buffer(void)
{
    interlock_wait(buffer_trigger);
    hw_read_buf_data(buffer_low, buffer_high);
    interlock_signal(buffer_trigger, NULL);
}


/* We need to keep track of the buffer selection for sequencer completion
 * detection below. */
static void write_buf_select(unsigned int value)
{
    buf_select = value;
    hw_write_buf_select(buf_select);
}


/* If buf_select is IQ (numerical value 1) then we have to report the buffer
 * busy when the sequencer is busy, otherwise it's up to the buffer.
 *    Think this really belongs in the hardware...! */
bool poll_buf_busy(void)
{
    if (buf_select == 1)
    {
        if (sequencer_enable)
            return hw_read_seq_status();
        else
            return false;   // It's never going to trigger in fact...
    }
    else
        return hw_read_buf_status();
}


static void write_seq_count(unsigned int count)
{
    sequencer_pc = count;
    if (sequencer_enable)
        hw_write_seq_count(sequencer_pc);
}

void enable_seq_trigger(bool enable)
{
    sequencer_enable = enable;
    hw_write_seq_count(enable ? sequencer_pc : 0);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


bool initialise_sequencer(void)
{
    PUBLISH_WRITE_VAR_P(mbbo, "SEQ:0:BANK", state0_bunch_bank);
    for (int i = 0; i < MAX_SEQUENCER_COUNT; i ++)
        publish_bank(i, &banks[i]);
    PUBLISH_ACTION("SEQ:WRITE", write_seq_state);
    PUBLISH_WRITER_P(ulongout, "SEQ:PC", write_seq_count);
    PUBLISH_READER(ulongin, "SEQ:PC", hw_read_seq_state);
    PUBLISH_ACTION("SEQ:RESET", hw_write_seq_reset);

    PUBLISH_WRITER_P(bo, "DET:MODE", hw_write_det_mode);
    PUBLISH_WRITER_P(mbbo, "DET:GAIN", hw_write_det_gain);

    /* Fast buffer configuration settings. */
    buffer_trigger = create_interlock("BUF:TRIG", "BUF:DONE", false);
    PUBLISH_WF_READ_VAR(short, "BUF:WFA", BUF_DATA_LENGTH, buffer_low);
    PUBLISH_WF_READ_VAR(short, "BUF:WFB", BUF_DATA_LENGTH, buffer_high);
    PUBLISH_WRITER_P(mbbo, "BUF:SELECT", write_buf_select);

    return true;
}
