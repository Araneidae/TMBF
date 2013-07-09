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

#include "sequencer.h"


struct sequencer_bank {
    double start_freq;
    double delta_freq;
    unsigned int dwell_time;
    unsigned int capture_count;
    unsigned int bunch_bank;
    unsigned int hom_gain;
};


static void publish_bank(int ix, struct sequencer_bank *bank)
{
    char buffer[20];
#define FORMAT(name) \
    (sprintf(buffer, "SEQ:%d:%s", ix, name), buffer)

    PUBLISH_WRITE_VAR_P(ao, FORMAT("START_FREQ"), bank->start_freq);
    PUBLISH_WRITE_VAR_P(ao, FORMAT("STEP_FREQ"), bank->delta_freq);
    PUBLISH_WRITE_VAR_P(ulongout, FORMAT("DWELL"), bank->dwell_time);
    PUBLISH_WRITE_VAR_P(ulongout, FORMAT("COUNT"), bank->capture_count);
    PUBLISH_WRITE_VAR_P(mbbo, FORMAT("BANK"), bank->bunch_bank);
    PUBLISH_WRITE_VAR_P(mbbo, FORMAT("GAIN"), bank->hom_gain);

#undef FORMAT
}


static struct sequencer_bank banks[MAX_SEQUENCER_COUNT] = { };

static unsigned int tune_to_freq(double tune)   // !!! share this
{
    return (unsigned int) round(pow(2,33) * tune / 936.0);
}

static void write_seq_state(void)
{
printf("write_seq_state\n");
    struct seq_entry entries[MAX_SEQUENCER_COUNT];
    for (int i = 0; i < MAX_SEQUENCER_COUNT; i ++)
    {
        struct sequencer_bank *bank = &banks[i];
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


bool initialise_sequencer(void)
{
    for (int i = 0; i < MAX_SEQUENCER_COUNT; i ++)
        publish_bank(i, &banks[i]);
    PUBLISH_ACTION("SEQ:WRITE", write_seq_state);
    PUBLISH_WRITER(ulongout, "SEQ:PC", hw_write_seq_count);
    PUBLISH_READER(ulongin, "SEQ:PC", hw_read_seq_state);
    PUBLISH_ACTION("SEQ:RESET", hw_write_seq_reset);

    PUBLISH_WRITER_P(bo, "DET:MODE", hw_write_det_mode);
    PUBLISH_WRITER_P(mbbo, "DET:GAIN", hw_write_det_gain);
    return true;
}
