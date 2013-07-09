/* Bunch selection control. */

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

#include "bunch_select.h"


struct bunch_bank {
    int index;
    bool use_waveform;
    unsigned int fir_select;
    unsigned int out_select;
    int gain_select;
    char current_fir_wf[SAMPLES_PER_TURN];
    char current_out_wf[SAMPLES_PER_TURN];
    int current_gain_wf[SAMPLES_PER_TURN];
    char set_fir_wf[SAMPLES_PER_TURN];
    char set_out_wf[SAMPLES_PER_TURN];
    int set_gain_wf[SAMPLES_PER_TURN];
    struct epics_record *fir_wf_pv;
    struct epics_record *out_wf_pv;
    struct epics_record *gain_wf_pv;
};


/* Called when the current waveforms are up to date.  Write the current
 * settings out to the FPGA. */
static void update_bunch_select(struct bunch_bank *bank)
{
    /* Reshape our waveforms into format suitable for hardware. */
    struct bunch_entry entries[SAMPLES_PER_TURN];
    for (int i = 0; i < SAMPLES_PER_TURN; i++)
    {
        entries[i].bunch_gain    = bank->current_gain_wf[i];
        entries[i].output_select = bank->current_out_wf[i];
        entries[i].fir_select    = bank->current_fir_wf[i];
    }

    hw_write_bun_entry(bank->index, entries);
    trigger_record(bank->fir_wf_pv, 0, NULL);
    trigger_record(bank->out_wf_pv, 0, NULL);
    trigger_record(bank->gain_wf_pv, 0, NULL);
}


static bool reload_settings(void *context, const bool *value)
{
    struct bunch_bank *bank = context;
    if (!bank->use_waveform)
    {
#define MAKE_WF(name, type) \
    for (int i = 0; i < SAMPLES_PER_TURN; i ++) \
        bank->current_##name##_wf[i] = (type) bank->name##_select
        MAKE_WF(fir, char);
        MAKE_WF(out, char);
        MAKE_WF(gain, int);
#undef MAKE_WF
        update_bunch_select(bank);
    }
    return true;
}


static bool reload_waveforms(void *context, const bool *value)
{
    struct bunch_bank *bank = context;
    if (bank->use_waveform)
    {
#define COPY_WF(name) \
    memcpy(bank->current_##name##_wf, bank->set_##name##_wf, \
        sizeof(bank->current_##name##_wf))
        COPY_WF(fir);
        COPY_WF(out);
        COPY_WF(gain);
#undef COPY_WF
        update_bunch_select(bank);
    }
    return true;
}


static bool set_use_waveform(void *context, const bool *use_waveform)
{
    struct bunch_bank *bank = context;
    bank->use_waveform = *use_waveform;
    if (bank->use_waveform)
        reload_waveforms(bank, use_waveform);
    else
        reload_settings(bank, use_waveform);
    return true;
}


static void publish_bank(int ix, struct bunch_bank *bank)
{
    bank->index = ix;

    char buffer[20];
#define FORMAT(name) \
    (sprintf(buffer, "BUN:%d:%s", ix, name), buffer)

    PUBLISH(bo, FORMAT("RELOAD"), reload_settings, .context = bank);
    PUBLISH(bo, FORMAT("RELOADWF"), reload_waveforms, .context = bank);
    PUBLISH(bo, FORMAT("USEWF"),
        set_use_waveform, .context = bank, .persist = true);

    PUBLISH_WRITE_VAR_P(mbbo, FORMAT("FIR"), bank->fir_select);
    PUBLISH_WRITE_VAR_P(mbbo, FORMAT("OUT"), bank->out_select);
    PUBLISH_WRITE_VAR_P(longout, FORMAT("GAIN"), bank->gain_select);

    PUBLISH_WF_WRITE_VAR_P(
        char, FORMAT("FIRWF_S"), SAMPLES_PER_TURN, bank->set_fir_wf);
    PUBLISH_WF_WRITE_VAR_P(
        char, FORMAT("OUTWF_S"), SAMPLES_PER_TURN, bank->set_out_wf);
    PUBLISH_WF_WRITE_VAR_P(
        int, FORMAT("GAINWF_S"), SAMPLES_PER_TURN, bank->set_gain_wf);
    bank->fir_wf_pv = PUBLISH_WF_READ_VAR_I(
        char, FORMAT("FIRWF"), SAMPLES_PER_TURN, bank->current_fir_wf);
    bank->out_wf_pv = PUBLISH_WF_READ_VAR_I(
        char, FORMAT("OUTWF"), SAMPLES_PER_TURN, bank->current_out_wf);
    bank->gain_wf_pv = PUBLISH_WF_READ_VAR_I(
        int, FORMAT("GAINWF"), SAMPLES_PER_TURN, bank->current_gain_wf);

#undef FORMAT
}


static struct bunch_bank banks[BUNCH_BANKS];


static void write_nco_freq(double tune)
{
    hw_write_nco_freq((unsigned int) round(pow(2,33) * tune / 936.0));
}


bool initialise_bunch_select(void)
{
    PUBLISH_ACTION("BUN:SYNC", hw_write_bun_sync);
    for (int i = 0; i < BUNCH_BANKS; i ++)
        publish_bank(i, &banks[i]);

    PUBLISH_WRITER_P(ao, "NCO:FREQ", write_nco_freq);
    PUBLISH_WRITER_P(mbbo, "NCO:GAIN", hw_write_nco_gain);
    return true;
}
