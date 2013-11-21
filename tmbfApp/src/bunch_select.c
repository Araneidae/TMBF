/* Bunch selection control. */

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <pthread.h>
#include <semaphore.h>

#include "error.h"
#include "hardware.h"
#include "epics_device.h"
#include "adc_dac.h"

#include "bunch_select.h"


struct bunch_bank {
    int index;
    bool use_waveform;
    unsigned int fir_select;
    unsigned int out_select;
    int gain_select;
    char current_fir_wf[BUNCHES_PER_TURN];
    char current_out_wf[BUNCHES_PER_TURN];
    int current_gain_wf[BUNCHES_PER_TURN];
    char set_fir_wf[BUNCHES_PER_TURN];
    char set_out_wf[BUNCHES_PER_TURN];
    int set_gain_wf[BUNCHES_PER_TURN];
    struct epics_record *fir_wf_pv;
    struct epics_record *out_wf_pv;
    struct epics_record *gain_wf_pv;
};


/* Called when the current waveforms are up to date.  Write the current
 * settings out to the FPGA. */
static void update_bunch_select(struct bunch_bank *bank)
{
    /* Reshape our waveforms into format suitable for hardware. */
    struct bunch_entry entries[BUNCHES_PER_TURN];
    for (int i = 0; i < BUNCHES_PER_TURN; i++)
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
    for (int i = 0; i < BUNCHES_PER_TURN; i ++) \
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
        char, FORMAT("FIRWF_S"), BUNCHES_PER_TURN, bank->set_fir_wf);
    PUBLISH_WF_WRITE_VAR_P(
        char, FORMAT("OUTWF_S"), BUNCHES_PER_TURN, bank->set_out_wf);
    PUBLISH_WF_WRITE_VAR_P(
        int, FORMAT("GAINWF_S"), BUNCHES_PER_TURN, bank->set_gain_wf);
    bank->fir_wf_pv = PUBLISH_WF_READ_VAR_I(
        char, FORMAT("FIRWF"), BUNCHES_PER_TURN, bank->current_fir_wf);
    bank->out_wf_pv = PUBLISH_WF_READ_VAR_I(
        char, FORMAT("OUTWF"), BUNCHES_PER_TURN, bank->current_out_wf);
    bank->gain_wf_pv = PUBLISH_WF_READ_VAR_I(
        int, FORMAT("GAINWF"), BUNCHES_PER_TURN, bank->current_gain_wf);

#undef FORMAT
}


static struct bunch_bank banks[BUNCH_BANKS];


/* Bunch synchronisation.  This is a trifle involved because when
 * synchronisation is started we then need to poll the trigger phase until
 * triggering is complete and then update the ADC skew accordingly.  We spawn a
 * thread who's only purpose in life is to poll the bunch trigger status until a
 * trigger occurs. */

static sem_t bunch_sync_sem;
static struct epics_record *sync_phase_pv;
static int sync_phase;

/* Each valid phase bit pattern corresponds to a clock skew in bunches. */
static bool phase_to_skew(int phase, unsigned int *skew)
{
    switch (phase)
    {
        case 8:     *skew = 0;  break;
        case 12:    *skew = 1;  break;
        case 14:    *skew = 2;  break;
        case 15:    *skew = 3;  break;
        default:    return FAIL_("Invalid phase: %d\n", phase);
    }
    return true;
}

static void *bunch_sync_thread(void *context)
{
    sync_phase = hw_read_bun_trigger_phase();
    trigger_record(sync_phase_pv, 0, NULL);

    while (true)
    {
        /* Block until sync button pressed, then request sync operation. */
        sem_wait(&bunch_sync_sem);
        hw_write_bun_sync();

        /* Now loop until trigger seen. */
        do {
            sync_phase = hw_read_bun_trigger_phase();
            trigger_record(sync_phase_pv, 0, NULL);
            usleep(100000);
        } while (sync_phase == 0);

        /* Finally use the measured phase to update the ADC skew. */
        unsigned int skew;
        if (phase_to_skew(sync_phase, &skew))
            set_adc_skew(skew);
    }
    return NULL;
}

static void write_bun_sync(void)
{
    sem_post(&bunch_sync_sem);
}


bool initialise_bunch_select(void)
{
    PUBLISH_WRITER_P(longout, "BUN:OFFSET", hw_write_bun_zero_bunch);
    PUBLISH_ACTION("BUN:SYNC", write_bun_sync);
    sync_phase_pv = PUBLISH_READ_VAR_I(longin, "BUN:PHASE", sync_phase);

    for (int i = 0; i < BUNCH_BANKS; i ++)
        publish_bank(i, &banks[i]);

    pthread_t thread_id;
    return
        TEST_IO(sem_init(&bunch_sync_sem, 0, 0))  &&
        TEST_0(pthread_create(&thread_id, NULL, bunch_sync_thread, NULL));
}
