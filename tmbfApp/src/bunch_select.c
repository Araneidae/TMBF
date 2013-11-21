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

    int fir_wf[BUNCHES_PER_TURN];
    int out_wf[BUNCHES_PER_TURN];
    int gain_wf[BUNCHES_PER_TURN];

    EPICS_STRING fir_status;
    EPICS_STRING out_status;
    EPICS_STRING gain_status;
};


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Status string computation. */

/* Helper routine: counts number of instances of given value, not all set,
 * returns index of last non-equal value. */
static int count_value(int *wf, int value, int *diff_ix)
{
    int count = 0;
    for (int i = 0; i < BUNCHES_PER_TURN; i ++)
        if (wf[i] == value)
            count += 1;
        else
            *diff_ix = i;
    return count;
}


/* Converts output control into concise displayable value. */
static const char *out_name(int value)
{
    const char *out_names[] = {
        "Off",      "FIR",      "NCO",      "NCO+FIR",
        "Sweep",    "Sw+FIR",   "Sw+NCO",   "S+F+N" };
    if (0 <= value  &&  value < 8)
        return out_names[value];
    else
        return "Error";
}


/* Status computation.  We support three possibilities:
 *  1.  All one value
 *  2.  All one value except for one different value
 *  3.  Something else: "It's complicated" */
enum complexity { ALL_SAME, ALL_BUT_ONE, COMPLICATED };
static enum complexity assess_complexity(
    int wf[BUNCHES_PER_TURN], int *value, int *other, int *other_ix)
{
    *other_ix = 0;
    *value = wf[0];
    int count = count_value(wf, *value, other_ix);
    if (count == BUNCHES_PER_TURN)
        return ALL_SAME;
    else if (1 < count  &&  count < BUNCHES_PER_TURN-1)
        return COMPLICATED;
    else
    {
        *other = wf[*other_ix];
        if (count == 1)
        {
            int t = *value; *value = *other; *other = t;
            *other_ix = 0;
        }
        return ALL_BUT_ONE;
    }
}

static void update_fir_status(struct bunch_bank *bank)
{
    int value, other, other_ix;
    switch (assess_complexity(bank->fir_wf, &value, &other, &other_ix))
    {
        case ALL_SAME:
            sprintf(bank->fir_status.s, "All #%d", value);
            break;
        case ALL_BUT_ONE:
            sprintf(bank->fir_status.s, "#%d (#%d @%d)",
                value, other, other_ix);
            break;
        case COMPLICATED:
            sprintf(bank->fir_status.s, "Mixed");
            break;
    }
}

static void update_out_status(struct bunch_bank *bank)
{
    int value, other, other_ix;
    switch (assess_complexity(bank->out_wf, &value, &other, &other_ix))
    {
        case ALL_SAME:
            sprintf(bank->out_status.s, "%s", out_name(value));
            break;
        case ALL_BUT_ONE:
            sprintf(bank->out_status.s, "%s (%s @%d)",
                out_name(value), out_name(other), other_ix);
            break;
        case COMPLICATED:
            sprintf(bank->out_status.s, "Mixed outputs");
            break;
    }
}

static void update_gain_status(struct bunch_bank *bank)
{
    int value, other, other_ix;
    switch (assess_complexity(bank->gain_wf, &value, &other, &other_ix))
    {
        case ALL_SAME:
            sprintf(bank->gain_status.s, "%d", value);
            break;
        case ALL_BUT_ONE:
            sprintf(bank->gain_status.s, "%d (%d @%d)",
                value, other, other_ix);
            break;
        case COMPLICATED:
            sprintf(bank->gain_status.s, "Mixed gains");
            break;
    }
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Bunch publish and control. */

/* Called when the current waveforms are up to date.  Write the current
 * settings out to the FPGA. */
static void update_bunch_select(struct bunch_bank *bank)
{
    /* Reshape our waveforms into format suitable for hardware. */
    struct bunch_entry entries[BUNCHES_PER_TURN];
    for (int i = 0; i < BUNCHES_PER_TURN; i++)
    {
        entries[i].bunch_gain    = bank->gain_wf[i];
        entries[i].output_select = bank->out_wf[i];
        entries[i].fir_select    = bank->fir_wf[i];
    }
    hw_write_bun_entry(bank->index, entries);
}



/* The update for each bunch control waveform follows the same pattern, but
 * there are type differences preventing direct reuse. */
#define DEFINE_WRITE_WF(type, field) \
    static void write_##field##_wf(void *context, type *wf, size_t *length) \
    { \
        struct bunch_bank *bank = context; \
        *length = BUNCHES_PER_TURN; \
        for (int i = 0; i < BUNCHES_PER_TURN; i ++) \
            bank->field##_wf[i] = wf[i]; \
        update_bunch_select(bank); \
        update_##field##_status(bank); \
    }

DEFINE_WRITE_WF(char, fir)
DEFINE_WRITE_WF(char, out)
DEFINE_WRITE_WF(int, gain)


static void publish_bank(int ix, struct bunch_bank *bank)
{
    bank->index = ix;

    char buffer[20];
#define FORMAT(name) \
    (sprintf(buffer, "BUN:%d:%s", ix, name), buffer)

#define PUBLISH_BANK_WF(type, name, action) \
    PUBLISH_WAVEFORM( \
        type, FORMAT(name), BUNCHES_PER_TURN, action, \
        .context = bank, .persist = true)

    PUBLISH_BANK_WF(char, "FIRWF_S", write_fir_wf);
    PUBLISH_BANK_WF(char, "OUTWF_S", write_out_wf);
    PUBLISH_BANK_WF(int, "GAINWF_S", write_gain_wf);

    PUBLISH_READ_VAR(stringin, FORMAT("FIRWF:STA"), bank->fir_status);
    PUBLISH_READ_VAR(stringin, FORMAT("OUTWF:STA"), bank->out_status);
    PUBLISH_READ_VAR(stringin, FORMAT("GAINWF:STA"), bank->gain_status);

#undef PUBLISH_BANK_WF
#undef FORMAT
}


static struct bunch_bank banks[BUNCH_BANKS];

static void publish_bank_control(void)
{
    for (int i = 0; i < BUNCH_BANKS; i ++)
        publish_bank(i, &banks[i]);
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Bunch synchronisation. */

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



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

bool initialise_bunch_select(void)
{
    publish_bank_control();

    PUBLISH_WRITER_P(longout, "BUN:OFFSET", hw_write_bun_zero_bunch);
    PUBLISH_ACTION("BUN:SYNC", write_bun_sync);
    sync_phase_pv = PUBLISH_READ_VAR_I(longin, "BUN:PHASE", sync_phase);

    pthread_t thread_id;
    return
        TEST_IO(sem_init(&bunch_sync_sem, 0, 0))  &&
        TEST_0(pthread_create(&thread_id, NULL, bunch_sync_thread, NULL));
}
