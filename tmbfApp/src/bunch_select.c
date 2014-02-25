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
#include "epics_extra.h"
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

static struct epics_interlock *sync_interlock;
static int zero_bunch_offset;
static int sync_phase;
enum sync_status {
    SYNC_NO_SYNC,   // Initial startup state, need to synchronise
    SYNC_ZERO,      // Bunch reset to sync on bunch zero
    SYNC_WAITING,   // Waiting for first synchronisation trigger
    SYNC_FIRST,     // First trigger seen, waiting for second trigger
    SYNC_OK,        // Synchronisation completed ok
    SYNC_FAILED     // Synchronisation failed
};
static unsigned int sync_status = SYNC_NO_SYNC;

static sem_t bunch_sync_sem;
enum { SYNC_ACTION_RESET, SYNC_ACTION_SYNC } sync_action;


/* Updates reported phase and status. */
static void update_status(int phase, enum sync_status status)
{
    interlock_wait(sync_interlock);
    sync_phase = phase;
    sync_status = status;
    interlock_signal(sync_interlock, NULL);
}

/* Triggers bunch synchronisation and busy waits for completion.  Returns
 * measured phase after successful synchronisation. */
static int wait_for_bunch_sync(void)
{
    /* Configure selected zero bunch offset and request synchronisation. */
    hw_write_bun_sync();

    /* Busy loop until trigger seen. */
    int phase;
    while (phase = hw_read_bun_trigger_phase(),
           phase == 0)
        usleep(20000);     // 20 ms
    return phase;
}

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

/* Reset bunch synchronisation so that bunch zero corresponds to the trigger.
 * We don't care about any constant offsets between the trigger and the bunch,
 * but we want reliable identification of bunch zero. */
static void do_reset_bunch_sync(void)
{
    update_status(0, SYNC_WAITING);
    hw_write_bun_zero_bunch(0);
    int phase = wait_for_bunch_sync();

    unsigned int skew;
    if (phase_to_skew(phase, &skew))
    {
        set_adc_skew(skew);
        update_status(phase, SYNC_ZERO);
    }
    else
        update_status(phase, SYNC_FAILED);
}

/* Use double triggering to discover the trigger skew and then offset the bunch
 * zero so the selected bunch becomes bunch zero. */
static void do_sync_bunch_sync(void)
{
    /* Discover trigger skew.  At this stage we leave the bunch offset alone, we
     * need the skew to determine the final offset. */
    update_status(0, SYNC_WAITING);
    int phase = wait_for_bunch_sync();

    /* Decode the skew and set up for the second trigger. */
    unsigned int base_skew;
    if (phase_to_skew(phase, &base_skew))
    {
        /* Figure out the right skew and whether we need to adjust the bin
         * offset and retrigger.  We always retrigger, even if the bin offset
         * hasn't changed, for consistency and a sanity check on the phase. */

        /* Compute adc skew and figure out corresponding offset. */
        int offset = (BUNCHES_PER_TURN - zero_bunch_offset) / 4;
        int skew = 4 - zero_bunch_offset % 4 + base_skew;
        offset += skew / 4;
        skew = skew % 4;
        set_adc_skew(skew);
        hw_write_bun_zero_bunch(offset);

        update_status(phase, SYNC_FIRST);
        int second_phase = wait_for_bunch_sync();

        /* We expect the trigger phases to be identical between the two
         * triggers, if not there is a problem. */
        if (phase == second_phase)
            update_status(phase, SYNC_OK);
        else
        {
            print_error(
                "Bunch sync phase discrepancy: %d != %d\n",
                phase, second_phase);
            update_status(second_phase, SYNC_FAILED);
        }
    }
    else
        /* Invalid skew, fail right away. */
        update_status(phase, SYNC_FAILED);
}

static void *bunch_sync_thread(void *context)
{
    /* Read phase and update initial status on startup. */
    update_status(hw_read_bun_trigger_phase(), SYNC_NO_SYNC);

    while (true)
    {
        /* Block until sync button pressed, perform requested action. */
        sem_wait(&bunch_sync_sem);
        switch (sync_action)
        {
            case SYNC_ACTION_RESET: do_reset_bunch_sync(); break;
            case SYNC_ACTION_SYNC:  do_sync_bunch_sync();  break;
        }
    }
    return NULL;
}

static void reset_bunch_sync(void)
{
    sync_action = SYNC_ACTION_RESET;
    sem_post(&bunch_sync_sem);
}

static void sync_bunch_sync(void)
{
    sync_action = SYNC_ACTION_SYNC;
    sem_post(&bunch_sync_sem);
}

static bool publish_bunch_sync(void)
{
    PUBLISH_WRITE_VAR_P(longout, "BUN:OFFSET", zero_bunch_offset);
    PUBLISH_ACTION("BUN:RESET", reset_bunch_sync);
    PUBLISH_ACTION("BUN:SYNC", sync_bunch_sync);

    sync_interlock = create_interlock("BUN:SYNC:TRIG", "BUN:SYNC:DONE", false);
    PUBLISH_READ_VAR(longin, "BUN:PHASE", sync_phase);
    PUBLISH_READ_VAR(mbbi, "BUN:STATUS", sync_status);

    pthread_t thread_id;
    return
        TEST_IO(sem_init(&bunch_sync_sem, 0, 0))  &&
        TEST_0(pthread_create(&thread_id, NULL, bunch_sync_thread, NULL));
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

bool initialise_bunch_select(void)
{
    publish_bank_control();
    return publish_bunch_sync();
}
