/* Bunch selection control. */

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <pthread.h>
#include <math.h>

#include "error.h"
#include "hardware.h"
#include "epics_device.h"
#include "epics_extra.h"
#include "adc_dac.h"
#include "tmbf.h"

#include "bunch_select.h"


struct bunch_bank {
    unsigned int index;

    int fir_wf[BUNCHES_PER_TURN];
    int out_wf[BUNCHES_PER_TURN];
    int gain_wf[BUNCHES_PER_TURN];

    EPICS_STRING fir_status;
    EPICS_STRING out_status;
    EPICS_STRING gain_status;
};


#define GAIN_SCALE      1024



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Status string computation. */

/* Helper routine: counts number of instances of given value, not all set,
 * returns index of last non-equal value. */
static int count_value(const int *wf, int value, unsigned int *diff_ix)
{
    int count = 0;
    for (unsigned int i = 0; i < BUNCHES_PER_TURN; i ++)
        if (wf[i] == value)
            count += 1;
        else
            *diff_ix = i;
    return count;
}


/* Status computation.  We support three possibilities:
 *  1.  All one value
 *  2.  All one value except for one different value
 *  3.  Something else: "It's complicated" */
enum complexity { ALL_SAME, ALL_BUT_ONE, COMPLICATED };
static enum complexity assess_complexity(
    const int wf[BUNCHES_PER_TURN],
    int *value, int *other, unsigned int *other_ix)
{
    *value = wf[0];
    *other_ix = 0;
    *other = 0;
    switch (count_value(wf, *value, other_ix))
    {
        case BUNCHES_PER_TURN:
            return ALL_SAME;
        case 1:
            /* Need to check whether all rest are the same. */
            *value = wf[1];
            *other = wf[0];
            if (count_value(wf, *value, other_ix) == BUNCHES_PER_TURN-1)
                return ALL_BUT_ONE;
            else
                return COMPLICATED;
        case BUNCHES_PER_TURN-1:
            *other = wf[*other_ix];
            return ALL_BUT_ONE;
        default:
            return COMPLICATED;
    }
}


static void update_status_core(
    const char *name, const int *wf, EPICS_STRING *status,
    void (*render)(int, char[]))
{
    int value, other;
    unsigned int other_ix;
    enum complexity complexity =
        assess_complexity(wf, &value, &other, &other_ix);
    char value_name[20], other_name[20];
    render(value, value_name);
    render(other, other_name);

    switch (complexity)
    {
        case ALL_SAME:
            snprintf(status->s, 40, "%s", value_name);
            break;
        case ALL_BUT_ONE:
            snprintf(status->s, 40, "%s (%s @%u)",
                value_name, other_name, other_ix);
            break;
        case COMPLICATED:
            snprintf(status->s, 40, "Mixed %s", name);
            break;
    }
}


/* Name rendering methods for calls to update_status_core above.  These three
 * functions are invoked in the macro DEFINE_WRITE_WF below. */

static void fir_name(int fir, char result[])
{
    sprintf(result, "#%d", fir);
}

static void out_name(int out, char result[])
{
    const char *out_names[] = {
        "Off",      "FIR",      "NCO",      "NCO+FIR",
        "Sweep",    "Sw+FIR",   "Sw+NCO",   "Sw+N+F" };
    ASSERT_OK(0 <= out  &&  out < (int) ARRAY_SIZE(out_names));
    sprintf(result, "%s", out_names[out]);
}

static void gain_name(int gain, char result[])
{
    sprintf(result, "%.3g", (double) gain / GAIN_SCALE);
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
#define DEFINE_WRITE_WF(type, field, name) \
    static void write_##field##_wf(void *context, type *wf, size_t *length) \
    { \
        struct bunch_bank *bank = context; \
        *length = BUNCHES_PER_TURN; \
        for (int i = 0; i < BUNCHES_PER_TURN; i ++) \
            bank->field##_wf[i] = wf[i]; \
        update_bunch_select(bank); \
        update_status_core(name, \
            bank->field##_wf, &bank->field##_status, field##_name); \
    }

DEFINE_WRITE_WF(char, fir, "FIR")
DEFINE_WRITE_WF(char, out, "outputs")
DEFINE_WRITE_WF(int, gain, "gains")

static void write_gain_wf_float(void *context, float gain[], size_t *length)
{
    int gain_int[BUNCHES_PER_TURN];
    float_array_to_int(BUNCHES_PER_TURN, gain, gain_int, 11, 0);
    write_gain_wf(context, gain_int, length);
}


static void publish_bank(unsigned int ix, struct bunch_bank *bank)
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
    PUBLISH_BANK_WF(float, "GAINWF_S", write_gain_wf_float);

    PUBLISH_READ_VAR(stringin, FORMAT("FIRWF:STA"), bank->fir_status);
    PUBLISH_READ_VAR(stringin, FORMAT("OUTWF:STA"), bank->out_status);
    PUBLISH_READ_VAR(stringin, FORMAT("GAINWF:STA"), bank->gain_status);

#undef PUBLISH_BANK_WF
#undef FORMAT
}


static struct bunch_bank banks[BUNCH_BANKS];

static void publish_bank_control(void)
{
    for (unsigned int i = 0; i < BUNCH_BANKS; i ++)
        publish_bank(i, &banks[i]);
}


const int *read_bank_out_wf(unsigned int bank)
{
    return banks[bank].out_wf;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Feedback mode calculation. */


static EPICS_STRING read_feedback_mode(void)
{
    unsigned int current_bank = READ_NAMED_RECORD(mbbo, "SEQ:0:BANK");
    struct bunch_bank *bank = &banks[current_bank];

    /* Evaluate DAC out and FIR waveforms. */
    bool all_off = true;
    bool all_fir = true;
    bool same_fir = true;
    for (int i = 0; i < BUNCHES_PER_TURN; i ++)
    {
        if ((bank->out_wf[i] & DAC_OUT_FIR) != 0)
            all_off = false;
        if (bank->out_wf[i] != DAC_OUT_FIR)
            all_fir = false;
        if (bank->fir_wf[i] != bank->fir_wf[0])
            same_fir = false;
    }

    EPICS_STRING result;
    if (all_off)
        snprintf(result.s, 40, "Feedback off");
    else if (!all_fir)
        snprintf(result.s, 40, "Feedback mixed mode");
    else if (same_fir)
        snprintf(result.s, 40, "Feedback on, FIR: #%d", bank->fir_wf[0]);
    else
        snprintf(result.s, 40, "Feedback on, FIR: mixed");
    return result;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Bunch synchronisation. */

/* Bunch synchronisation.  This is a trifle involved because when
 * synchronisation is started we then need to poll the trigger phase until
 * triggering is complete and then update the ADC skew accordingly.  We spawn a
 * thread who's only purpose in life is to poll the bunch trigger status until a
 * trigger occurs. */

static struct epics_interlock *sync_interlock;
static unsigned int zero_bunch_offset;
static unsigned int sync_phase;
enum sync_status {
    SYNC_NO_SYNC,   // Initial startup state, need to synchronise
    SYNC_ZERO,      // Bunch reset to sync on bunch zero
    SYNC_WAITING,   // Waiting for first synchronisation trigger
    SYNC_FIRST,     // First trigger seen, waiting for second trigger
    SYNC_OK,        // Synchronisation completed ok
    SYNC_FAILED     // Synchronisation failed
};
static unsigned int sync_status = SYNC_NO_SYNC;

static pthread_mutex_t bunch_sync_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t bunch_sync_signal = PTHREAD_COND_INITIALIZER;
enum sync_action { SYNC_ACTION_RESET, SYNC_ACTION_SYNC } sync_action;


/* Called from bunch synchronisation thread to block until action requested. */
static enum sync_action wait_for_sync_action(void)
{
    ASSERT_0(pthread_mutex_lock(&bunch_sync_mutex));
    ASSERT_0(pthread_cond_wait(&bunch_sync_signal, &bunch_sync_mutex));
    enum sync_action action = sync_action;
    ASSERT_0(pthread_mutex_unlock(&bunch_sync_mutex));
    return action;
}

/* Requests specified action from bunch synchronisation thread. */
static void request_sync_action(enum sync_action action)
{
    ASSERT_0(pthread_mutex_lock(&bunch_sync_mutex));
    sync_action = action;
    ASSERT_0(pthread_cond_signal(&bunch_sync_signal));
    ASSERT_0(pthread_mutex_unlock(&bunch_sync_mutex));
}


/* Updates reported phase and status. */
static void update_status(unsigned int phase, enum sync_status status)
{
    interlock_wait(sync_interlock);
    sync_phase = phase;
    sync_status = status;
    interlock_signal(sync_interlock, NULL);
}

/* Triggers bunch synchronisation and busy waits for completion.  Returns
 * measured phase after successful synchronisation. */
static unsigned int wait_for_bunch_sync(void)
{
    /* Configure selected zero bunch offset and request synchronisation. */
    hw_write_bun_sync();

    /* Busy loop until trigger seen. */
    unsigned int phase;
    while (phase = hw_read_bun_trigger_phase(),
           phase == 0)
        usleep(20000);     // 20 ms
    return phase;
}

/* Each valid phase bit pattern corresponds to a clock skew in bunches: this is
 * the delay in 2ns bunch clocks from the start of the 8ns FPGA clock to the
 * trigger. */
static bool phase_to_skew(unsigned int phase, unsigned int *skew)
{
    switch (phase)
    {
        case 8:     *skew = 3;  break;
        case 12:    *skew = 2;  break;
        case 14:    *skew = 1;  break;
        case 15:    *skew = 0;  break;
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
    unsigned int phase = wait_for_bunch_sync();

    unsigned int skew;
    if (phase_to_skew(phase, &skew))
    {
        /* Correct for trigger delay by adding extra delay in ADC input. */
        set_adc_skew(3 - skew);
        update_status(phase, SYNC_ZERO);
    }
    else
        update_status(phase, SYNC_FAILED);
}


/* Computes ADC skew and bunch synchronisation offset to ensure that the
 * selected zero bunch is reported as bunch zero. */
static void select_bunch_zero(unsigned int base_skew)
{
    /* Total offset of target bunch from trigger point. */
    unsigned int skewed_offset = zero_bunch_offset + base_skew;
    /* Compute the index of the clock containing the zero bunch relative to the
     * current trigger; in this case we assume that bunch zero has been
     * configured with bunch synchronisation reset. */
    unsigned int clock_delay = skewed_offset / 4;
    unsigned int zero_clock =
        clock_delay == 0 ? 0 : BUNCHES_PER_TURN / 4 - clock_delay;
    /* The final adjustment needs a fine delay of 0 to 3 bunches within a single
     * FPGA capture clock. */
    unsigned int adc_skew = 3 - skewed_offset % 4;

    set_adc_skew(adc_skew);
    hw_write_bun_zero_bunch(zero_clock);
}


/* Use double triggering to discover the trigger skew and then offset the bunch
 * zero so the selected bunch becomes bunch zero. */
static void do_sync_bunch_sync(void)
{
    /* Discover trigger skew.  At this stage we leave the bunch offset alone, we
     * need the skew to determine the final offset. */
    update_status(0, SYNC_WAITING);
    unsigned int phase = wait_for_bunch_sync();

    /* Decode the skew and set up for the second trigger. */
    unsigned int base_skew;
    if (phase_to_skew(phase, &base_skew))
    {
        /* Figure out the right skew and whether we need to adjust the bin
         * offset and retrigger.  We always retrigger, even if the bin offset
         * hasn't changed, for consistency and a sanity check on the phase. */

        select_bunch_zero(base_skew);

        update_status(phase, SYNC_FIRST);
        unsigned int second_phase = wait_for_bunch_sync();

        /* We expect the trigger phases to be identical between the two
         * triggers, if not there is a problem. */
        if (phase == second_phase)
            update_status(phase, SYNC_OK);
        else
        {
            print_error(
                "Bunch sync phase discrepancy: %u != %u\n",
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
        switch (wait_for_sync_action())
        {
            case SYNC_ACTION_RESET: do_reset_bunch_sync(); break;
            case SYNC_ACTION_SYNC:  do_sync_bunch_sync();  break;
        }
    }
    return NULL;
}

static void reset_bunch_sync(void)
{
    request_sync_action(SYNC_ACTION_RESET);
}

static void sync_bunch_sync(void)
{
    request_sync_action(SYNC_ACTION_SYNC);
}


static bool publish_bunch_sync(void)
{
    PUBLISH_WRITE_VAR_P(ulongout, "BUN:OFFSET", zero_bunch_offset);
    PUBLISH_ACTION("BUN:RESET", reset_bunch_sync);
    PUBLISH_ACTION("BUN:SYNC", sync_bunch_sync);

    sync_interlock = create_interlock("BUN:SYNC", false);
    PUBLISH_READ_VAR(ulongin, "BUN:PHASE", sync_phase);
    PUBLISH_READ_VAR(mbbi, "BUN:STATUS", sync_status);

    PUBLISH_READER(stringin, "BUN:MODE", read_feedback_mode);

    pthread_t thread_id;
    return TEST_0(pthread_create(&thread_id, NULL, bunch_sync_thread, NULL));
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

bool initialise_bunch_select(void)
{
    publish_bank_control();
    return publish_bunch_sync();
}
