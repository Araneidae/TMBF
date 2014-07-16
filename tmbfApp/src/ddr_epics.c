/* EPICS interface to DDR buffer. */

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <pthread.h>

#include "error.h"
#include "ddr.h"
#include "hardware.h"
#include "epics_device.h"
#include "epics_extra.h"

#include "ddr_epics.h"



/* Total number of turns in the DDR buffer following the trigger.  We have to
 * subtract a couple of turns because of some jitter in the trigger. */
#define BUFFER_TURN_COUNT   (64 * 1024 * 1024 / BUNCHES_PER_TURN / 2 - 2)

/* Size of turn readout waveform. */
#define SHORT_TURN_WF_COUNT       8
#define LONG_TURN_WF_COUNT        256


/* Need to interlock access to the DDR hardware. */
static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;

#define LOCK()      pthread_mutex_lock(&lock);
#define UNLOCK()    pthread_mutex_unlock(&lock);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* DDR waveform readout records. */

/* The input selection can be changed at any time, but is only written to
 * hardware when enabling the DDR input. */
static unsigned int new_input_selection;    // written by EPICS
static unsigned int input_selection;        // written to hardware

static unsigned int selected_bunch = 0;
static int selected_turn = 0;
static bool bunch_waveform_fault;
static bool long_waveform_fault;

static bool ddr_autostop;

/* This is set during capture and reset when capture completes. */
static bool capture_active;

static struct epics_interlock *update_trigger;

/* Overflow status from the last sweep. */
static bool overflows[PULSED_BIT_COUNT];
static struct epics_interlock *overflows_interlock;


/* Waveform readout methods.  Each of these is called as part of EPICS
 * processing triggered in response to process_ddr_buffer. */

static void read_bunch_waveform(short waveform[])
{
    bunch_waveform_fault =
        !read_ddr_bunch(0, selected_bunch, BUFFER_TURN_COUNT, waveform);
}

static void read_short_turn_waveform(short waveform[])
{
    read_ddr_turns(0, SHORT_TURN_WF_COUNT, waveform);
}

static void read_long_turn_waveform(short waveform[])
{
    long_waveform_fault =
        !read_ddr_turns(selected_turn, LONG_TURN_WF_COUNT, waveform);
}


/* Reads DDR specific overflow bits. */
static void read_overflows(bool overflow_bits[PULSED_BIT_COUNT])
{
    const bool read_mask[PULSED_BIT_COUNT] = {
        [OVERFLOW_IQ_FIR_DDR] = true,
        [OVERFLOW_IQ_ACC_DDR] = true,
        [OVERFLOW_IQ_SCALE_DDR] = true,
    };
    hw_read_pulsed_bits(read_mask, overflow_bits);
}

/* Called periodically during IQ data capture to refresh the overflow bits. */
static void update_overflows(void)
{
    bool new_overflows[PULSED_BIT_COUNT];
    read_overflows(new_overflows);

    interlock_wait(overflows_interlock);
    for (int i = 0; i < PULSED_BIT_COUNT; i++)
        overflows[i] |= new_overflows[i];
    interlock_signal(overflows_interlock, NULL);
}

/* Called at the start of IQ capture. */
static void reset_overflows(void)
{
    interlock_wait(overflows_interlock);
    read_overflows(overflows);
    interlock_signal(overflows_interlock, NULL);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* Called each time the DDR buffer is armed.  In response we have to invalidate
 * the long term data. */
void prepare_ddr_buffer(void)
{
    LOCK();
    if (capture_active)
        print_error("Ignoring unexpected DDR start: already running");
    else
    {
        input_selection = new_input_selection;
        hw_write_ddr_select(input_selection);
        hw_write_ddr_enable();  // Initiate capture of selected DDR data
        reset_overflows();
        capture_active = true;
    }
    UNLOCK();
}


/* This is called each time the DDR buffer successfully triggers. */
void process_ddr_buffer(void)
{
    LOCK();
    if (!capture_active)
        print_error("Unexpected DDR process: not running?");

    update_overflows();
    capture_active = false;

    /* Now read out the processed waveforms. */
    interlock_signal(update_trigger, NULL);
    interlock_wait(update_trigger);
    UNLOCK();
}


/* Called when the DDR trigger is deliberately disarmed.*/
void disarm_ddr_buffer(void)
{
    LOCK();
    /* Only halt the DDR buffer if in normal capture mode, as otherwise it's
     * possible that the DDR trigger is being used to trigger the sequencer in
     * multi-shot mode. */
    if (!input_selection < DDR_SELECT_IQ)
        hw_write_ddr_disable();
    UNLOCK();
}


/* Called when the sequencer has finished a round of activity. */
void notify_ddr_seq_ready(void)
{
    LOCK();
    if (ddr_autostop  &&  input_selection == DDR_SELECT_IQ)
        hw_write_ddr_disable();
    UNLOCK();
}


/* Reads current capture count from DDR.  Only meaningful if the currently
 * selected source is IQ or Debug. */
static uint32_t read_ddr_count(void)
{
    LOCK();
    uint32_t count;
    if (input_selection >= DDR_SELECT_IQ)
    {
        update_overflows();
        /* Convert count into number of IQ samples. */
        count = hw_read_ddr_offset() / 2;
    }
    else
        count = 0;
    UNLOCK();
    return count;
}


bool initialise_ddr_epics(void)
{
    /* The three waveforms.  Each of these reads its value directly from the
     * buffer when processed. */
    PUBLISH_WF_ACTION(short, "DDR:BUNCHWF",
        BUFFER_TURN_COUNT, read_bunch_waveform);
    PUBLISH_WF_ACTION(short, "DDR:SHORTWF",
        SHORT_TURN_WF_COUNT * BUNCHES_PER_TURN, read_short_turn_waveform);
    PUBLISH_WF_ACTION(short, "DDR:LONGWF",
        LONG_TURN_WF_COUNT * BUNCHES_PER_TURN, read_long_turn_waveform);
    PUBLISH_READ_VAR(bi, "DDR:BUNCHWF:STATUS", bunch_waveform_fault);
    PUBLISH_READ_VAR(bi, "DDR:LONGWF:STATUS", long_waveform_fault);

    /* Interlock for record update when ready.  We use the interlock backwards
     * so that we block until they've all processed. */
    update_trigger = create_interlock("DDR:UPDATE", false);

    /* Control variables for record readout. */
    PUBLISH_WRITE_VAR(ulongout, "DDR:BUNCHSEL", selected_bunch);
    PUBLISH_WRITE_VAR(longout, "DDR:TURNSEL", selected_turn);
    PUBLISH_WRITE_VAR_P(mbbo, "DDR:INPUT", new_input_selection);

    /* DDR control and status readbacks. */
    PUBLISH_ACTION("DDR:START", prepare_ddr_buffer);
    PUBLISH_ACTION("DDR:STOP", hw_write_ddr_disable);
    PUBLISH_READER(ulongin, "DDR:COUNT", read_ddr_count);
    PUBLISH_WRITE_VAR_P(bo, "DDR:AUTOSTOP", ddr_autostop);

    /* Overflow detection. */
    PUBLISH_READ_VAR(bi, "DDR:OVF:INP", overflows[OVERFLOW_IQ_FIR_DDR]);
    PUBLISH_READ_VAR(bi, "DDR:OVF:ACC", overflows[OVERFLOW_IQ_ACC_DDR]);
    PUBLISH_READ_VAR(bi, "DDR:OVF:IQ",  overflows[OVERFLOW_IQ_SCALE_DDR]);
    overflows_interlock = create_interlock("DDR:OVF", false);

    return initialise_ddr();
}
