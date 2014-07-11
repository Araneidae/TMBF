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


/* Need to serialise some of our operations. */
static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;

#define LOCK()      pthread_mutex_lock(&lock);
#define UNLOCK()    pthread_mutex_unlock(&lock);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* DDR waveform readout records. */

static int selected_bunch = 0;
static int selected_turn = 0;

/* Triggers for long and short waveform readout. */
static struct epics_record *long_trigger;
static struct epics_record *short_trigger;

static bool long_enable;                // Whether long data capture is enabled
static struct in_epics_record_bi *long_data_ready;   // Status report to outside

static bool ddr_autostop;


/* Used for waiting for record processing completion callbacks. */
static pthread_cond_t done_cond = PTHREAD_COND_INITIALIZER;
static bool short_done, long_done;


/* Waveform readout methods. */

static void read_bunch_waveform(short *waveform)
{
    read_ddr_bunch(0, selected_bunch, BUFFER_TURN_COUNT, waveform);
}

static void read_short_turn_waveform(short *waveform)
{
    read_ddr_turns(0, SHORT_TURN_WF_COUNT, waveform);
}

static void read_long_turn_waveform(short *waveform)
{
    read_ddr_turns(selected_turn, LONG_TURN_WF_COUNT, waveform);
}


/* Just writes value to DDR:READY flag. */
static void set_long_data_ready(bool ready)
{
    WRITE_IN_RECORD(bi, long_data_ready, ready);
}


static void set_long_enable(bool enable)
{
    LOCK();
    if (enable != long_enable)
    {
        long_enable = enable;
        if (long_enable)
            trigger_record(long_trigger, 0, NULL);
    }
    UNLOCK();
}


/* This is called each time the DDR buffer successfully triggers.  The short
 * waveforms are always triggered, but the long waveform is only triggered when
 * we're in one-shot mode. */
void process_ddr_buffer(void)
{
    printf("Process DDR buffer\n");

    LOCK();
    if (long_enable)
        trigger_record(long_trigger, 0, NULL);
    trigger_record(short_trigger, 0, NULL);

    /* Now wait for the completion callbacks from both records (if necessary)
     * have reported completion. */
    short_done = false;
    long_done = !long_enable;
    while (!short_done  ||  !long_done)
        pthread_cond_wait(&done_cond, &lock);
    UNLOCK();
}

void disarm_ddr_buffer(void)
{
    hw_write_ddr_disable();
}

void notify_ddr_seq_ready(void)
{
    printf("SEQ ready notify!\n");
    if (ddr_autostop)  // && IQ capture
        disarm_ddr_buffer();
}


/* Called each time the DDR buffer is armed.  In response we have to invalidate
 * the long term data. */
void prepare_ddr_buffer(void)
{
    printf("Prepare DDR buffer\n");
    // !!!!  Write input select and record setting

    hw_write_ddr_enable();  // Initiate capture of selected DDR data

    LOCK();
    set_long_data_ready(false);
    UNLOCK();
}


/* Called on completion of short buffer processing.  Report completion to
 * waiting process_ddr_buffer call. */
static void short_trigger_done(void)
{
    LOCK();
    short_done = true;
    pthread_cond_signal(&done_cond);
    UNLOCK();
}


/* Called on completion of long buffer processing.  Update ready status. */
static void long_trigger_done(void)
{
    LOCK();
    long_done = true;
    pthread_cond_signal(&done_cond);
    set_long_data_ready(true);
    UNLOCK();
}


static void set_selected_turn(int value)
{
    selected_turn = value;
    LOCK();
    if (long_enable)
        trigger_record(long_trigger, 0, NULL);
    UNLOCK();
}


static void write_ddr_select(unsigned int select)
{
    hw_write_ddr_select(select);
}


static uint32_t read_ddr_count(void)
{
    uint32_t offset;
    bool iq_select;
    hw_read_ddr_offset(&offset, &iq_select);
    if (iq_select)
        return offset;
    else
        return 0;
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

    /* These two interlock chains are used to process the waveforms. */
    short_trigger = PUBLISH_TRIGGER("DDR:SHORT:TRIG");
    long_trigger  = PUBLISH_TRIGGER("DDR:LONG:TRIG");
    PUBLISH_ACTION("DDR:SHORT:DONE", short_trigger_done);
    PUBLISH_ACTION("DDR:LONG:DONE",  long_trigger_done);

    /* Used to report completion of long record updates. */
    long_data_ready = PUBLISH_IN_VALUE_I(bi, "DDR:READY");

    /* Select bunch for single bunch waveform. */
    PUBLISH_WRITE_VAR(longout, "DDR:BUNCHSEL", selected_bunch);

    /* Turn selection and readback.  Readback is triggered after completion of
     * long record processing. */
    PUBLISH_WRITER_P(bo, "DDR:LONGEN", set_long_enable);
    PUBLISH_WRITER(longout, "DDR:TURNSEL", set_selected_turn);
    PUBLISH_READ_VAR(longin, "DDR:TURNSEL", selected_turn);

    /* Data source. */
    PUBLISH_WRITER_P(mbbo, "DDR:INPUT", write_ddr_select);

    PUBLISH_ACTION("DDR:START", hw_write_ddr_enable);
    PUBLISH_ACTION("DDR:STOP", hw_write_ddr_disable);
    PUBLISH_READER(ulongin, "DDR:COUNT", read_ddr_count);
    PUBLISH_WRITE_VAR_P(bo, "DDR:AUTOSTOP", ddr_autostop);

    return initialise_ddr();
}
