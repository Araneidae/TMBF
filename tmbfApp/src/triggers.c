/* Common management of triggers and events. */

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <pthread.h>

#include "error.h"
#include "hardware.h"
#include "epics_device.h"
#include "ddr.h"
#include "ddr_epics.h"
#include "sequencer.h"

#include "triggers.h"


/* Need to serialise some of our operations. */
static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;

#define LOCK()      pthread_mutex_lock(&lock);
#define UNLOCK()    pthread_mutex_unlock(&lock);



/* Each of the two physical trigger targets (DDR and BUF) can separately be
 * configured to be triggered from the common external trigger or else from one
 * of two internal virtual soft trigger sources. */
enum {
    TRIG_SOURCE_DISABLE,
    TRIG_SOURCE_SOFT1,
    TRIG_SOURCE_SOFT2,
    TRIG_SOURCE_EXTERNAL
};


/* Used for reporting armed status. */
struct armed_status {
    bool armed;
    struct epics_record *status_pv;
};


/* Records state of a virtual trigger source (soft or external). */
struct trigger_source {
    unsigned int source_id;     // Source identification
    bool retrigger;             // If auto-retrigger enabled, written from PV
    struct armed_status armed;  // Aggregate status of all our targets

    /* We keep track of which targets this source triggered last time it fired
     * so that we can refresh our status as appropriate. */
    bool ddr_armed;
    bool buf_armed;
    bool seq_armed;
};


/* Records state of a trigger target (DDR or fast buffer). */
struct ddr_target {
    struct armed_status armed;
    unsigned int source_select; // Configured source, written from PV
    bool one_shot;
};

struct buf_target {
    struct armed_status armed;
    unsigned int source_select; // Configured source, written from PV
};

/* The sequencer is a little different as it shares triggering with the
 * buffer, but has its own enable state. */
struct seq_target {
    struct armed_status armed;
    bool enabled;               // Updated from PV
};



static struct trigger_source s1_source  = { .source_id = TRIG_SOURCE_SOFT1 };
static struct trigger_source s2_source  = { .source_id = TRIG_SOURCE_SOFT2 };
static struct trigger_source ext_source = { .source_id = TRIG_SOURCE_EXTERNAL };

static struct ddr_target ddr_target;
static struct buf_target buf_target;
static struct seq_target seq_target;



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Armed status reporting. */

/* Initialises armed_status structure including publishing of associated PV. */
static void publish_status(const char *name, struct armed_status *status)
{
    status->armed = false;
    status->status_pv = PUBLISH_READ_VAR_I(bi, name, status->armed);
}


/* Updates armed status and generates PV update if necessary. */
static void set_armed_status(struct armed_status *status, bool armed)
{
    if (armed != status->armed)
    {
        status->armed = armed;
        trigger_record(status->status_pv, 0, NULL);
    }
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Trigger targets (DDR/BUF/SEQ). */


static void do_arm_ddr(bool external, bool one_shot)
{
    set_armed_status(&ddr_target.armed, true);
    ddr_target.one_shot = one_shot;

    hw_write_ddr_trigger_select(external);
    arming_ddr_buffer();
}

static void disarm_ddr(void)
{
    set_armed_status(&ddr_target.armed, false);
}


static void do_arm_buf(bool external)
{
    set_armed_status(&buf_target.armed, true);
    hw_write_buf_trigger_select(external);
}

static void disarm_buf(void)
{
    set_armed_status(&buf_target.armed, false);
}


static void maybe_arm_seq(void)
{
    set_armed_status(&seq_target.armed, seq_target.enabled);
    enable_seq_trigger(seq_target.enabled);
}

static void disarm_seq(void)
{
    set_armed_status(&seq_target.armed, false);
    enable_seq_trigger(false);
}


/* Arming and firing of DDR and BUF triggers to hardware need to be
 * simultaneous, so are gathered in this function. */
static void arm_hardware(bool arm_ddr, bool arm_buf, bool external)
{
    hw_write_trg_arm(arm_ddr, arm_buf);
    if (!external)
        hw_write_trg_soft_trigger(arm_ddr, arm_buf);
}


/* Update the reported status for the three trigger targets as part of event
 * processing. */
static void update_target_status(bool ddr_ready, bool buf_ready, bool seq_ready)
{
    if (ddr_ready)  disarm_ddr();
    if (buf_ready)  disarm_buf();
    if (seq_ready)  disarm_seq();
}


static void publish_targets(void)
{
    publish_status("TRG:DDR:STATUS", &ddr_target.armed);
    publish_status("TRG:BUF:STATUS", &buf_target.armed);
    publish_status("TRG:SEQ:STATUS", &seq_target.armed);
    PUBLISH_WRITE_VAR_P(mbbo, "TRG:DDR:SEL", ddr_target.source_select);
    PUBLISH_WRITE_VAR_P(mbbo, "TRG:BUF:SEL", buf_target.source_select);
    PUBLISH_WRITE_VAR_P(bo,   "TRG:SEQ:ENA", seq_target.enabled);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Trigger sources (two software, one external). */


/* This is the starting point for all triggering.  This is called directly by
 * the user or can be invoked when retriggering. */
static void arm_and_fire(struct trigger_source *source)
{
    /* If we're already armed or if we're not actually configured to act on any
     * sources then arming is a no-op. */
    if (source->armed.armed)
        return;

    /* Work out which targets we're going to fire. */
    bool external = source->source_id == TRIG_SOURCE_EXTERNAL;
    bool arm_ddr = source->source_id == ddr_target.source_select;
    bool arm_buf = source->source_id == buf_target.source_select;
    bool arm_seq = arm_buf  &&  seq_target.enabled;

    /* Do nothing more if there are no takers. */
    if (!arm_ddr  &&  !arm_buf  &&  !arm_seq)
        return;

    /* Prepare the targets that need arming. */
    if (arm_ddr)
        do_arm_ddr(external, !source->retrigger);
    if (arm_buf)
    {
        do_arm_buf(external);
        maybe_arm_seq();
    }

    /* Hardware now ready, let's go. */
    arm_hardware(arm_ddr, arm_buf, external);

    /* Update reported status and record which reports we expect. */
    source->ddr_armed = arm_ddr;
    source->buf_armed = arm_buf;
    source->seq_armed = arm_seq;
    set_armed_status(&source->armed, true);
}

static bool call_arm_and_fire(void *context, const bool *value)
{
    LOCK();
    arm_and_fire(context);
    UNLOCK();
    return true;
}


/* Updates the status of a trigger source: if all of its targets have finished
 * processing then the source is ready, and can retrigger if appropriate. */
static void update_source(struct trigger_source *source)
{
    if (source->armed.armed)
    {
        bool busy =
            (source->ddr_armed  &&  ddr_target.armed.armed)  ||
            (source->buf_armed  &&  buf_target.armed.armed)  ||
            (source->seq_armed  &&  seq_target.armed.armed);
        if (!busy)
        {
            /* We're not busy, so mark ourself as no longer armed ... but we
             * may also retrigger if appropriate. */
            set_armed_status(&source->armed, false);
            if (source->retrigger)
                arm_and_fire(source);
        }
    }
}


/* Allows all three sources to update their status.  Called periodically. */
static void update_source_status(void)
{
    update_source(&s1_source);
    update_source(&s2_source);
    update_source(&ext_source);
}


#if 0
/* If a trigger does not arrive (should only happen for external trigger), or if
 * a target fails to complete (should only happen for sequencer) put things back
 * as far as possible. */
static void reset_source(struct trigger_source *source)
{
    if (source->armed)
    {
        if (source->ddr_armed)
        /* Turn off triggers as appropriate.  Ideally would also like to reset
         * the arm if possible. */
        if (arm_ddr)    hw_write_ddr_trigger_select(false);
        if (arm_buf)    hw_write_buf_trigger_select(false);
        if (arm_seq)    hw_write_seq_reset();

        source->ddr_armed = false;
        source->buf_armed = false;
        source->seq_armed = false;
        set_source_armed(source, false);
    }
}
#endif

static bool call_reset_source(void *context, const bool *value)
{
    LOCK();
//     reset_source(context);
    UNLOCK();
    return true;
}


static void publish_source(const char *name, struct trigger_source *source)
{
    const char *arm_name =
        source->source_id == TRIG_SOURCE_EXTERNAL ? "ARM" : "FIRE";
    char buffer[20];
#define FORMAT(field) \
    (sprintf(buffer, "TRG:%s:%s", name, field), buffer)

    PUBLISH_WRITE_VAR_P(bo, FORMAT("MODE"), source->retrigger);
    PUBLISH(bo, FORMAT(arm_name), call_arm_and_fire, .context = source);
    PUBLISH(bo, FORMAT("RESET"),  call_reset_source, .context = source);
    publish_status(FORMAT("STATUS"), &source->armed);

#undef FORMAT
}


static void publish_sources(void)
{
    publish_source("S1",  &s1_source);
    publish_source("S2",  &s2_source);
    publish_source("EXT", &ext_source);
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Event processing and polling. */


/* This thread monitors the hardware for events and dispatches them as
 * appropriate.  The following events and states are recognised and updated:
 *  - DDR trigger
 *  - Buffer ready trigger
 *  - Sequencer state and sequencer ready */
static void *monitor_events(void *context)
{
    while (true)
    {
        /* Capture hardware status. */
        bool ddr_ready = poll_ddr_trigger();

        LOCK();

        bool buf_ready = buf_target.armed.armed  &&  !poll_buf_busy();
        bool seq_ready = seq_target.armed.armed  &&  !hw_read_seq_status();

        /* Allow all the corresponding targets to be processed. */
        if (ddr_ready)
            process_ddr_buffer(ddr_target.one_shot);
        if (buf_ready)
            process_fast_buffer();

        /* Clear the busy flags for all targets that have completed. */
        update_target_status(ddr_ready, buf_ready, seq_ready);
        /* Let the trigger sources update their status and maybe retrigger. */
        update_source_status();

        UNLOCK();
        /* Now sleep for a bit; 50ms seems responsive enough (20Hz). */
        usleep(50000);
    }
    return NULL;
}



bool initialise_triggers(void)
{
    publish_sources();
    publish_targets();

    PUBLISH_READER(longin, "TRG:RAWPHASE", hw_read_trg_raw_phase);

    pthread_t thread_id;
    return TEST_0(pthread_create(&thread_id, NULL, monitor_events, NULL));
}
