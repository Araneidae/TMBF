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



/* Records state of a virtual trigger source (soft or external). */
struct trigger_source {
    unsigned int source;        // Source identification
    bool retrigger;             // If auto-retrigger enabled

    /* The following three flags record which events we're waiting for.  Once
     * they all fire we can retrigger. */
    bool ddr_armed;
    bool buf_armed;
    bool seq_armed;

    /* We report as armed until all of the events above have reported firing. */
    bool armed;
    struct epics_record *status_pv;
};


/* Records status of a trigger target. */
struct target_status {
    bool armed;             // If armed and waiting for completion
    struct epics_record *status_pv;
    struct trigger_source *current_source;
};


/* Records state of a trigger target (DDR or fast buffer). */
struct trigger_target {
    struct target_status status;
    /* Identifies physical target. */
    unsigned int source;    // Configured source (invalid for sequencer)
    void (*notify)(void);    // Notifies arming event
};



static struct trigger_source s1_source  = { .source = TRIG_SOURCE_SOFT1 };
static struct trigger_source s2_source  = { .source = TRIG_SOURCE_SOFT2 };
static struct trigger_source ext_source = { .source = TRIG_SOURCE_EXTERNAL };

static struct trigger_target ddr_target = { .notify = arming_ddr_buffer };
static struct trigger_target buf_target;

static struct target_status seq_target;
static bool seq_enable;


/* Some forward declarations as source and target form a dependency loop. */
static bool maybe_arm_target(
    struct trigger_target *target, struct trigger_source *source);

static bool maybe_arm_sequencer(struct trigger_source *source);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Trigger sources (two software, one external). */


/* Updates reported armed status for this trigger source to EPICS. */
static void set_source_armed(struct trigger_source *source, bool armed)
{
    source->armed = armed;
    trigger_record(source->status_pv, 0, NULL);
}


/* This is the starting point for all triggering.  This is called directly by
 * the user or can be invoked when retriggering. */
static void arm_and_fire(struct trigger_source *source)
{
    /* If we're already armed or if we're not actually configured to act on any
     * sources then arming is a no-op. */
    if (source->armed)
        return;

    /* Inform all the possible targets that we're ready. */
    bool arm_ddr = maybe_arm_target(&ddr_target, source);
    bool arm_buf = maybe_arm_target(&buf_target, source);
    bool arm_seq = maybe_arm_sequencer(source);
    if (!arm_ddr  &&  !arm_buf  &&  !arm_seq)
        /* If no takers then there's nothing more to do, don't actually arm
         * at this point. */
        return;

    /* Configure trigger sources we're controlling as appropriate. */
    bool external = source->source == TRIG_SOURCE_EXTERNAL;
    if (arm_ddr)    hw_write_ddr_trigger_select(external);
    if (arm_buf)    hw_write_buf_trigger_select(external);

    /* Hardware now ready, let's go. */
    hw_write_trg_arm(arm_ddr, arm_buf);
    if (!external)
        hw_write_trg_soft_trigger(arm_ddr, arm_buf);

    /* Update reported status and record which reports we expect. */
    source->ddr_armed = arm_ddr;
    source->buf_armed = arm_buf;
    source->seq_armed = arm_seq;
    set_source_armed(source, true);
}

static bool call_arm_and_fire(void *context, const bool *value)
{
    LOCK();
    arm_and_fire(context);
    UNLOCK();
    return true;
}


/* Called when the corresponding target has completed processing. */
static void notify_target_ready(
    struct trigger_source *source, struct target_status *target)
{
    /* Clear the appropriate target flag. */
    if (target == &ddr_target.status  &&  source->ddr_armed)
        source->ddr_armed = false;
    if (target == &buf_target.status  &&  source->buf_armed)
        source->buf_armed = false;
    if (target == &seq_target  &&  source->seq_armed)
        source->seq_armed = false;

    /* Check if we're still waiting for something. */
    bool armed = source->ddr_armed || source->buf_armed || source->seq_armed;
    if (source->armed  &&  !armed)
    {
        set_source_armed(source, false);
        if (source->retrigger)
            arm_and_fire(source);
    }
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
        source->source == TRIG_SOURCE_EXTERNAL ? "ARM" : "FIRE";
    char buffer[20];
#define FORMAT(field) \
    (sprintf(buffer, "TRG:%s:%s", name, field), buffer)

    PUBLISH_WRITE_VAR_P(bo, FORMAT("MODE"), source->retrigger);
    PUBLISH(bo, FORMAT(arm_name), call_arm_and_fire, .context = source);
    PUBLISH(bo, FORMAT("RESET"),  call_reset_source, .context = source);
    source->status_pv = PUBLISH_READ_VAR_I(bi, FORMAT("STATUS"), source->armed);

#undef FORMAT
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Trigger targets (DDR/BUF/SEQ). */

static void set_target_armed(
    struct target_status *status, struct trigger_source *source)
{
    status->current_source = source;
    status->armed = source != NULL;
    trigger_record(status->status_pv, 0, NULL);
}


/* Called by a trigger source to ask whether the target should be armed. */
static bool maybe_arm_target(
    struct trigger_target *target, struct trigger_source *source)
{
    if (source->source == target->source)
    {
        set_target_armed(&target->status, source);
        if (target->notify)
            target->notify();
        return true;
    }
    else
        return false;
}


static bool maybe_arm_sequencer(struct trigger_source *source)
{
    if (source->source == buf_target.source)
    {
        /* The associated trigger will have been armed so need to enable or
         * disable the sequencer as appropriate. */
        set_target_armed(&seq_target, seq_enable ? source : NULL);
        enable_seq_trigger(seq_enable);
        return seq_enable;
    }
    else
        return false;
}


/* Called when the corresponding target is armed and has successfully completed
 * processing.  Updates its own status and passes the readiness indication up to
 * the corresponding trigger source. */
static void process_target_ready(struct target_status *status)
{
    if (status->current_source)
    {
        /* Let the source know we're done.  Note that we might get retriggered
         * as part of the notify call! */
        struct trigger_source *source = status->current_source;
        set_target_armed(status, NULL);
        notify_target_ready(source, status);
    }
}


static void publish_status(const char *name, struct target_status *status)
{
    char buffer[20];
#define FORMAT(field) \
    (sprintf(buffer, "TRG:%s:%s", name, field), buffer)

    status->status_pv = PUBLISH_READ_VAR_I(bi, FORMAT("STATUS"), status->armed);

#undef FORMAT
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Event processing and polling. */


/* Determines whether the trigger was for a one-shot trigger: the DDR
 * processing code cares about this detail. */
static bool is_one_shot(struct target_status *target)
{
    return target->current_source  &&  !target->current_source->retrigger;
}


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

        bool buf_ready = buf_target.status.armed  &&  !poll_buf_busy();
        bool seq_ready = seq_target.armed  &&  !hw_read_seq_status();

        /* Allow all the corresponding targets to be processed. */
        if (ddr_ready)
            process_ddr_buffer(is_one_shot(&ddr_target.status));
        if (buf_ready)
            process_fast_buffer();

        /* Retrigger as appropriate.  Note that this must come after target
         * trigger processing to avoid loss of data. */
        if (ddr_ready)
            process_target_ready(&ddr_target.status);
        if (buf_ready)
            process_target_ready(&buf_target.status);
        if (seq_ready)
            process_target_ready(&seq_target);

        UNLOCK();
        /* Now sleep for a bit; 50ms seems responsive enough (20Hz). */
        usleep(50000);
    }
    return NULL;
}



bool initialise_triggers(void)
{
    publish_source("S1",  &s1_source);
    publish_source("S2",  &s2_source);
    publish_source("EXT", &ext_source);

    publish_status("DDR", &ddr_target.status);
    publish_status("BUF", &buf_target.status);
    publish_status("SEQ", &seq_target);
    PUBLISH_WRITE_VAR_P(mbbo, "TRG:DDR:SEL", ddr_target.source);
    PUBLISH_WRITE_VAR_P(mbbo, "TRG:BUF:SEL", buf_target.source);
    PUBLISH_WRITE_VAR_P(bo,   "TRG:SEQ:ENA", seq_enable);

    pthread_t thread_id;
    return TEST_0(pthread_create(&thread_id, NULL, monitor_events, NULL));
}
