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
#include "epics_extra.h"
#include "ddr.h"
#include "ddr_epics.h"
#include "sequencer.h"

#include "triggers.h"


/* Need to serialise some of our operations. */
static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;

#define LOCK()      pthread_mutex_lock(&lock)
#define UNLOCK()    pthread_mutex_unlock(&lock)



/* Used for reporting armed status. */
struct armed_status {
    bool armed;
    struct epics_record *status_pv;
};

/* Records state of a trigger target. */
struct trigger_target {
    enum { DDR_TARGET, BUF_TARGET } target_id;
    struct armed_status status;

    // Configuration settings set by EPICS
    bool external;          // Configured source, written from PV
    bool auto_rearm;

    enum {
        STATE_NORMAL,       // Normal triggering
        STATE_STOPPED,      // Explicitly stopped, don't auto re-arm
        STATE_ARMED,        // Explicitly rearmed but companion busy
    } state;
};

/* The sequencer is a little different as it shares triggering with the
 * buffer, but has its own enable state. */
struct seq_target {
    struct armed_status status;
    bool enabled;               // Updated from PV
};


static struct trigger_target ddr_target = { .target_id = DDR_TARGET };
static struct trigger_target buf_target = { .target_id = BUF_TARGET };
static struct seq_target seq_target;

/* Whether DDR and BUF triggers should be armed and fired together.  This
 * affects both arm/fire and rearming: if this flag is set, arm/fire will fire
 * both triggers and rearming will wait for both targets to complete, but only
 * if both targets agree on their trigger source. */
static bool synchronise_triggers;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* Updates armed status and generates PV update if necessary. */
static void set_armed_status(struct armed_status *status, bool armed)
{
    if (armed != status->armed)
    {
        status->armed = armed;
        trigger_record(status->status_pv, 0, NULL);
    }
}


/* Computes whether this target should be armed. */
static bool check_arm_target(
    struct trigger_target *self, struct trigger_target *target,
    bool external, bool auto_arm)
{
    bool arm =
        target->external == external  &&  !target->status.armed  &&
        (synchronise_triggers  ||  self == target);
    if (auto_arm)
        /* Auto arming behaviour depends on the trigger target's state. */
        switch (target->state)
        {
            case STATE_NORMAL:  return arm  &&  target->auto_rearm;
            case STATE_STOPPED: return false;
            case STATE_ARMED:   return arm;
            default:            ASSERT_FAIL();
        }
    else
        return arm;
}

static void do_arm_ddr(void)
{
    arming_ddr_buffer();    // Let software know DDR buffer is active
    hw_write_ddr_enable();  // Initiate capture of selected DDR data

    set_armed_status(&ddr_target.status, true);
    ddr_target.state = STATE_NORMAL;
}

static void do_arm_buf(void)
{
    set_armed_status(&seq_target.status, seq_target.enabled);
    prepare_sequencer(seq_target.enabled);

    set_armed_status(&buf_target.status, true);
    buf_target.state = STATE_NORMAL;
}

static void arm_and_fire(bool arm_ddr, bool arm_buf, bool external)
{
    /* Prepare the targets. */
    if (arm_ddr)
        do_arm_ddr();
    if (arm_buf)
        do_arm_buf();

    /* Fire! */
    if (external)
        hw_write_trg_arm(arm_ddr, arm_buf);
    else
        hw_write_trg_soft_trigger(arm_ddr, arm_buf);
}

static void arm_target(struct trigger_target *target, bool auto_arm)
{
    /* There are two arming targets, DDR and BUF, and target is one of them.  If
     * synchronise_triggers is set then we need to take the other target into
     * account, otherwise we can just rearm ourself. */
    bool external = target->external;
    bool arm_ddr = check_arm_target(target, &ddr_target, external, auto_arm);
    bool arm_buf = check_arm_target(target, &buf_target, external, auto_arm);

    /* Figure out whether we can accept this arming request: only if all users
     * of the source are currently idle.  If we're not in synchronise_triggers
     * mode we ignore the status of the other target. */
    bool targets_busy =
        synchronise_triggers  &&
        ((ddr_target.external == external  &&  ddr_target.status.armed)  ||
         (buf_target.external == external  &&  buf_target.status.armed));
    if (targets_busy)
    {
        /* Configure this target for later re-arming if it's armable. */
        if (check_arm_target(target, target, external, auto_arm))
            target->state = STATE_ARMED;
    }
    else
        arm_and_fire(arm_ddr, arm_buf, external);
}


/* To stop a target it's enough to disarm it and block further rearming. */
static void stop_target(struct trigger_target *target)
{
    hw_write_trg_disarm(
        target->target_id == DDR_TARGET,
        target->target_id == BUF_TARGET);

    target->state = STATE_STOPPED;
    set_armed_status(&target->status, false);
}


/* When changing the trigger source need to stop the trigger because otherwise
 * the new source won't be processed. */
static void set_source(struct trigger_target *target, bool value)
{
    stop_target(target);
    target->external = value;
}


/* Called as part of the event polling loop.  The ready flags indicate
 * transition from busy to ready and so represent edge triggered events. */
static void update_target_status(bool ddr_ready, bool buf_ready, bool seq_ready)
{
    /* First update the status of each target. */
    if (ddr_ready)  set_armed_status(&ddr_target.status, false);
    if (buf_ready)  set_armed_status(&buf_target.status, false);
    if (seq_ready)  set_armed_status(&seq_target.status, false);

    /* Trigger rearming as appropriate. */
    if (ddr_ready)  arm_target(&ddr_target, true);
    if (buf_ready)  arm_target(&buf_target, true);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* DDR external trigger source control and monitoring. */

/* Configured DDR trigger source enables. */
static bool ddr_trigger_source[DDR_SOURCE_COUNT];
/* Blanking enables for each trigger source. */
static bool ddr_trigger_blanking[DDR_SOURCE_COUNT];
/* Polled input status for each trigger source. */
static bool ddr_trigger_in[DDR_SOURCE_COUNT];
/* Source of last trigger when DDR trigger fired. */
static bool ddr_trigger_hit[DDR_SOURCE_COUNT];

/* Used to update trigger source on a trigger hit. */
static struct epics_interlock *ddr_hit_update;


/* Called each time any of the configuration bits is changed. */
static void set_ddr_trigger_source(void)
{
    hw_write_trg_ddr_source(ddr_trigger_source);
    hw_write_trg_ddr_blanking(ddr_trigger_blanking);
}

/* Polled to read the state of the trigger sources. */
static void update_ddr_trigger_in(void)
{
    static int const trigger_bits[DDR_SOURCE_COUNT] = {
        TRIGGER_TRG_IN,
        TRIGGER_PM_IN,
        TRIGGER_ADC_IN,
        TRIGGER_SEQ_IN,
        TRIGGER_SCLK_IN,
    };
    bool pulsed_bits[PULSED_BIT_COUNT];
    bool read_bits[PULSED_BIT_COUNT] = { };
    for (int i = 0; i < DDR_SOURCE_COUNT; i ++)
        read_bits[trigger_bits[i]] = true;
    hw_read_pulsed_bits(read_bits, pulsed_bits);

    interlock_wait(ddr_hit_update);
    for (int i = 0; i < DDR_SOURCE_COUNT; i ++)
        ddr_trigger_in[i] = pulsed_bits[trigger_bits[i]];
    interlock_signal(ddr_hit_update, NULL);
}

static void update_ddr_trigger_hit(void)
{
    hw_read_trg_ddr_source(ddr_trigger_hit);
}

static void publish_ddr_external(void)
{
    /* These five trigger names need to match the five trigger sources defined
     * in triggers.py and in the hardware interface. */
    static const char *const names[DDR_SOURCE_COUNT] = {
        "EXT", "PM", "ADC", "SEQ", "SCLK" };

    for (int ix = 0; ix < DDR_SOURCE_COUNT; ix ++)
    {
        char buffer[40];
#define FORMAT(field) \
        (sprintf(buffer, "TRG:DDR:%s:%s", names[ix], field), buffer)

        PUBLISH_WRITE_VAR_P(bo, FORMAT("EN"), ddr_trigger_source[ix]);
        PUBLISH_WRITE_VAR_P(bo, FORMAT("BL"), ddr_trigger_blanking[ix]);
        PUBLISH_READ_VAR(bi, FORMAT("IN"), ddr_trigger_in[ix]);
        PUBLISH_READ_VAR(bi, FORMAT("HIT"), ddr_trigger_hit[ix]);
#undef FORMAT
    }

    PUBLISH_ACTION("TRG:DDR:SET", set_ddr_trigger_source);
    PUBLISH_ACTION("TRG:DDR:IN", update_ddr_trigger_in);
    ddr_hit_update = create_interlock(
        "TRG:DDR:HIT:TRIG", "TRG:DDR:HIT:DONE", false);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* EPICS interface to triggers. */


/* Initialises armed_status structure including publishing of associated PV. */
static void publish_status(const char *name, struct armed_status *status)
{
    status->armed = false;
    status->status_pv = PUBLISH_READ_VAR_I(bi, name, status->armed);
}

static bool call_set_source(void *context, const bool *value)
{
    LOCK();
    set_source(context, *value);
    UNLOCK();
    return true;
}

static bool call_arm_target(void *context, const bool *value)
{
    LOCK();
    arm_target(context, false);
    UNLOCK();
    return true;
}

static bool call_stop_target(void *context, const bool *value)
{
    LOCK();
    stop_target(context);
    UNLOCK();
    return true;
}

static void publish_target(struct trigger_target *target, const char *name)
{
    char buffer[20];
#define FORMAT(pv) \
    (sprintf(buffer, "TRG:%s:%s", name, pv), buffer)

    publish_status(FORMAT("STATUS"), &target->status);
    PUBLISH(bo, FORMAT("SEL"), call_set_source,
        .context = target, .persist = true);
    PUBLISH_WRITE_VAR_P(bo,      FORMAT("MODE"),  target->auto_rearm);
    PUBLISH(bo, FORMAT("ARM"),   call_arm_target,  .context = target);
    PUBLISH(bo, FORMAT("RESET"), call_stop_target, .context = target);

#undef FORMAT
}


static void publish_targets(void)
{
    publish_target(&ddr_target, "DDR");
    PUBLISH_WRITER_P(ulongout, "TRG:DDR:DELAY", hw_write_trg_ddr_delay);
    publish_ddr_external();

    publish_target(&buf_target, "BUF");
    PUBLISH_WRITER_P(ulongout, "TRG:BUF:DELAY", hw_write_trg_buf_delay);

    publish_status("TRG:SEQ:STATUS", &seq_target.status);
    PUBLISH_WRITE_VAR_P(bo, "TRG:SEQ:ENA", seq_target.enabled);

    PUBLISH_WRITE_VAR_P(bo, "TRG:SYNC", synchronise_triggers);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Event processing and polling. */

static struct epics_interlock *trigger_tick;
static int last_trig_phase;
static int raw_trig_phase;
static int trigger_count;
static int jitter_count;

/* Reset trigger and jitter counts. */
static void reset_trigger_count(void)
{
    trigger_count = 0;
    jitter_count = 0;
}

/* Probes the trigger phase so that we can observe the frequency of external
 * triggers.  We also watch for and record phase changes. */
static void poll_trigger_phase(void)
{
    int phase = hw_read_trg_raw_phase();
    if (phase != 0)
    {
        hw_write_trg_arm_raw_phase();
        interlock_wait(trigger_tick);
        raw_trig_phase = phase;
        trigger_count += 1;
        if (last_trig_phase  &&  last_trig_phase != raw_trig_phase)
            jitter_count += 1;
        interlock_signal(trigger_tick, NULL);

        last_trig_phase = raw_trig_phase;
    }
}


/* This thread monitors the hardware for events and dispatches them as
 * appropriate.  The following events and states are recognised and updated:
 *  - DDR trigger
 *  - Buffer ready trigger
 *  - Sequencer state and sequencer ready */
static void *monitor_events(void *context)
{
    wait_for_epics_start();

    while (true)
    {
        LOCK();

        int ddr_offset;
        bool ddr_ready =
            ddr_target.status.armed  &&  !hw_read_ddr_status(&ddr_offset);
        bool buf_ready = buf_target.status.armed  &&  !hw_read_buf_busy();
        bool seq_ready = seq_target.status.armed  &&  !hw_read_seq_status();

        /* Allow all the corresponding targets to be processed. */
        if (ddr_ready)
        {
            update_ddr_trigger_hit();
            set_ddr_offset(ddr_offset);
            process_ddr_buffer(!ddr_target.auto_rearm);
        }
        if (buf_ready)
            process_fast_buffer();

        /* Clear the busy flags for all targets that have completed and handle
         * rearming as appropriate. */
        update_target_status(ddr_ready, buf_ready, seq_ready);

        /* Update trigger phase. */
        poll_trigger_phase();

        UNLOCK();
        /* Now sleep for a bit; 50ms seems responsive enough (20Hz). */
        usleep(50000);
    }
    return NULL;
}



bool initialise_triggers(void)
{
    publish_targets();

    trigger_tick = create_interlock("TRG:TICK:TRIG", "TRG:TICK:DONE", false);
    PUBLISH_READ_VAR(longin, "TRG:RAWPHASE", raw_trig_phase);
    PUBLISH_READ_VAR(longin, "TRG:COUNT", trigger_count);
    PUBLISH_READ_VAR(longin, "TRG:JITTER", jitter_count);
    PUBLISH_ACTION("TRG:RESET_COUNT", reset_trigger_count);

    PUBLISH_WRITER(bo, "FPLED", hw_write_front_panel_led);
    PUBLISH_WRITER_P(ulongout, "TRG:BLANKING", hw_write_trg_blanking);
    PUBLISH_WRITER_P(mbbo, "TRG:BLANKING:SOURCE", hw_write_trg_blanking_source);

    pthread_t thread_id;
    return TEST_0(pthread_create(&thread_id, NULL, monitor_events, NULL));
}
