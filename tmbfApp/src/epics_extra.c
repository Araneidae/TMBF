/* Extra support built on top of epics_device. */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <errno.h>
#include <pthread.h>
#include <string.h>

#include <initHooks.h>

#include "error.h"
#include "epics_device.h"

#include "epics_extra.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* EPICS Interlock */


/* An epics_interlock object contains a mutex for interlocking with database
 * processing and a record to trigger processing.  To allow early initialisation
 * there is also support for blocking until EPICS initialisation is done. */
struct epics_interlock {
    struct epics_interlock *next;   // For ready notification
    struct epics_record *trigger;   // Triggers EPICS database processing
    pthread_mutex_t mutex;          // Locked when EPICS busy
};

#define LOCK_INTERLOCK(interlock) \
    ASSERT_0(pthread_mutex_lock(&(interlock)->mutex))
#define UNLOCK_INTERLOCK(interlock) \
    ASSERT_0(pthread_mutex_unlock(&(interlock)->mutex))


/* If EPICS isn't ready yet then we need to block all callers until it is. */
static bool epics_ready = false;
static pthread_mutex_t epics_ready_mutex = PTHREAD_MUTEX_INITIALIZER;
static struct epics_interlock *notify_list = NULL;

#define LOCK_READY()    ASSERT_0(pthread_mutex_lock(&epics_ready_mutex))
#define UNLOCK_READY()  ASSERT_0(pthread_mutex_unlock(&epics_ready_mutex))


void interlock_wait(struct epics_interlock *interlock)
{
    LOCK_INTERLOCK(interlock);
}

void interlock_signal(struct epics_interlock *interlock, struct timespec *ts)
{
    trigger_record(interlock->trigger, 0, ts);
}

/* Completion of EPICS processing chain, allow waiting thread to proceed.
 * To be called by EPICS database at end of triggered processing chain. */
static bool interlock_done(void *context, const bool *value)
{
    struct epics_interlock *interlock = context;
    UNLOCK_INTERLOCK(interlock);
    return true;
}

struct epics_interlock *create_interlock(const char *base_name, bool set_time)
{
    struct epics_interlock *interlock = malloc(sizeof(struct epics_interlock));
    interlock->next = NULL;

    char buffer[strlen(base_name) + strlen(":TRIG") + 1];
    sprintf(buffer, "%s:TRIG", base_name);
    interlock->trigger = PUBLISH(bi, buffer,
        .read = _publish_trigger_bi, .io_intr = true, .set_time = set_time);
    pthread_mutex_init(&interlock->mutex, NULL);
    sprintf(buffer, "%s:DONE", base_name);
    PUBLISH(bo, buffer, .write = interlock_done, .context = interlock);

    LOCK_READY();
    if (!epics_ready)
    {
        /* Start with the mutex locked if EPICS hasn't started yet and chain
         * onto the list of pending interlocks, this will be unlocked once EPICS
         * ready is signalled. */
        LOCK_INTERLOCK(interlock);
        interlock->next = notify_list;
        notify_list = interlock;
    }
    UNLOCK_READY();
    return interlock;
}


void wait_for_epics_start(void)
{
    /* This is very naughty code.  We fake up a temporary interlock object just
     * so we can go onto the initialisation list.  Still, it will serve... */
    struct epics_interlock interlock = { .mutex = PTHREAD_MUTEX_INITIALIZER };

    LOCK_READY();
    bool lock_needed = !epics_ready;
    if (lock_needed)
    {
        interlock.next = notify_list;
        notify_list = &interlock;
        LOCK_INTERLOCK(&interlock);
    }
    UNLOCK_READY();

    if(lock_needed)
    {
        LOCK_INTERLOCK(&interlock);
        UNLOCK_INTERLOCK(&interlock);
        pthread_mutex_destroy(&interlock.mutex);
    }
}


/* Called during EPICS initialisation so we can detect completion. */
static void init_hook(initHookState state)
{
    /* Once EPICS initialisation completes signal all waiting interlocks. */
    if (state == initHookAtEnd)
    {
        LOCK_READY();
        epics_ready = true;
        while (notify_list)
        {
            /* Because of the naughty implementation of wait_for_epics_start()
             * it's possible that the current head will evaporate as soon as we
             * touch it, so grab its next pointer first for safety. */
            struct epics_interlock *next = notify_list->next;
            UNLOCK_INTERLOCK(notify_list);
            notify_list = next;
        }
        UNLOCK_READY();
    }
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* IN records with associated value. */

#ifndef __BIGGEST_ALIGNMENT__
#define __BIGGEST_ALIGNMENT__   8
#endif

struct in_epics_record_ {
    enum record_type record_type;
    struct epics_record *record;
    int field_size;
    bool force_update;
    char value[] __attribute__((aligned(__BIGGEST_ALIGNMENT__)));
};

_DECLARE_IN_ARGS_(void, void);

static size_t record_field_size(enum record_type record_type)
{
    switch (record_type)
    {
        case RECORD_TYPE_longin:    return sizeof(TYPEOF(longin));
        case RECORD_TYPE_ulongin:   return sizeof(TYPEOF(ulongin));
        case RECORD_TYPE_ai:        return sizeof(TYPEOF(ai));
        case RECORD_TYPE_bi:        return sizeof(TYPEOF(bi));
        case RECORD_TYPE_stringin:  return sizeof(TYPEOF(stringin));
        case RECORD_TYPE_mbbi:      return sizeof(TYPEOF(mbbi));
        default: ASSERT_FAIL();
    }
}

static bool read_in_record(void *context, void *value)
{
    struct in_epics_record_ *record = context;
    memcpy(value, record->value, record->field_size);
    return true;
}

struct in_epics_record_ *_publish_write_epics_record(
    enum record_type record_type, const char *name,
    const struct publish_in_epics_record_args *args)
{
    int field_size = record_field_size(record_type);
    struct in_epics_record_ *record = malloc(
        sizeof(struct in_epics_record_) + field_size);
    record->record_type = record_type;
    record->field_size = field_size;
    record->record = publish_epics_record(
        record_type, name, &(const struct record_args_void) {
            .read = read_in_record, .context = record,
            .io_intr = args->io_intr, .set_time = args->set_time });
    record->force_update = args->force_update;
    memset(record->value, 0, record->field_size);
    return record;
}


void _write_in_record(
    enum record_type record_type, struct in_epics_record_ *record,
    const void *value, const struct write_in_epics_record_args *args)
{
    ASSERT_OK(record->record_type == record_type);
    bool do_update = record->force_update;
    if (value)
    {
        do_update = do_update  ||  args->force_update  ||
            memcmp(record->value, value, record->field_size) != 0;
        memcpy(record->value, value, record->field_size);
    }
    if (do_update)
        trigger_record(record->record, args->severity, args->timestamp);
}


void *_read_in_record(
    enum record_type record_type, struct in_epics_record_ *record)
{
    ASSERT_OK(record->record_type == record_type);
    return record->value;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

bool initialise_epics_extra(void)
{
    initHookRegister(init_hook);
    return true;
}
