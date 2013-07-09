/* Extra support built on top of epics_device. */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <errno.h>
#include <pthread.h>

#include <initHooks.h>

#include "error.h"
#include "epics_device.h"

#include "epics_extra.h"


/* An epics_interlock object contains a mutex for interlocking with database
 * processing and a record to trigger processing.  To allow early initialisation
 * there is also support for blocking until EPICS initialisation is done. */
struct epics_interlock {
    struct epics_interlock *next;   // For ready notification
    struct epics_record *trigger;   // Triggers EPICS database processing
    pthread_mutex_t mutex;          // Locked when EPICS busy
};

#define LOCK_INTERLOCK(interlock) \
    ASSERT_0(pthread_mutex_lock(&interlock->mutex))
#define UNLOCK_INTERLOCK(interlock) \
    ASSERT_0(pthread_mutex_unlock(&interlock->mutex))


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

struct epics_interlock *create_interlock(
    const char *trigger_name, const char *done_name, bool set_time)
{
    struct epics_interlock *interlock = malloc(sizeof(struct epics_interlock));
    interlock->next = NULL;
    interlock->trigger = PUBLISH(bi, trigger_name,
        .read = _publish_trigger_bi, .io_intr = true, .set_time = set_time);
    pthread_mutex_init(&interlock->mutex, NULL);
    PUBLISH(bo, done_name, .write = interlock_done, .context = interlock);

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


/* Called during EPICS initialisation so we can detect completion. */
static void init_hook(initHookState state)
{
    /* Once EPICS initialisation completes signal all waiting interlocks. */
    if (state == initHookAtEnd)
    {
        LOCK_READY();
        epics_ready = true;
        for ( ; notify_list; notify_list = notify_list->next)
            UNLOCK_INTERLOCK(notify_list);
        UNLOCK_READY();
    }
}

bool initialise_epics_extra(void)
{
    initHookRegister(init_hook);
    return true;
}
