/* Interface to fast buffer. */

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <pthread.h>

#include "error.h"
#include "hardware.h"
#include "epics_device.h"
#include "epics_extra.h"

#include "buffer.h"


static struct epics_interlock *trigger;
static short buffer_low[BUF_DATA_LENGTH];
static short buffer_high[BUF_DATA_LENGTH];

static bool armed = false;


static void process_trigger(void)
{
    interlock_wait(trigger);
    hw_read_buf_data(buffer_low, buffer_high);
    interlock_signal(trigger, NULL);
}


static bool check_buffer_trigger(void)
{
    bool triggered = armed  &&  !hw_read_buf_status();
    if (triggered)
        armed = false;
    return triggered;
}


/* This loop monitors the buffer for triggers. */
static void *buffer_monitor(void *context)
{
    while (true)
    {
        if (check_buffer_trigger())
            process_trigger();
        usleep(50000);      // 50ms
    }
    return NULL;
}


static void arm_trigger(void)
{
    armed = true;
    hw_write_buf_arm();
}


bool initialise_buffer(void)
{
    trigger = create_interlock("BUF:TRIG", "BUF:DONE", false);
    PUBLISH_WF_READ_VAR(short, "BUF:WFA", BUF_DATA_LENGTH, buffer_low);
    PUBLISH_WF_READ_VAR(short, "BUF:WFB", BUF_DATA_LENGTH, buffer_high);
    PUBLISH_WRITER_P(mbbo, "BUF:SELECT", hw_write_buf_select);
    PUBLISH_WRITER_P(mbbo, "BUF:TRIGSEL", hw_write_buf_trigger_select);

    PUBLISH_ACTION("BUF:ARM", arm_trigger);
    PUBLISH_ACTION("BUF:SOFT_TRIG", hw_write_buf_soft_trigger);

    pthread_t thread_id;
    return TEST_0(pthread_create(&thread_id, NULL, buffer_monitor, NULL));
}
