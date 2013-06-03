/* EPICS interface to DDR buffer. */

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>

#include "error.h"
#include "ddr.h"
#include "hardware.h"
#include "epics_device.h"

#include "ddr_epics.h"


/* Need to share these with device.c */
#define PUBLISH_REGISTER_P(record, name, register) \
    PUBLISH_SIMPLE_WRITE_INIT( \
        record, name, write_##register, read_##register, .persist = true)
#define PUBLISH_PULSE(name, register) \
    PUBLISH_METHOD(name, pulse_##register)



/* Total number of turns in the DDR buffer. */
#define BUFFER_TURN_COUNT   (16 * 1024 * 1024 / SAMPLES_PER_TURN)

/* Size of turn readout waveform. */
#define TURN_WF_COUNT       256

static struct record_base *trigger;

static int selected_bunch = 0;
static int selected_turn = 0;
enum trigger_mode { TRIG_ALL, TRIG_ONE_SHOT };
static unsigned int trigger_mode = TRIG_ALL;


static void read_ddr_bunch_waveform(short *waveform)
{
    read_ddr_bunch(
        -BUFFER_TURN_COUNT/2, selected_bunch, BUFFER_TURN_COUNT, waveform);
}


static void read_ddr_turn_waveform(short *waveform)
{
    read_ddr_turns(selected_turn, TURN_WF_COUNT, waveform);
}

PUBLISH_SIMPLE_WAVEFORM(short, "DDR:BUNCHWF",
    BUFFER_TURN_COUNT, read_ddr_bunch_waveform)
PUBLISH_SIMPLE_WAVEFORM(short, "DDR:TURNWF",
    TURN_WF_COUNT * SAMPLES_PER_TURN, read_ddr_turn_waveform)

static void update_trigger_mode(void)
{
    printf("trgger_mode: %d\n", trigger_mode);
}

static bool on_trigger(void *context, bool *result)
{
    *result = true;
    return true;
}



/* This is called each time the DDR buffer successfully triggers. */
static void ddr_trigger(void)
{
    SignalRecord(trigger);
}


/* This is called at the completion of record processing. */
static void trigger_done(void)
{
    if (trigger_mode == TRIG_ALL)
        pulse_CTRL_ARM_DDR();
}


PUBLISH_VARIABLE_WRITE_ACTION(
    mbbo, "DDR:TRIGMODE", trigger_mode, update_trigger_mode, .persist = true)
PUBLISH_VARIABLE_WRITE(longout, "DDR:BUNCHSEL", selected_bunch)
PUBLISH_VARIABLE_WRITE(longout, "DDR:TURNSEL",  selected_turn)

PUBLISH(bi, "DDR:TRIG", on_trigger, .io_intr = true)
PUBLISH_METHOD("DDR:TRIGDONE", trigger_done)

PUBLISH_REGISTER_P(mbbo, "DDR:INPUT", CTRL_DDR_INPUT)
PUBLISH_PULSE("DDR:ARM",       CTRL_ARM_DDR)
PUBLISH_PULSE("DDR:SOFT_TRIG", CTRL_TRIG_DDR)




#ifndef __DEFINE_EPICS__
#include "ddr_epics.EPICS"
#endif


bool initialise_ddr_epics(void)
{
    return
        initialise_ddr(ddr_trigger)  &&
        PUBLISH_EPICS_DATA()  &&
        TEST_NULL(trigger = LookupRecord("DDR:TRIG"));
}
