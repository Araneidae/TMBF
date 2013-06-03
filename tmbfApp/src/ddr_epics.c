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



/* Total number of turns in the DDR buffer following the trigger. */
#define BUFFER_TURN_COUNT   (16 * 1024 * 1024 / SAMPLES_PER_TURN / 2)

/* Size of turn readout waveform. */
#define SHORT_TURN_WF_COUNT       8
#define LONG_TURN_WF_COUNT        256


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* DDR waveform readout records. */

static int selected_bunch = 0;
static int selected_turn = 0;



static void read_ddr_bunch_waveform(short *waveform)
{
    read_ddr_bunch(0, selected_bunch, BUFFER_TURN_COUNT, waveform);
}

static void read_short_ddr_turn_waveform(short *waveform)
{
    read_ddr_turns(0, SHORT_TURN_WF_COUNT, waveform);
}

static void read_long_ddr_turn_waveform(short *waveform)
{
    read_ddr_turns(selected_turn, LONG_TURN_WF_COUNT, waveform);
}

PUBLISH_SIMPLE_WAVEFORM(short, "DDR:BUNCHWF",
    BUFFER_TURN_COUNT, read_ddr_bunch_waveform)
PUBLISH_SIMPLE_WAVEFORM(short, "DDR:SHORTWF",
    SHORT_TURN_WF_COUNT * SAMPLES_PER_TURN, read_short_ddr_turn_waveform)
PUBLISH_SIMPLE_WAVEFORM(short, "DDR:LONGWF",
    LONG_TURN_WF_COUNT * SAMPLES_PER_TURN, read_long_ddr_turn_waveform,
    .io_intr = true)

PUBLISH_VARIABLE_WRITE(longout, "DDR:BUNCHSEL", selected_bunch)



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Triggering and control. */

static struct record_base *short_trigger;   // Triggers short waveform readout
static struct record_base *long_trigger;    // Triggers long waveform readout

enum trigger_mode { TRIG_ALL, TRIG_ONE_SHOT };
static unsigned int trigger_mode = TRIG_ALL;

static bool long_data_ready = false;        // Used to report long wf ready
static struct record_base *long_ready;      // Updates ready flag


/* Just writes value to DDR:READY flag. */
static void set_long_data_ready(bool ready)
{
    if (ready != long_data_ready)
    {
        long_data_ready = ready;
        SignalRecord(long_ready);
    }
}


/* Enables capture of data to DDR until a trigger is received. */
static void arm_trigger(void)
{
    set_long_data_ready(false);
    pulse_CTRL_ARM_DDR();
}


/* This is called each time the DDR buffer successfully triggers.  The short
 * waveforms are always triggered, but the long waveform is only triggered when
 * we're in one-shot mode. */
static void ddr_trigger(void)
{
    if (trigger_mode == TRIG_ONE_SHOT)
        SignalRecord(long_trigger);
    SignalRecord(short_trigger);
}

/* This is called at the completion of record processing.  In retriggering mode
 * we rearm, otherwise just report that long data is now ready. */
static void trigger_done(void)
{
    if (trigger_mode == TRIG_ALL)
        arm_trigger();
    else
        set_long_data_ready(true);
}


static void set_trigger_mode(void)
{
    if (trigger_mode == TRIG_ALL)
    {
        set_long_data_ready(false);
        arm_trigger();
    }
}


static void set_selected_turn(void)
{
    if (trigger_mode == TRIG_ONE_SHOT)
        SignalRecord(long_trigger);
}


PUBLISH_VARIABLE_WRITE_ACTION(
    mbbo, "DDR:TRIGMODE", trigger_mode, set_trigger_mode, .persist = true)

PUBLISH_VARIABLE_WRITE_ACTION(
    longout, "DDR:TURNSEL", selected_turn, set_selected_turn)
PUBLISH_VARIABLE_READ(longin, "DDR:TURNSEL",  selected_turn)

PUBLISH_VARIABLE_READ(bi, "DDR:READY", long_data_ready, .io_intr = true)

PUBLISH_TRIGGER("DDR:TRIG")
PUBLISH_METHOD("DDR:TRIGDONE", trigger_done)

PUBLISH_METHOD("DDR:ARM", arm_trigger)

PUBLISH_REGISTER_P(mbbo, "DDR:INPUT", CTRL_DDR_INPUT)
PUBLISH_METHOD("DDR:SOFT_TRIG", pulse_CTRL_TRIG_DDR)




#ifndef __DEFINE_EPICS__
#include "ddr_epics.EPICS"
#endif


bool initialise_ddr_epics(void)
{
    return
        initialise_ddr(ddr_trigger)  &&
        PUBLISH_EPICS_DATA()  &&
        TEST_NULL(short_trigger = LOOKUP_RECORD(bi, "DDR:TRIG"))  &&
        TEST_NULL(long_ready    = LOOKUP_RECORD(bi, "DDR:READY"))  &&
        TEST_NULL(long_trigger  = LOOKUP_RECORD(waveform, "DDR:LONGWF"));
}
