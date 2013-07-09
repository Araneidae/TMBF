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



/* Total number of turns in the DDR buffer following the trigger.  We have to
 * subtract a couple of turns because of some jitter in the trigger. */
#define BUFFER_TURN_COUNT   (64 * 1024 * 1024 / SAMPLES_PER_TURN / 2 - 2)

/* Size of turn readout waveform. */
#define SHORT_TURN_WF_COUNT       8
#define LONG_TURN_WF_COUNT        256


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* DDR waveform readout records. */

static int selected_bunch = 0;
static int selected_turn = 0;

static struct epics_record *long_waveform;


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


static void publish_waveforms(void)
{
    PUBLISH_WF_ACTION(short, "DDR:BUNCHWF",
        BUFFER_TURN_COUNT, read_ddr_bunch_waveform);
    PUBLISH_WF_ACTION(short, "DDR:SHORTWF",
        SHORT_TURN_WF_COUNT * SAMPLES_PER_TURN, read_short_ddr_turn_waveform);
    long_waveform = PUBLISH_WF_ACTION_I(short, "DDR:LONGWF",
        LONG_TURN_WF_COUNT * SAMPLES_PER_TURN, read_long_ddr_turn_waveform);

    PUBLISH_WRITE_VAR(longout, "DDR:BUNCHSEL", selected_bunch);
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Triggering and control. */

static struct epics_record *short_trigger;   // Triggers short waveform readout

enum trigger_mode { TRIG_ALL, TRIG_ONE_SHOT };
static unsigned int trigger_mode = TRIG_ALL;
/* Don't allow rearming while already armed.  This prevents retriggering until
 * all trigger processing is complete. */
static bool armed = false;

static bool data_ready = false;
static struct epics_record *long_ready;      // Updates ready flag


/* Just writes value to DDR:READY flag. */
static void set_long_data_ready(bool ready)
{
    if (ready != data_ready)
    {
        data_ready = ready;
        trigger_record(long_ready, 0, NULL);
    }
}


/* Enables capture of data to DDR until a trigger is received. */
static void arm_trigger(void)
{
    if (!armed)
    {
        armed = true;
        set_long_data_ready(false);
        pulse_CTRL_ARM_DDR();
    }
}


/* This is called each time the DDR buffer successfully triggers.  The short
 * waveforms are always triggered, but the long waveform is only triggered when
 * we're in one-shot mode. */
static void ddr_trigger(void)
{
    if (trigger_mode == TRIG_ONE_SHOT)
        trigger_record(long_waveform, 0, NULL);
    trigger_record(short_trigger, 0, NULL);
}

/* This is called at the completion of record processing as a direct consequence
 * of triggering short_trigger.  In retriggering mode we rearm, otherwise just
 * report that long data is now ready and enable further triggering. */
static void trigger_done(void)
{
    armed = false;
    if (trigger_mode == TRIG_ALL)
        arm_trigger();
    else
        set_long_data_ready(true);
}


static void set_trigger_mode(unsigned int value)
{
    trigger_mode = value;
    if (trigger_mode == TRIG_ALL)
    {
        set_long_data_ready(false);
        arm_trigger();
    }
}


static void set_selected_turn(int value)
{
    selected_turn = value;
    if (trigger_mode == TRIG_ONE_SHOT)
        trigger_record(long_waveform, 0, NULL);
}

static void arm_callback(void)
{
    arm_trigger();
}


static void publish_controls(void)
{
    PUBLISH_WRITER_P(mbbo, "DDR:TRIGMODE", set_trigger_mode);
    PUBLISH_WRITER(longout, "DDR:TURNSEL", set_selected_turn);

    PUBLISH_READ_VAR(longin, "DDR:TURNSEL", selected_turn);
    long_ready = PUBLISH_READ_VAR_I(bi, "DDR:READY", data_ready);
    short_trigger = PUBLISH_TRIGGER("DDR:TRIG");

    PUBLISH_ACTION("DDR:DONE", trigger_done);
    PUBLISH_ACTION("DDR:ARM", arm_callback);
    PUBLISH_ACTION("DDR:SOFT_TRIG", pulse_CTRL_TRIG_DDR);

    PUBLISH_WRITER(mbbo, "DDR:INPUT", write_CTRL_DDR_INPUT);
}


bool initialise_ddr_epics(void)
{
    publish_waveforms();
    publish_controls();
    return initialise_ddr(ddr_trigger);
}
