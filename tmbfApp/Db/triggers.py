# Common trigger control

from common import *


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Core trigger configuration

def TriggerTarget(name):
    boolOut('TRG:%s:SEL' % name, 'Soft', 'Hardware',
        DESC = '%s trigger source' % name)
    boolOut('TRG:%s:MODE' % name, 'One Shot', 'Retrigger',
        DESC = 'Mode select for %s' % name)
    Action('TRG:%s:ARM' % name, DESC = 'Arm or fire %s' % name)
    longOut('TRG:%s:DELAY' % name, 0, (1<<24)-1, EGU = 'turns',
        DESC = '%s trigger delay' % name)
    Action('TRG:%s:RESET' % name, DESC = 'Reset %s' % name)
    mbbIn('TRG:%s:STATUS' % name, 'Ready', 'Armed', 'Busy',
        SCAN = 'I/O Intr', DESC = '%s status' % name)

# External trigger source configuration.  For each source we have a source
# enable, a blanking enable, and a trigger source status.
def TriggerSources(name, trigger_list):
    set_ext = Action('TRG:%s:SET' % name,
        DESC = 'Update %s trigger source' % name)
    trigger_status = []
    for src, desc in trigger_list:
        boolOut('TRG:%s:%s:EN' % (name, src), 'Ignore', 'Enable',
            FLNK = set_ext, DESC = '%s %s trigger enable' % (name, desc))
        boolOut('TRG:%s:%s:BL' % (name, src), 'All', 'Blanking',
            FLNK = set_ext, DESC = '%s %s blanking' % (name, desc))
        trigger_status.append(
            boolIn('TRG:%s:%s:HIT' % (name, src), 'No', 'Yes',
                ZSV = 'MINOR', DESC = '%s %s trigger source' % (name, desc)))
    Trigger('TRG:%s:HIT' % name, *trigger_status)


# List of all available external trigger sources.  Only the first three are used
# for BUF triggering.
external_triggers = [
    ('EXT', 'External'),
    ('ADC', 'Min/max limit'),
    ('SCLK', 'SCLK input'),
    ('PM', 'Postmortem'),
    ('SEQ', 'Sequencer state')]

# Monitor input status for each trigger source.
input_status = [
    boolIn('TRG:%s:IN' % name, 'No', 'Yes',
        ZSV = 'MINOR', DESC = '%s present' % desc)
    for name, desc in external_triggers]
create_fanout('TRG:IN:FAN',
    Action('TRG:IN', DESC = 'Scan trigger inputs'),
    SCAN = '.2 second', *input_status)


# Trigger source selections for the trigger targets.
TriggerTarget('DDR')
TriggerTarget('BUF')

# Trigger external sources.  For BUF we don't support PM or SEQ triggering
TriggerSources('DDR', external_triggers)
TriggerSources('BUF', external_triggers[:3])


boolOut('TRG:SYNC', 'Separate', 'Synched',
    DESC = 'Synchronise DDR & BUF triggers')


# Sequencer control.
mbbOut('TRG:SEQ:SEL', 'Disabled', 'BUF trigger', 'DDR trigger',
    DESC = 'Sequencer trigger enable')
mbbIn('TRG:SEQ:STATUS', 'Ready', 'Armed', 'Busy',
    SCAN = 'I/O Intr', DESC = 'SEQ status')


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

# Blanking configuration
longOut('TRG:BLANKING', 0, 65535, EGU = 'turns',
    DESC = 'Sequencer blanking window after trigger')
mbbOut('TRG:BLANKING:SOURCE', 'Trigger', 'SCLK Input',
    DESC = 'Configure blanking window source')


# A simple device to toggle the front panel LED at 1Hz to show life.
fp_led = boolOut('FPLED', DESC = 'Front panel LED control')
fp_led_control = records.calcout('FPLED:TOGGLE',
    CALC = '!A', SCAN = '1 second',
    OUT  = PP(fp_led), OOPT = 'Every Time', DOPT = 'Use CALC')
fp_led_control.INPA = fp_led_control


# Trigger monitoring and status.  We show seconds since the last trigger, the
# raw phase bits from the last trigger, and a count of phase discrepancies.
# Age since last trigger
tick = records.calc('TRG:AGE',
    SCAN = '.5 second', CALC = 'A+0.5',
    EGU  = 's', PREC = 1, HIGH = 1,   HSV  = 'MINOR',
    DESC = 'Seconds since last trigger')
tick.INPA = tick

trigger_count = longIn('TRG:COUNT', DESC = 'Total trigger count')
jitter = longIn('TRG:JITTER', DESC = 'Trigger jitter count')
Trigger('TRG:TICK',
    longIn('TRG:RAWPHASE', DESC = 'Raw trigger phase bits'),
    records.calcout('TRG:RESET_TICK',
        CALC = '0', OUT  = tick, OOPT = 'Every Time', DOPT = 'Use CALC',
        DESC = 'Reset trigger count'),
    trigger_count, jitter,
    records.calc('TRG:JITTERPC',
        CALC = 'B?100*A/B:0', EGU = '%', PREC = 3,
        INPA = jitter, INPB = trigger_count, HIGH = 0.001, HSV = 'MAJOR',
        DESC = 'Jitter count percentage'))
Action('TRG:RESET_COUNT', DESC = 'Reset trigger count')
