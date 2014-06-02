# Common trigger control

from common import *


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Core trigger configuration

def TriggerTarget(name):
    boolOut('TRG:%s:SEL' % name, 'Soft', 'External',
        DESC = '%s trigger source' % name)
    boolOut('TRG:%s:MODE' % name, 'One Shot', 'Retrigger',
        DESC = 'Mode select for %s' % name)
    Action('TRG:%s:ARM' % name, DESC = 'Arm or fire %s' % name)
    longOut('TRG:%s:DELAY' % name, 0, (1<<24)-1, EGU = 'turns',
        DESC = '%s trigger delay' % name)
    Action('TRG:%s:RESET' % name, DESC = 'Reset %s' % name)
    boolIn('TRG:%s:STATUS' % name, 'Ready', 'Busy',
        SCAN = 'I/O Intr', DESC = '%s status' % name)

# Trigger source selections for the trigger targets.
TriggerTarget('DDR')
TriggerTarget('BUF')

boolOut('TRG:SYNC', 'Separate', 'Synched',
    DESC = 'Synchronise DDR & BUF triggers')

# External trigger source configuration for DDR trigger.  For each source we
# have a source enable, a blanking enable, an active status, and a trigger
# source status.
set_ddr_ext = Action('TRG:DDR:SET', DESC = 'Update DDR trigger source')
external_triggers = [
    ('EXT', 'External'),
    ('PM', 'Postmortem'),
    ('ADC', 'Min/max limit'),
    ('SEQ', 'Sequencer state'),
    ('SCLK', 'SCLK input')]
input_status = []
trigger_status = []
for name, desc in external_triggers:
    boolOut('TRG:DDR:%s:EN' % name, 'Ignore', 'Enable', FLNK = set_ddr_ext,
        DESC = '%s trigger enable' % desc)
    boolOut('TRG:DDR:%s:BL' % name, 'Off', 'Blanking', FLNK = set_ddr_ext,
        DESC = '%s blanking' % desc)
    input_status.append(
        boolIn('TRG:DDR:%s:IN' % name, 'No', 'Yes',
            ZSV = 'MINOR', DESC = '%s present' % desc))
    trigger_status.append(
        boolIn('TRG:DDR:%s:HIT' % name, 'No', 'Yes',
            ZSV = 'MINOR', DESC = '%s trigger source' % desc))
create_fanout('TRG:DDR:FAN',
    Action('TRG:DDR:IN'), SCAN = '.2 second', *input_status)
Trigger('TRG:DDR:HIT', *trigger_status)




# Sequencer control.
boolOut('TRG:SEQ:ENA', 'Disabled', 'Enabled', DESC = 'Sequencer trigger enable')
boolIn('TRG:SEQ:STATUS', 'Ready', 'Busy',
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
