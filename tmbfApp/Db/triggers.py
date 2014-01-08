# Common trigger control

from common import *


def TriggerSource(name, external, desc):
    boolOut('TRG:%s:MODE' % name, 'One Shot', 'Retrigger',
        DESC = 'Mode select for %s' % desc)
    Action('TRG:%s:%s' % (name, 'ARM' if external else 'FIRE'),
        DESC = '%s %s' % ('Arm' if external else 'Fire', desc))
    Action('TRG:%s:RESET' % name, DESC = 'Reset %s' % desc)
    boolIn('TRG:%s:STATUS' % name, 'Ready', 'Busy',
        SCAN = 'I/O Intr', DESC = 'Status for %s' % desc)


def TriggerStatus(name, desc):
    boolIn('TRG:%s:STATUS' % name, 'Ready', 'Busy',
        SCAN = 'I/O Intr', DESC = '%s status' % desc)

def TriggerTarget(name, desc):
    mbbOut('TRG:%s:SEL' % name, 'Disabled', 'Soft 1', 'Soft 2', 'External',
        DESC = '%s trigger source' % desc)
    TriggerStatus(name, desc)
    longOut('TRG:%s:DELAY' % name, 0, (1<<24)-1, EGU = 'turns',
        DESC = 'Trigger delay in turns')


# Three trigger sources including two soft triggers
TriggerSource('S1', False, 'soft trigger 1')
TriggerSource('S2', False, 'soft trigger 2')
TriggerSource('EXT', True, 'external trigger')

# Trigger source selections for the trigger targets.
TriggerTarget('DDR', 'Long buffer')
TriggerTarget('BUF', 'Fast buffer')

boolOut('TRG:SEQ:ENA', 'Disabled', 'Enabled', DESC = 'Sequencer trigger enable')
TriggerStatus('SEQ', 'Sequencer')


longOut('TRG:BLANKING', 0, 65535, EGU = 'turns',
    DESC = 'Sequencer blanking window after trigger')


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
