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
    longOut('TRG:%s:DELAY' % name, 0, 65535, EGU = 'turns')


# Three trigger sources including two soft triggers
TriggerSource('S1', False, 'soft trigger 1')
TriggerSource('S2', False, 'soft trigger 2')
TriggerSource('EXT', True, 'external trigger')

# Trigger source selections for the trigger targets.
TriggerTarget('DDR', 'Long buffer')
TriggerTarget('BUF', 'Fast buffer')

boolOut('TRG:SEQ:ENA', 'Disabled', 'Enabled', DESC = 'Sequencer trigger enable')
TriggerStatus('SEQ', 'Sequencer')

longIn('TRG:RAWPHASE', DESC = 'Raw trigger phase bits', SCAN = '.2 second')
