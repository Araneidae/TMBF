# Sequencer and NCO

from common import *


# The sequencer has eight possible states, however state 0 has somewhat
# different behaviour from the remaining 7.

update = boolOut('SEQ:WRITE', DESC = 'Write sequencer settings')
for state in range(1, 8):
    aOut('SEQ:%d:START_FREQ' % state, -936, 936, 'tune', 5,
        FLNK = update, DESC = 'Sweep NCO start frequency')
    aOut('SEQ:%d:STEP_FREQ' % state, -936, 936, 'tune', 7,
        FLNK = update, DESC = 'Sweep NCO step frequency')
    longOut('SEQ:%d:DWELL' % state, 1, (1<<31) - 1,
        FLNK = update, EGU = 'turns', DESC = 'Sweep dwell time')
    longOut('SEQ:%d:COUNT' % state, 1, 1<<12,
        FLNK = update, DESC = 'Sweep count')
    mbbOut('SEQ:%d:BANK' % state, 'Bank 0', 'Bank 1', 'Bank 2', 'Bank 3',
        FLNK = update, DESC = 'Bunch bank selection')
    mbbOut('SEQ:%d:GAIN' % state, DESC = 'Sweep NCO gain', FLNK = update,
        *dBrange(8, -6))
    boolOut('SEQ:%d:ENWIN' % state, 'Disabled', 'Enabled',
        FLNK = update, DESC = 'Enable detector window')

# This is the only valid control in state 0.
mbbOut('SEQ:0:BANK', 'Bank 0', 'Bank 1', 'Bank 2', 'Bank 3',
    DESC = 'Bunch bank selection')

longOut('SEQ:PC', 0, 7, DESC = 'Sequencer PC')
longIn('SEQ:PC', DESC = 'Current sequencer state', SCAN = '.1 second')
Action('SEQ:RESET', DESC = 'Halt detector if busy')
