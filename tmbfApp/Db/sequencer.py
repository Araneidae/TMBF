# Sequencer and NCO

from common import *


# The sequencer has eight possible states, however state 0 has somewhat
# different behaviour from the remaining 7.


validity = records.bi('SEQ:VALID',
    ZNAM = 'Needs Write',   ZSV  = 'MINOR',
    ONAM = 'Written',       OSV  = 'NO_ALARM')
invalidate = records.bo('SEQ:INVALIDATE', OUT = PP(validity), VAL = 0)
make_valid = records.bo('SEQ:MAKE_VALID', OUT = PP(validity), VAL = 1)
for state in range(1, 8):
    aOut('SEQ:%d:START_FREQ' % state, -936, 936, 'tune', 5,
        FLNK = invalidate, DESC = 'Sweep NCO start frequency')
    aOut('SEQ:%d:STEP_FREQ' % state, -936, 936, 'tune', 7,
        FLNK = invalidate, DESC = 'Sweep NCO step frequency')
    longOut('SEQ:%d:DWELL' % state, 1, (1<<31) - 1,
        FLNK = invalidate, EGU = 'turns', DESC = 'Sweep dwell time')
    longOut('SEQ:%d:COUNT' % state, 1, 1<<12,
        FLNK = invalidate, DESC = 'Sweep count')
    mbbOut('SEQ:%d:BANK' % state, 'Bank 0', 'Bank 1', 'Bank 2', 'Bank 3',
        FLNK = invalidate, DESC = 'Bunch bank selection')
    mbbOut('SEQ:%d:GAIN' % state, DESC = 'Sweep NCO gain', FLNK = invalidate,
        *['Off'] + dBrange(15, -3))

# This is the only valid control in state 0.
mbbOut('SEQ:0:BANK', 'Bank 0', 'Bank 1', 'Bank 2', 'Bank 3',
    DESC = 'Bunch bank selection')

# Note the use of PINI='RUN' -- this triggers PINI processing later than all
# the saved states above, ensuring that we write the correct saved state.
boolOut('SEQ:WRITE',
    PINI = 'RUN', FLNK = make_valid, DESC = 'Write sequencer settings')

longOut('SEQ:PC', 0, 7, DESC = 'Sequencer PC')
longIn('SEQ:PC', DESC = 'Current sequencer state', SCAN = '.1 second')
Action('SEQ:RESET')
