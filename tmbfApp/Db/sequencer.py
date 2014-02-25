# Sequencer and NCO

from common import *

import tune


# The sequencer has eight possible states, however state 0 has somewhat
# different behaviour from the remaining 7.

# Update the entire sequencer state on a write to any PV.  However, we don't
# want to write anything until startup and PV initialisation is complete, so we
# start with writing disabled and reenable at end.
update = boolOut('SEQ:WRITE',
    DISV = 0, FLNK = tune.setting_changed,
    DESC = 'Update sequencer settings')
seq_pc = longOut('SEQ:PC', 1, 7, DISV = 0, DESC = 'Sequencer PC')
records.seq('SEQ:ENWRITE',
    PINI = 'RUN', SELM = 'All',     # Perform initialisation after startup
    LNK1 = update.DISV, DO1 = 1,
    LNK2 = update.PROC, DO2 = 0,
    LNK3 = seq_pc.DISV, DO3 = 1,
    LNK4 = seq_pc.PROC, DO4 = 0)

for state in range(1, 8):
    aOut('SEQ:%d:START_FREQ' % state,
        -BUNCHES_PER_TURN, BUNCHES_PER_TURN, 'tune', 5,
        FLNK = update, DESC = 'Sweep NCO start frequency')
    aOut('SEQ:%d:STEP_FREQ' % state,
        -BUNCHES_PER_TURN, BUNCHES_PER_TURN, 'tune', 7,
        FLNK = update, DESC = 'Sweep NCO step frequency')
    longOut('SEQ:%d:DWELL' % state, 1, (1<<31) - 1,
        FLNK = update, EGU = 'turns', DESC = 'Sweep dwell time')
    longOut('SEQ:%d:COUNT' % state, 1, 1<<12,
        FLNK = update, DESC = 'Sweep count')
    mbbOut('SEQ:%d:BANK' % state, 'Bank 0', 'Bank 1', 'Bank 2', 'Bank 3',
        FLNK = update, DESC = 'Bunch bank selection')
    mbbOut('SEQ:%d:GAIN' % state, DESC = 'Sweep NCO gain', FLNK = update,
        *dBrange(14, -6) + ['Off'])
    boolOut('SEQ:%d:ENWIN' % state, 'Disabled', 'Windowed',
        FLNK = update, DESC = 'Enable detector window')
    boolOut('SEQ:%d:CAPTURE' % state, 'Discard', 'Capture',
        FLNK = update, DESC = 'Enable data capture')
    longOut('SEQ:%d:HOLDOFF' % state, 0, 65535,
        FLNK = update, DESC = 'Detector holdoff')
    boolOut('SEQ:%d:BLANK' % state, 'Off', 'Blanking',
        FLNK = update, DESC = 'Detector blanking control')

    # This fellow is treated a little differently and is processed internally.
    aOut('SEQ:%d:END_FREQ' % state,
        -BUNCHES_PER_TURN, BUNCHES_PER_TURN, 'tune', 5,
        PINI = 'NO', DESC = 'Sweep NCO start frequency')

# This is the only valid control in state 0.
mbbOut('SEQ:0:BANK', 'Bank 0', 'Bank 1', 'Bank 2', 'Bank 3',
    DESC = 'Bunch bank selection')

duration = longIn('SEQ:DURATION', EGU = 'turns', DESC = 'Raw capture duration')
Trigger('SEQ:INFO',
    longIn('SEQ:LENGTH',
        LOW = 0, LSV = 'MAJOR', HIGH = 4097, HSV = 'MINOR',
        DESC = 'Sequencer capture count'),
    duration,
    records.calc('SEQ:DURATION:S',
        CALC = 'A/B', INPA = duration, INPB = 533830, PREC = 3, EGU = 's',
        DESC = 'Capture duration'))

longIn('SEQ:PC', DESC = 'Current sequencer state', SCAN = '.1 second')
Action('SEQ:RESET', DESC = 'Halt detector if busy')


Trigger('BUF',
    Waveform('BUF:WFA', BUF_DATA_LENGTH, 'SHORT',
        DESC = 'Fast buffer first half'),
    Waveform('BUF:WFB', BUF_DATA_LENGTH, 'SHORT',
        DESC = 'Fast buffer second half'))

mbbOut('BUF:SELECT', 'FIR+ADC', 'IQ', 'FIR+DAC', 'ADC+DAC',
    FLNK = tune.setting_changed,
    DESC = 'Select buffer capture source')
