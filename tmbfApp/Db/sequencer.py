# Sequencer and NCO

from common import *

import tune

SUPER_SEQ_STATES = 1024

# Nominal RF frequency is for 60 cm bunch separation.
RF_FREQ = 299792458 / 0.6
REV_FREQ = RF_FREQ / BUNCHES_PER_TURN

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
    longOut('SEQ:%d:DWELL' % state, 1, 1<<16,
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
        None, None, 'tune', 5,
        PINI = 'NO', DESC = 'Sweep NCO end frequency')

# This is the only valid control in state 0.
mbbOut('SEQ:0:BANK', 'Bank 0', 'Bank 1', 'Bank 2', 'Bank 3',
    DESC = 'Bunch bank selection')

duration = longIn('SEQ:DURATION', EGU = 'turns', DESC = 'Raw capture duration')
length = longIn('SEQ:LENGTH', DESC = 'Sequencer capture count')
Trigger('SEQ:INFO',
    length, duration,
    records.calc('SEQ:DURATION:S',
        CALC = 'A/B', INPA = duration, INPB = REV_FREQ, PREC = 3, EGU = 's',
        DESC = 'Capture duration'))

longIn('SEQ:PC', DESC = 'Current sequencer state', SCAN = '.1 second')
Action('SEQ:RESET', DESC = 'Halt sequencer if busy')

longOut('SEQ:TRIGGER', 0, 7, DESC = 'State to generate sequencer trigger')


# Super-sequencer control and state
longIn('SEQ:SUPER:COUNT', 0, SUPER_SEQ_STATES, SCAN = '.1 second',
    DESC = 'Current super sequencer count')
super_count = longOut('SEQ:SUPER:COUNT', 1, SUPER_SEQ_STATES,
    FLNK = tune.setting_changed,
    DESC = 'Super sequencer count')
offsets = WaveformOut('SEQ:SUPER:OFFSET', SUPER_SEQ_STATES, 'DOUBLE',
    PREC = 5, FLNK = tune.setting_changed,
    DESC = 'Frequency offsets for super sequencer')
Action('SEQ:SUPER:RESET', FLNK = offsets,
    DESC = 'Reset super sequencer offsets')

# Total sequencer counts
super_duration = records.calc('SEQ:TOTAL:DURATION',
    CALC = 'A*B', INPA = CP(super_count), INPB = CP(duration),
    EGU = 'turns', DESC = 'Super sequence raw capture duration')
records.calc('SEQ:TOTAL:DURATION:S',
    CALC = 'A/B', INPA = CP(super_duration), INPB = REV_FREQ,
    PREC = 3, EGU = 's', DESC = 'Super capture duration')
records.calc('SEQ:TOTAL:LENGTH',
    CALC = 'A*B', INPA = CP(super_count), INPB = CP(length),
    DESC = 'Super sequencer capture count')


Trigger('BUF',
    Waveform('BUF:WF', BUF_DATA_LENGTH, 'LONG', DESC = 'Raw fast buffer'),
    Waveform('BUF:WFA', BUF_DATA_LENGTH, 'SHORT',
        DESC = 'Fast buffer first half'),
    Waveform('BUF:WFB', BUF_DATA_LENGTH, 'SHORT',
        DESC = 'Fast buffer second half'))

mbbOut('BUF:SELECT', 'FIR+ADC', 'IQ', 'FIR+DAC', 'ADC+DAC', 'Debug',
    FLNK = tune.setting_changed,
    DESC = 'Select buffer capture source')
