# DDR

from common import *


SHORT_TURN_WF_COUNT = 8
LONG_TURN_WF_COUNT = 256

BUFFER_TURN_COUNT = 64 * 1024 * 1024 // BUNCHES_PER_TURN // 2 - 2
SHORT_TURN_WF_LENGTH = BUNCHES_PER_TURN * SHORT_TURN_WF_COUNT
LONG_TURN_WF_LENGTH = BUNCHES_PER_TURN * LONG_TURN_WF_COUNT


# Input mode selection.  Note that when input mode IQ is selected the trigger
# behaviour of the DDR buffer is very different.
mbbOut('DDR:INPUT',
    'ADC', 'FIR', 'Raw DAC', 'DAC', 'IQ', DESC = 'DDR input select')

# Triggering and control for IQ mode
boolOut('DDR:AUTOSTOP', 'Multi-shot', 'One-shot',
    DESC = 'Action when sequencer completes')
Action('DDR:START', DESC = 'Initiate DDR data capture')
Action('DDR:STOP', DESC = 'Halt DDR data capture')

longIn('DDR:COUNT', SCAN = '.2 second', DESC = 'Captured sample count')


# The short waveforms will be updated on any DDR trigger.
ddr_bunch_buffer = Waveform('DDR:BUNCHWF', BUFFER_TURN_COUNT, 'SHORT',
    DESC = 'Single bunch waveform')
Trigger('DDR:SHORT',
    ddr_bunch_buffer,
    Waveform('DDR:SHORTWF', SHORT_TURN_WF_LENGTH, 'SHORT',
        DESC = 'Short turn by turn waveform'))
# Updating BUNCHSEL will always update BUNCHWF
longOut('DDR:BUNCHSEL', 0, BUNCHES_PER_TURN-1,
    FLNK = ddr_bunch_buffer,
    DESC = 'Select bunch for DDR readout')


# The long waveform and its records are only processed when selected.
boolOut('DDR:LONGEN', 'No', 'Long', DESC = 'Enable long waveform update')
Trigger('DDR:LONG',
    Waveform('DDR:LONGWF', LONG_TURN_WF_LENGTH, 'SHORT',
        DESC = 'Long turn by turn waveform'),
    longIn('DDR:TURNSEL', DESC = 'Turn selection readback'))
# Updating TURNSEL will trigger reprocessing of LONGWF and the readback, but
# this may not happen immediately.
longOut('DDR:TURNSEL', -BUFFER_TURN_COUNT, BUFFER_TURN_COUNT,
    DESC = 'Select start turn for readout')

# This reflects the state of the long waveform.
boolIn('DDR:READY', 'Not Ready', 'Triggered',
    SCAN = 'I/O Intr', DESC = 'Long buffer status')
