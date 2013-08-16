# DDR

from common import *


SHORT_TURN_WF_COUNT = 8
LONG_TURN_WF_COUNT = 256

BUFFER_TURN_COUNT = 64 * 1024 * 1024 // SAMPLES_PER_TURN // 2 - 2
SHORT_TURN_WF_LENGTH = SAMPLES_PER_TURN * SHORT_TURN_WF_COUNT
LONG_TURN_WF_LENGTH = SAMPLES_PER_TURN * LONG_TURN_WF_COUNT


mbbOut('DDR:INPUT', 'ADC', 'DAC', 'FIR', DESC = 'DDR input select')


# The short waveforms will be updated on any DDR trigger.
ddr_bunch_buffer = Waveform('DDR:BUNCHWF', BUFFER_TURN_COUNT, 'SHORT',
    DESC = 'Single bunch waveform')
Trigger('DDR:SHORT',
    ddr_bunch_buffer,
    Waveform('DDR:SHORTWF', SHORT_TURN_WF_LENGTH, 'SHORT',
        DESC = 'Short turn by turn waveform'))
# Updating BUNCHSEL will always update BUNCHWF
longOut('DDR:BUNCHSEL', 0, 935,
    FLNK = ddr_bunch_buffer,
    DESC = 'Select bunch for DDR readout')


# The long waveform and its records are only processed in single shot mode.
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
