# DDR

from common import *


SHORT_TURN_WF_COUNT = 8
LONG_TURN_WF_COUNT = 256

BUFFER_TURN_COUNT = 64 * 1024 * 1024 // SAMPLES_PER_TURN // 2 - 2
SHORT_TURN_WF_LENGTH = SAMPLES_PER_TURN * SHORT_TURN_WF_COUNT
LONG_TURN_WF_LENGTH = SAMPLES_PER_TURN * LONG_TURN_WF_COUNT

mbbOut('DDR:INPUT', 'ADC', 'DAC',
    DESC = 'DDR input select')

trig_mode = mbbOut('DDR:TRIGMODE', 'Retrigger', 'One Shot',
    DESC = 'DDR retrigger enable')
mbbOut('DDR:TRIGSEL', 'Soft trigger', 'External trigger',
    DESC = 'Select DDR trigger source')

ddr_bunch_buffer = Waveform('DDR:BUNCHWF', BUFFER_TURN_COUNT, 'SHORT',
    DESC = 'Single bunch waveform')
longOut('DDR:BUNCHSEL', 0, 935,
    FLNK = ddr_bunch_buffer,
    DESC = 'Select bunch for DDR readout')

Trigger('DDR',
    ddr_bunch_buffer,
    Waveform('DDR:SHORTWF', SHORT_TURN_WF_LENGTH, 'SHORT',
        DESC = 'Short turn by turn waveform'))


# The long waveform and its records are only processed in single shot mode.
turnsel_readback = longIn('DDR:TURNSEL', DESC = 'Turn selection readback')
longOut('DDR:TURNSEL', -BUFFER_TURN_COUNT, BUFFER_TURN_COUNT,
    DESC = 'Select start turn for readout')

long_ddr_turn_buffer = Waveform('DDR:LONGWF', LONG_TURN_WF_LENGTH, 'SHORT',
    SCAN = 'I/O Intr',
    FLNK = turnsel_readback,
    DESC = 'Long turn by turn waveform')
boolIn('DDR:READY', 'Not Ready', 'Triggered',
    SCAN = 'I/O Intr', DESC = 'Long buffer status')


Action('DDR:ARM', DESC = 'Arm DDR trigger')
Action('DDR:SOFT_TRIG', DESC = 'DDR soft trigger')
