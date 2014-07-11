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
boolOut('DDR:AUTOSTOP', 'Manual stop', 'Auto-stop',
    DESC = 'Action when sequencer completes')
Action('DDR:START', DESC = 'Initiate DDR data capture')
Action('DDR:STOP', DESC = 'Halt DDR data capture')

longIn('DDR:COUNT', SCAN = '.2 second', DESC = 'Captured sample count')


# All three waveforms are updated when capture is completed, and the long and
# bunch waveforms are also updated when their corresponding control parameters
# are updated.
bunch_waveform = Waveform('DDR:BUNCHWF', BUFFER_TURN_COUNT, 'SHORT',
    FLNK = boolIn('DDR:BUNCHWF:STATUS', 'Ok', 'Fault', OSV = 'MAJOR',
        DESC = 'DDR bunch waveform status'),
    DESC = 'Single bunch waveform')
long_waveform = Waveform('DDR:LONGWF', LONG_TURN_WF_LENGTH, 'SHORT',
    FLNK = boolIn('DDR:LONGWF:STATUS', 'Ok', 'Fault', OSV = 'MAJOR',
        DESC = 'DDR long waveform status'),
    DESC = 'Long turn by turn waveform')
short_waveform = Waveform('DDR:SHORTWF', SHORT_TURN_WF_LENGTH, 'SHORT',
    DESC = 'Short turn by turn waveform')
Trigger('DDR:UPDATE', short_waveform, bunch_waveform, long_waveform)

# Control parameters for long and bunch waveforms.
longOut('DDR:TURNSEL', -BUFFER_TURN_COUNT, BUFFER_TURN_COUNT,
    FLNK = long_waveform, DESC = 'Select start turn for readout')
longOut('DDR:BUNCHSEL', 0, BUNCHES_PER_TURN-1,
    FLNK = bunch_waveform, DESC = 'Select bunch for DDR readout')

# Three overflow detection bits are generated
overflows = [
    boolIn('DDR:OVF:INP', 'Ok', 'Overflow', OSV = 'MAJOR',
        DESC = 'Detector input overflow'),
    boolIn('DDR:OVF:ACC', 'Ok', 'Overflow', OSV = 'MAJOR',
        DESC = 'Detector accumulator overflow'),
    boolIn('DDR:OVF:IQ',  'Ok', 'Overflow', OSV = 'MAJOR',
        DESC = 'IQ scaling overflow')]
overflows.append(
    AggregateSeverity('DDR:OVF', 'Detector overflow', overflows))
Trigger('DDR:OVF', *overflows)
