from common import *

BUF_DATA_LENGTH = 16384


Trigger('BUF',
    Waveform('BUF:WFA', BUF_DATA_LENGTH, 'SHORT',
        DESC = 'Fast buffer first half'),
    Waveform('BUF:WFB', BUF_DATA_LENGTH, 'SHORT',
        DESC = 'Fast buffer second half'))

mbbOut('BUF:SELECT', 'FIR+ADC', 'IQ', 'FIR+DAC', 'ADC+DAC',
    DESC = 'Select buffer capture source')
mbbOut('BUF:TRIGSEL', 'Soft trigger', 'External trigger',
    DESC = 'Select buffer trigger source')

Action('BUF:ARM', DESC = 'Arm buffer trigger')
Action('BUF:SOFT_TRIG', DESC = 'Buffer soft trigger')
