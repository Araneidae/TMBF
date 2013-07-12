from common import *


Trigger('BUF',
    Waveform('BUF:WFA', BUF_DATA_LENGTH, 'SHORT',
        DESC = 'Fast buffer first half'),
    Waveform('BUF:WFB', BUF_DATA_LENGTH, 'SHORT',
        DESC = 'Fast buffer second half'))

mbbOut('BUF:SELECT', 'FIR+ADC', 'IQ', 'FIR+DAC', 'ADC+DAC',
    DESC = 'Select buffer capture source')
