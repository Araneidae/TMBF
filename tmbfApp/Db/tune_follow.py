# Definitions for fast tune following

from common import *

IIR_ORDER = Parameter('IIR_ORDER', 'Order of IIR filter')
IIR_LENGTH = Parameter('IIR_LENGTH', 'Order of IIR filter + 1')

ForwardLink('FTUN:CONTROL', 'Update tune follow control settings',
    longOut('FTUN:DWELL', 1, 1<<16, DESC = 'Tune following dwell time'),
    longOut('FTUN:BUNCH', 0, BUNCHES_PER_TURN-1, DESC = 'Tune following bunch'),
    boolOut('FTUN:MULTIBUNCH', 'Single Bunch', 'Multi-Bunch',
        DESC = 'Tune following bunch mode'),
    mbbOut('FTUN:INPUT', 'ADC', 'FIR', DESC = 'Tune following input selection'),
    mbbOut('FTUN:GAIN',
        DESC = 'Tune following detector gain', *dBrange(8, -12)))

ForwardLink('FTUN:SET_TAPS', 'Update tune following IIR taps',
    WaveformOut('FTUN:TAPS:A', IIR_LENGTH, 'FLOAT', DESC = 'Forward IIR taps'),
    WaveformOut('FTUN:TAPS:B', IIR_ORDER,  'FLOAT', DESC = 'Feedback IIR taps'))

DATA_LENGTH = RAW_BUF_DATA_LENGTH / 4
Trigger('FTUN:DEBUG',
    Waveform('FTUN:DEBUG:I', DATA_LENGTH, 'SHORT'),
    Waveform('FTUN:DEBUG:Q', DATA_LENGTH, 'SHORT'),
    Waveform('FTUN:DEBUG:MAG', DATA_LENGTH, 'LONG'),
    Waveform('FTUN:DEBUG:ANGLE', DATA_LENGTH, 'FLOAT'),
    Waveform('FTUN:DEBUG:FILTER', DATA_LENGTH, 'FLOAT'))
