# Definitions for fast tune following

from common import *

FTUN_FREQ_LENGTH = 1024

MAX_DELTA_FREQ = (2**17 - 1) * 2**-32 * 936

ForwardLink('FTUN:CONTROL', 'Update tune follow control settings',
    longOut('FTUN:DWELL', 1, 1<<16, DESC = 'Tune following dwell time'),
    longOut('FTUN:BUNCH', 0, BUNCHES_PER_TURN-1, DESC = 'Tune following bunch'),
    boolOut('FTUN:MULTIBUNCH', 'Single Bunch', 'Multi-Bunch',
        DESC = 'Tune following bunch mode'),
    mbbOut('FTUN:INPUT', 'ADC', 'FIR', DESC = 'Tune following input selection'),
    mbbOut('FTUN:GAIN',
        DESC = 'Tune following detector gain', *dBrange(8, -12)),
    mbbOut('FTUN:IIR', DESC = 'IIR half life', *['2^%d' % n for n in range(8)]),
    aOut('FTUN:TARGET', -180, 180, 'deg', 1,
        DESC = 'Target phase for tune following'),
    longOut('FTUN:SCALE', -(1<<17), (1<<17)-1,
        DESC = 'Tune following feedback scale'),
    longOut('FTUN:MINMAG', 0, (1<<15)-1, DESC = 'Minimum signal magnitude'),
    aOut('FTUN:MAXDELTA', 0, MAX_DELTA_FREQ, 'tune', 6,
        DESC = 'Maximum frequency deviation to follow'))


Action('FTUN:START', DESC = 'Start tune following')


DATA_LENGTH = RAW_BUF_DATA_LENGTH / 4
Trigger('FTUN:DEBUG',
    Waveform('FTUN:DEBUG:I', DATA_LENGTH, 'SHORT'),
    Waveform('FTUN:DEBUG:Q', DATA_LENGTH, 'SHORT'),
    Waveform('FTUN:DEBUG:MAG', DATA_LENGTH, 'LONG'),
    Waveform('FTUN:DEBUG:ANGLE', DATA_LENGTH, 'FLOAT'),
    Waveform('FTUN:DEBUG:FILTER', DATA_LENGTH, 'FLOAT'),
    Waveform('FTUN:DEBUG:DELTAF', DATA_LENGTH, 'FLOAT'),
    Waveform('FTUN:DEBUG:STATUS', DATA_LENGTH, 'SHORT'))

Trigger('FTUN',
    boolIn('FTUN:FREQ:DROPOUT', 'Ok', 'Data lost',
        ZSV = 'NO_ALARM', OSV = 'MAJOR', DESC = 'Data lost detection'),
    Waveform('FTUN:FREQ', FTUN_FREQ_LENGTH, 'FLOAT',
        DESC = 'Tune following frequency'),
    Waveform('FTUN:RAWFREQ', FTUN_FREQ_LENGTH, 'LONG',
        DESC = 'Tune following frequency (raw units)'))


# Also put the fixed NCO control here
aOut('NCO:FREQ', -BUNCHES_PER_TURN, BUNCHES_PER_TURN, 'tune', 5,
    DESC = 'Fixed NCO frequency')
aIn('NCO:FREQ',  -BUNCHES_PER_TURN, BUNCHES_PER_TURN, 'tune', 5,
    DESC = 'Current fixed NCO frequency', SCAN = '.1 second')
mbbOut('NCO:GAIN', DESC = 'Fixed NCO gain', *dBrange(14, -6) + ['Off'])
