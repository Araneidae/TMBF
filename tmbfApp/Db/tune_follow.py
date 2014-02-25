# Definitions for fast tune following

from common import *

FTUN_FREQ_LENGTH = 1024

MAX_DELTA_FREQ = (2**17 - 1) * 2**-32 * 936

# Tune following settings.
ForwardLink('FTUN:CONTROL', 'Update tune follow control settings',
    longOut('FTUN:DWELL', 1, 1<<16, DESC = 'Tune following dwell time'),
    longOut('FTUN:BUNCH', 0, BUNCHES_PER_TURN-1, DESC = 'Tune following bunch'),
    boolOut('FTUN:BLANKING', 'Off', 'Blanking',
        DESC = 'Respect blanking trigger'),
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
Action('FTUN:STOP', DESC = 'Stop tune following')

# Detailed data about internal processing of tune following data stream
DATA_LENGTH = RAW_BUF_DATA_LENGTH / 4
Trigger('FTUN:DEBUG',
    Waveform('FTUN:DEBUG:I', DATA_LENGTH, 'SHORT', DESC = 'Detected I'),
    Waveform('FTUN:DEBUG:Q', DATA_LENGTH, 'SHORT', DESC = 'Detected Q'),
    Waveform('FTUN:DEBUG:MAG', DATA_LENGTH, 'LONG',
        DESC = 'Detected magnitude'),
    Waveform('FTUN:DEBUG:ANGLE', DATA_LENGTH, 'FLOAT',
        DESC = 'Detected angle'),
    Waveform('FTUN:DEBUG:FILTER', DATA_LENGTH, 'FLOAT',
        DESC = 'Filtered angle'),
    Waveform('FTUN:DEBUG:DELTAF', DATA_LENGTH, 'FLOAT',
        DESC = 'Frequency offset'),
    Waveform('FTUN:DEBUG:STATUS', DATA_LENGTH, 'SHORT',
        DESC = 'Raw status bits'))

# Tune following data.
Trigger('FTUN',
    boolIn('FTUN:FREQ:DROPOUT', 'Ok', 'Data lost',
        ZSV = 'NO_ALARM', OSV = 'MAJOR', DESC = 'Data lost detection'),
    Waveform('FTUN:FREQ', FTUN_FREQ_LENGTH, 'FLOAT',
        DESC = 'Tune following frequency'),
    Waveform('FTUN:RAWFREQ', FTUN_FREQ_LENGTH, 'LONG',
        DESC = 'Tune following frequency (raw units)'))

# Status
def status(name, desc):
    return boolIn(name, 'Ok', 'Fault', OSV = 'MAJOR', DESC = desc)
Action('FTUN:STAT:SCAN',
    SCAN = '.2 second', DESC = 'Tune following status scan',
    FLNK = create_fanout('FTUN:SCAN:FAN',
        # Normal status bits
        status('FTUN:STAT:INP', 'FIR input overflow'),
        status('FTUN:STAT:ACC', 'Detector accumulator overflow'),
        status('FTUN:STAT:DET', 'Detector output overflow'),
        status('FTUN:STAT:MAG', 'Signal magnitude too small'),
        status('FTUN:STAT:VAL', 'Frequency shift too large'),
        # Stop reason status bits
        status('FTUN:STOP:INP', 'FIR input overflow'),
        status('FTUN:STOP:ACC', 'Detector accumulator overflow'),
        status('FTUN:STOP:DET', 'Detector output overflow'),
        status('FTUN:STOP:MAG', 'Signal magnitude too small'),
        status('FTUN:STOP:VAL', 'Frequency shift too large'),
        # Activity status
        boolIn('FTUN:ACTIVE', 'Not running', 'Running',
            DESC = 'Tune following activity'),

        # Current position and angle readout
        aIn('FTUN:ANGLE', -180, 180, 'deg', 6,
            DESC = 'Current angle from detector'),
        longIn('FTUN:MAG', 0, 65535, DESC = 'Current magnitude from detector')
    ))


# Fixed NCO control
aOut('NCO:FREQ', -BUNCHES_PER_TURN, BUNCHES_PER_TURN, 'tune', 5,
    DESC = 'Fixed NCO frequency')
aIn('NCO:FREQ',  -BUNCHES_PER_TURN, BUNCHES_PER_TURN, 'tune', 5,
    DESC = 'Current fixed NCO frequency', SCAN = '.1 second')
mbbOut('NCO:GAIN', DESC = 'Fixed NCO gain', *dBrange(14, -6) + ['Off'])
