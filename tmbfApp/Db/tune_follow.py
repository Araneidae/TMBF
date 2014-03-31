# Definitions for fast tune following

from common import *

FTUN_FREQ_LENGTH = 4096

MAX_DELTA_FREQ = (2**17 - 1) * 2**-32 * 936

def IIR(count, step = 1):
    return ['2^%d' % (n * step) for n in range(count)]

# Tune following settings.
ftun_control = ForwardLink(
    'FTUN:CONTROL', 'Update tune follow control settings',

    longOut('FTUN:DWELL', 1, 1<<16, DESC = 'Tune following dwell time'),
    longOut('FTUN:BUNCH', 0, BUNCHES_PER_TURN-1, DESC = 'Tune following bunch'),
    boolOut('FTUN:BLANKING', 'Off', 'Blanking',
        DESC = 'Respect blanking trigger'),
    boolOut('FTUN:MULTIBUNCH', 'Single Bunch', 'All Bunches',
        DESC = 'Tune following bunch mode'),
    mbbOut('FTUN:INPUT', 'ADC', 'FIR', DESC = 'Tune following input selection'),
    mbbOut('FTUN:GAIN',
        DESC = 'Tune following detector gain', *dBrange(8, -12)),
    mbbOut('FTUN:IIR', DESC = 'IIR half life', *IIR(8)),
    aOut('FTUN:TARGET', -360, 360, 'deg', 1,
        DESC = 'Target phase for tune following'),
    longOut('FTUN:KI', -(1<<17), (1<<17)-1,
        DESC = 'Tune following integral scale'),
    longOut('FTUN:KP', -(1<<17), (1<<17)-1,
        DESC = 'Tune following proportional scale'),
    longOut('FTUN:MINMAG', 0, (1<<15)-1, DESC = 'Minimum signal magnitude'),
    aOut('FTUN:MAXDELTA', 0, MAX_DELTA_FREQ, 'tune', 6,
        DESC = 'Maximum frequency deviation to follow'),

    mbbOut('FTUN:IQ:IIR', DESC = 'IQ readback IIR', *IIR(8, 2)),
    mbbOut('FTUN:FREQ:IIR', DESC = 'Frequency readback IIR', *IIR(8, 2)),

    aOut('FTUN:LOOP:DELAY',
        EGU = 'turns', PREC = 1, DESC = 'Closed loop delay in turns'),

    FLNK = create_fanout('FTUN:IIR:FAN',
        aIn('FTUN:IIR:TC', EGU = 'ms', PREC = 1,
            DESC = 'Time constant for feedback IIR'),
        aIn('FTUN:IQ:IIR:TC', EGU = 'ms', PREC = 1,
            DESC = 'Time constant for IQ IIR'),
        aIn('FTUN:FREQ:IIR:TC', EGU = 'ms', PREC = 1,
            DESC = 'Time constant for frequency IIR'),
        aIn('FTUN:PHASE:OFFSET', PREC= 2, EGU = 'deg',
            DESC = 'Phase offset'))
)


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
        DESC = 'Tune following frequency (raw units)'),
    aIn('NCO:FREQ:MEAN', 0, BUNCHES_PER_TURN, 'tune', 5,
        DESC = 'Mean NCO frequency'))

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
        longIn('FTUN:I', -32768, 32767, DESC = 'Current I value'),
        longIn('FTUN:Q', -32768, 32767, DESC = 'Current Q value'),
        aIn('FTUN:ANGLE', -180, 180, 'deg', 3,
            DESC = 'Current angle from detector'),
        aIn('FTUN:ANGLEDELTA', -180, 180, 'deg', 3,
            DESC = 'Relative angle from detector'),
        longIn('FTUN:MAG', 0, 65535, DESC = 'Current magnitude from detector'),

        # Min/max readouts
        longIn('FTUN:I:MIN', -32768, 32767, DESC = 'Minimum I value'),
        longIn('FTUN:I:MAX', -32768, 32767, DESC = 'Maximum I value'),
        longIn('FTUN:Q:MIN', -32768, 32767, DESC = 'Minimum Q value'),
        longIn('FTUN:Q:MAX', -32768, 32767, DESC = 'Maximum Q value'),
        aIn('FTUN:IQ:VAR', PREC = 3, DESC = 'Relative IQ variation'),
    ))


# Fixed NCO control
aOut('NCO:FREQ', -BUNCHES_PER_TURN, BUNCHES_PER_TURN, 'tune', 5,
    DESC = 'Fixed NCO frequency', FLNK = ftun_control)
aIn('NCO:FREQ',  0, BUNCHES_PER_TURN, 'tune', 6,
    DESC = 'Current fixed NCO frequency', SCAN = '.1 second')
mbbOut('NCO:GAIN', DESC = 'Fixed NCO gain', *dBrange(14, -6) + ['Off'])
