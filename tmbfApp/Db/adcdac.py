from common import *

# ADC + DAC
#
# Both of these systems are very similar: all we have are min/max waveforms and
# derived data (identical for both) and a couple of device specific controls.

def dB(db): return 10.**(db/20.)

def minmax_pvs(source):
    # The following records are polled together at 200ms intervals
    pvs = [
        Waveform('%s:MINBUF' % source, BUNCHES_PER_TURN, 'FLOAT',
            DESC = '%s min value per bunch' % source),
        Waveform('%s:MAXBUF' % source, BUNCHES_PER_TURN, 'FLOAT',
            DESC = '%s max value per bunch' % source),
        Waveform('%s:DIFFBUF' % source, BUNCHES_PER_TURN, 'FLOAT',
            DESC = '%s max min diff per bunch' % source),
        aIn('%s:MEAN' % source, -4096, 4096, '', 5,
            DESC = 'Readback %s diff mean' % source),
        aIn('%s:STD' % source, 0, 4096, '', 5,
            DESC = 'Readback %s diff variance' % source),
        aIn('%s:MAX' % source, 0, 1, '', 4,
            DESC = 'Maximum %s reading' % source,
            HSV  = 'MINOR',  HIGH = dB(-3),     # 70.8 %
            HHSV = 'MAJOR',  HIHI = dB(-1.6)),  # 83.2 %
    ]
    Action('%s:SCAN' % source,
        DESC = 'Trigger %s scanning' % source, SCAN = '.2 second',
        FLNK = create_fanout('%s:FAN' % source, *pvs))


minmax_pvs('ADC')
WaveformOut('ADC:OFFSET', CHANNEL_COUNT, 'SHORT', DESC = 'ADC offsets')
WaveformOut('ADC:FILTER', 12, 'FLOAT', DESC = 'ADC compensation filter')
mbbOut('ADC:FILTER:DELAY', '-2 ns', '0 ns', '+2 ns',
    DESC = 'Compensation filter group delay')
mbbIn('ADC:SKEW', '0 ns', '2 ns', '4 ns', '6 ns',
    SCAN = 'I/O Intr', DESC = 'Input skew')
aOut('ADC:LIMIT', 0, 2 - 2**-15, PREC = 4, DESC = 'ADC error threshold')


minmax_pvs('DAC')
longOut('DAC:DELAY', 0, BUNCHES_PER_TURN-1, DESC = 'DAC output delay')
boolOut('DAC:ENABLE', 'Off', 'On', ZSV = 'MAJOR', DESC = 'DAC output enable')

WaveformOut('DAC:PREEMPH', 3, 'FLOAT', DESC = 'DAC output pre-emphasis')
mbbOut('DAC:PREEMPH:DELAY', '-2 ns', '0 ns', '+2 ns',
    DESC = 'Pre-emphasis group delay')
