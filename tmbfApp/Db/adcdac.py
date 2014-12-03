from common import *

# ADC + DAC
#
# Both of these systems are very similar: all we have are min/max waveforms and
# derived data (identical for both) and a couple of device specific controls.

def dB(db): return 10.**(db/20.)

def minmax_pvs(source, filter_length):
    # The following records are polled together at 200ms intervals
    pvs = [
        Waveform('%s:MINBUF' % source, BUNCHES_PER_TURN, 'FLOAT',
            DESC = '%s min value per bunch' % source),
        Waveform('%s:MAXBUF' % source, BUNCHES_PER_TURN, 'FLOAT',
            DESC = '%s max value per bunch' % source),
        Waveform('%s:DIFFBUF' % source, BUNCHES_PER_TURN, 'FLOAT',
            DESC = '%s max min diff per bunch' % source),
        aIn('%s:MEAN' % source, 0, 2, '', 5,
            DESC = 'Readback %s diff mean' % source),
        aIn('%s:STD' % source, 0, 1, '', 5,
            DESC = 'Readback %s diff variance' % source),
        aIn('%s:MAXDIFF' % source, 0, 2, '', 4,
            DESC = 'Readback %s max difference' % source),
        aIn('%s:MAX' % source, 0, 1, '', 4,
            DESC = 'Maximum %s reading' % source,
            HSV  = 'MINOR',  HIGH = dB(-3),     # 70.8 %
            HHSV = 'MAJOR',  HIHI = dB(-1.6)),  # 83.2 %
    ]
    Action('%s:SCAN' % source,
        DESC = 'Trigger %s scanning' % source, SCAN = '.2 second',
        FLNK = create_fanout('%s:FAN' % source, *pvs))

    # Also create PVs for controlling the compensation filter
    WaveformOut('%s:FILTER' % source, filter_length, 'FLOAT',
        DESC = '%s compensation filter' % source)
    mbbOut('%s:FILTER:DELAY' % source, '-2 ns', '0 ns', '+2 ns',
        DESC = '%s filter group delay' % source)



minmax_pvs('ADC', 12)
WaveformOut('ADC:OFFSET', CHANNEL_COUNT, 'LONG', DESC = 'ADC offsets')
mbbIn('ADC:SKEW', '0 ns', '2 ns', '4 ns', '6 ns',
    SCAN = 'I/O Intr', DESC = 'Input skew')
aOut('ADC:LIMIT', 0, 2 - 2**-15, PREC = 4, DESC = 'ADC error threshold')


minmax_pvs('DAC', 3)
longOut('DAC:DELAY', 0, BUNCHES_PER_TURN-1, DESC = 'DAC output delay')
boolOut('DAC:ENABLE', 'Off', 'On', ZSV = 'MAJOR', DESC = 'DAC output enable')
