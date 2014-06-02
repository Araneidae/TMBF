from common import *

# ADC + DAC
#
# Both of these systems are very similar: all we have are min/max waveforms and
# derived data (identical for both) and a couple of device specific controls.

# The ADC is scaled up with an extra two bits as a result of input processing
# and filtering.
MAX_ADC = (1<<15) - 1       # Signed 16-bit range [-32768..32767]
MAX_DAC = (1<<13) - 1       # Signed 14-bit range [-8192..8191]

def dB(db): return 10.**(db/20.)

def minmax_pvs(source, max_val):
    # The following records are polled together at 200ms intervals
    source_max = longIn('%s:MAX' % source, 0, max_val,
        DESC = 'Maximum %s reading' % source,
        HSV  = 'MINOR',  HIGH = int(max_val / dB(3)),    # 70.8 %
        HHSV = 'MAJOR',  HIHI = int(max_val / dB(1.6)))  # 83.2 %
    source_maxpc = records.calc('%s:MAX_PC' % source,
        DESC = 'Maximum %s reading (%%)' % source,
        CALC = 'A/B',
        INPA = MS(source_max),
        INPB = max_val / 100.,
        LOPR = 0,   HOPR = 100,
        PREC = 1,   EGU  = '%')
    pvs = [
        Waveform('%s:MINBUF' % source, BUNCHES_PER_TURN, 'SHORT',
            DESC = '%s min value per bunch' % source),
        Waveform('%s:MAXBUF' % source, BUNCHES_PER_TURN, 'SHORT',
            DESC = '%s max value per bunch' % source),
        Waveform('%s:DIFFBUF' % source, BUNCHES_PER_TURN, 'SHORT',
            DESC = '%s max min diff per bunch' % source),
        aIn('%s:MEAN' % source, -4096, 4096, '', 2,
            DESC = 'Readback %s diff mean' % source),
        aIn('%s:STD' % source, 0, 4096, '', 2,
            DESC = 'Readback %s diff variance' % source),
        source_max, source_maxpc]
    Action('%s:SCAN' % source,
        DESC = 'Trigger %s scanning' % source, SCAN = '.2 second',
        FLNK = create_fanout('%s:FAN' % source, *pvs))


minmax_pvs('ADC', MAX_ADC)
WaveformOut('ADC:OFFSET', CHANNEL_COUNT, 'SHORT', DESC = 'ADC offsets')
WaveformOut('ADC:FILTER', 12, 'SHORT', DESC = 'ADC compensation filter')
mbbOut('ADC:FILTER:DELAY', '-2 ns', '0 ns', '+2 ns',
    DESC = 'Compensation filter group delay')
mbbIn('ADC:SKEW', '0 ns', '2 ns', '4 ns', '6 ns',
    SCAN = 'I/O Intr', DESC = 'Input skew')
longOut('ADC:LIMIT', 0, MAX_ADC, DESC = 'ADC error threshold')


minmax_pvs('DAC', MAX_DAC)
longOut('DAC:DELAY', 0, BUNCHES_PER_TURN-1, DESC = 'DAC output delay')
boolOut('DAC:ENABLE', 'Off', 'On', ZSV = 'MAJOR', DESC = 'DAC output enable')

WaveformOut('DAC:PREEMPH', 3, 'SHORT', DESC = 'DAC output pre-emphasis')
mbbOut('DAC:PREEMPH:DELAY', '-2 ns', '0 ns', '+2 ns',
    DESC = 'Pre-emphasis group delay')
