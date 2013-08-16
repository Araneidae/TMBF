from common import *

# ADC + DAC
#
# Both of these systems are very similar: all we have are min/max waveforms and
# derived data (identical for both) and a couple of device specific controls.

MAX_ADC = (1<<13) - 1     # Signed 14-bit ADC, range [-8192..8191]

def dB(db): return 10.**(db/20.)

def minmax_pvs(source):
    # The following records are polled together at 200ms intervals
    source_max = longIn('%s:MAX' % source, 0, MAX_ADC,
        DESC = 'Maximum %s reading' % source,
        HSV  = 'MINOR',  HIGH = int(MAX_ADC / dB(3)),    # 70.8 %
        HHSV = 'MAJOR',  HIHI = int(MAX_ADC / dB(1.6)))  # 83.2 %
    source_maxpc = records.calc('%s:MAX_PC' % source,
        DESC = 'Maximum %s reading (%%)' % source,
        CALC = 'A/B',
        INPA = MS(source_max),
        INPB = MAX_ADC / 100.,
        LOPR = 0,   HOPR = 100,
        PREC = 1,   EGU  = '%')
    pvs = [
        Waveform('%s:MINBUF' % source, SAMPLES_PER_TURN, 'SHORT',
            DESC = '%s min value per bunch' % source),
        Waveform('%s:MAXBUF' % source, SAMPLES_PER_TURN, 'SHORT',
            DESC = '%s max value per bunch' % source),
        Waveform('%s:DIFFBUF' % source, SAMPLES_PER_TURN, 'SHORT',
            DESC = '%s max min diff per bunch' % source),
        aIn('%s:MEAN' % source, -4096, 4096, '', 2,
            DESC = 'Readback %s diff mean' % source),
        aIn('%s:STD' % source, 0, 4096, '', 2,
            DESC = 'Readback %s diff variance' % source),
        source_max, source_maxpc]
    Action('%s:SCAN' % source,
        DESC = 'Trigger %s scanning' % source, SCAN = '.2 second',
        FLNK = create_fanout('%s:FAN' % source, *pvs))


minmax_pvs('ADC')
WaveformOut('ADC:OFFSET', 4, 'SHORT', DESC = 'ADC offsets')


minmax_pvs('DAC')
longOut('DAC:DELAY', 0, 935, DESC = 'DAC output delay')
mbbOut('DAC:ENABLE', 'Off', 'On', DESC = 'DAC output enable')
