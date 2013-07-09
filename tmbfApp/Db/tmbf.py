import sys

# It is important to import support before importing iocbuilder, as the support
# module initialises iocbuilder (and determines which symbols it exports!)
from support import *
from iocbuilder import *


SAMPLES_PER_TURN = 936

MAX_FIR_TAPS = 10


# Bunch number
longOut('BUNCH', 0, 1023, VAL = 1023)
longOut('NCO')

status = longIn('STATUS')

WaveformOut('COEFFS', MAX_FIR_TAPS)
Waveform('COEFFSRB', MAX_FIR_TAPS)

WaveformOut('BB_GAINS', SAMPLES_PER_TURN, 'FLOAT')
WaveformOut('BB_DACS',  SAMPLES_PER_TURN, 'SHORT')
WaveformOut('BB_TEMPDACS',  SAMPLES_PER_TURN, 'SHORT')



hb_buf_upper = Waveform('HB_BUF_UPPER', 16384, 'SHORT',
    FLNK = status,
    DESC = 'BRAM buffer upper 16-bits')
hb_buf_lower = Waveform('HB_BUF_LOWER', 16384, 'SHORT',
    DESC = 'BRAM buffer lower 16-bits',
    PINI = 'YES',   FLNK = hb_buf_upper)

mbbOut('FIRGAIN',
    VAL  = 15,  FLNK = status,
    DESC = 'FIR gain select',
    *['%sdB' % db for db in range(21, -25, -3)])
mbbOut('HOMGAIN',
    VAL  = 15,  FLNK = status,
    DESC = 'HOM gain select',
    *['%sdB' % db for db in range(0, -46, -3)])
mbbOut('ARCHIVE',  'ADC+FIR', 'I+Q', 'DAC+FIR', 'N/A',
    VAL  = 1,   FLNK = status,
    DESC = 'Signal to archive')
mbbOut('CHSELECT', 'CH-A', 'CH-B', 'CH-C', 'CH-D',
    VAL  = 0,   FLNK = status,
    DESC = 'Readback channel select')
mbbOut('TRIGSEL',  'Soft', 'External',
    VAL  = 0,   FLNK = status,
    DESC = 'Trigger source select')
mbbOut('ARMSEL',   'Soft', 'External',
    VAL  = 0,   FLNK = status,
    DESC = 'Arm source select')
mbbOut('DDCINPUT',  'FIR', 'ADC',
    VAL  = 1,   FLNK = status,
    DESC = 'DDC input select')

longOut('FIRCYCLES', 1, MAX_FIR_TAPS,
    DESC = 'Cycles in filter')
longOut('FIRLENGTH', 1, MAX_FIR_TAPS,
    DESC = 'Length of filter')
aOut('FIRPHASE', -360, 360, VAL  = 0,
    DESC = 'FIR phase')

aOut('HOMFREQ', 0, 468, VAL  = 0.3, PREC = 4,
    DESC = 'NCO freq (tune)')

mbbOut('GROWDAMPMODE',
    'Off', 'On',  VAL  = 0,   FLNK = status,
    DESC = 'Grow/Damp enable')

longOut('IQSCALE', 0, 7, VAL  = 0, FLNK = status,
    DESC = 'IQ output scaling')
longOut('GROWDAMPPERIOD', 0, 255, VAL  = 1,
    DESC = 'Grow damp period')
longOut('PROGCLKVAL', VAL  = 12500,
    DESC = 'Programmable clock value')

mbbOut('TUNESWEEPMODE',
    'Off', 'On', VAL  = 0,
    DESC = 'Tune sweep enable')

# Processing the soft trigger will perform a triggered data capture.
softtrig = boolOut('SOFTTRIG', 'Trigger', DESC = 'Soft trigger')
# Processing the arm record will enable a hardware data capture.
mbbOut('SOFTARM', 'Arm', DESC = 'Soft Arm')

boolOut('BUNCHSYNC',
    'Sync', FLNK = status,
    DESC = 'Soft Arm')


# -----------------------------------------------------------------------------
# Tune measurement.


tunescale = Waveform('TUNESCALE', 4096, 'FLOAT',
    DESC = 'Scale for tune measurement')
startfreq = aOut('SWPSTARTFREQ', 0, 468, VAL  = 0.05, PREC = 4,
    DESC = 'Sweep start freq', MDEL = -1, FLNK = tunescale)
stopfreq  = aOut('SWPSTOPFREQ',  0, 468, VAL  = 0.45, PREC = 4,
    DESC = 'Sweep stop freq', MDEL = -1, FLNK = tunescale)
freqstep = aOut('SWPFREQSTEP',  VAL  = 1,  PREC = 4,
    DESC = 'Phase advance step', FLNK = tunescale)
longOut('DDCSKEW',
    VAL = 32, DESC = 'Direct update of DDC skew', FLNK = tunescale)

records.calcout('SWPFREQSTEP_C',
    CALC = '(B-A)/4096',
    INPA = CP(startfreq),
    INPB = CP(stopfreq),
    OUT  = PP(freqstep),
    OOPT = 'Every Time')


#
# The TUNESCAN record is designed to be configured to automatically scan.  On
# each scan it processes the dual buffer (to ensure current IQ data is
# captured) and then converts this to power and tune measurements.  Finally
# the soft trigger is fired to trigger another round of capture.
tune_records = [
    # Read the IQ data from the internal buffer
    hb_buf_lower,
    # Prepare the tune measurement data
    boolOut('PROCESS_TUNE'),
    # Update the I and Q waveforms
    Waveform('DDC_I', 4096,
        DESC = 'DDC response, I component'),
    Waveform('DDC_Q', 4096,
        DESC = 'DDC response, Q component'),
    # Compute the tune power spectrum
    Waveform('TUNEPOWER', 4096,
        DESC = 'DDC power spectrum'),
    # Compute the peak power and return this as the tune
    aIn('TUNE', PREC = 4,
        DESC = 'Measured tune'),
    aIn('TUNEPHASE', PREC = 2, EGU = 'deg',
        DESC = 'Phase at tune'),
    # Compute the cumulative sum of tune power
    Waveform('RAWCUMSUM_I', 4096,
        DESC = 'DDC cumulative sum, I part'),
    Waveform('RAWCUMSUM_Q', 4096,
        DESC = 'DDC cumulative sum, Q part'),
    Waveform('CUMSUM_I', 4096,
        DESC = 'DDC cumulative sum, I part'),
    Waveform('CUMSUM_Q', 4096,
        DESC = 'DDC cumulative sum, Q part'),
    # Now compute the tune and phase from the cumulative sum
    aIn('CUMSUMTUNE', PREC = 5,
        DESC = 'Measured tune using cumsum'),
    aIn('CUMSUMPHASE', PREC = 2, EGU = 'deg',
        DESC = 'Phase at cumsum tune'),

    # Finally trigger capture of the next round of data.
    softtrig]
create_fanout('TUNESCAN', SCAN = '1 second', *tune_records)


# ------------------------------------------------------------------------------
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
    boolOut('%s:SCAN' % source,
        DESC = 'Trigger %s scanning' % source, SCAN = '.2 second',
        FLNK = create_fanout('%s:FAN' % source,
            Waveform('%s:MINBUF' % source, SAMPLES_PER_TURN, 'SHORT',
                DESC = '%s min value per bunch' % source),
            Waveform('%s:MAXBUF' % source, SAMPLES_PER_TURN, 'SHORT',
                DESC = '%s max value per bunch' % source),
            Waveform('%s:DIFFBUF' % source, SAMPLES_PER_TURN, 'SHORT',
                DESC = '%s max min diff per bunch' % source),
            aIn('%s:MEAN' % source, PREC = 2,
                DESC = 'Readback %s diff mean' % source),
            aIn('%s:STD' % source, PREC = 2,
                DESC = 'Readback %s diff variance' % source),
            source_max, source_maxpc))


minmax_pvs('ADC')
WaveformOut('ADC:OFFSET', 4, 'SHORT', DESC = 'ADC offsets')


minmax_pvs('DAC')
longOut('DAC:DELAY', 4, 935, DESC = 'DAC output delay')
mbbOut('DAC:ENABLE', 'Off', 'On', DESC = 'DAC output enable')




# ------------------------------------------------------------------------------
# FIR

# There are four banks of FIR coefficients, each can either be written directly
# as a waveform or by separately controlling phase and fractional frequency.
for bank in range(1, 5):
    boolOut('FIR:%d:USEWF' % bank, 'Settings', 'Waveform',
        DESC = 'Use direct waveform or settings')

    # Direct settings of FIR parameters
    reload = boolOut('FIR:%d:RELOAD' % bank, DESC = 'Reload filter')
    longOut('FIR:%d:LENGTH' % bank, 2, MAX_FIR_TAPS,
        FLNK = reload, DESC = 'Length of filter')
    longOut('FIR:%d:CYCLES' % bank, 1, MAX_FIR_TAPS,
        FLNK = reload, DESC = 'Cycles in filter')
    aOut('FIR:%d:PHASE' % bank, -360, 360,
        FLNK = reload, DESC = 'FIR phase')

    # Waveform taps in two forms: TAPS_S is what is set directly as a waveform
    # write, TAPS is what is current loaded.
    WaveformOut('FIR:%d:TAPS_S' % bank, MAX_FIR_TAPS,
        DESC = 'Set waveform taps')
    Waveform('FIR:%d:TAPS' % bank, MAX_FIR_TAPS,
        SCAN = 'I/O Intr', DESC = 'Current waveform taps')


mbbOut('FIR:GAIN',
    DESC = 'FIR gain select',
    *['%sdB' % db for db in range(21, -25, -3)])


# ------------------------------------------------------------------------------
# DDR


SHORT_TURN_WF_COUNT = 8
LONG_TURN_WF_COUNT = 256

BUFFER_TURN_COUNT = 16 * 1024 * 1024 // SAMPLES_PER_TURN // 2
SHORT_TURN_WF_LENGTH = SAMPLES_PER_TURN * SHORT_TURN_WF_COUNT
LONG_TURN_WF_LENGTH = SAMPLES_PER_TURN * LONG_TURN_WF_COUNT

mbbOut('DDR:INPUT', 'ADC', 'DAC',
    DESC = 'DDR input select')

trig_mode = mbbOut('DDR:TRIGMODE', 'Retrigger', 'One Shot',
    DESC = 'DDR retrigger enable')

ddr_bunch_buffer = Waveform('DDR:BUNCHWF', BUFFER_TURN_COUNT, 'SHORT',
    DESC = 'Single bunch waveform')
longOut('DDR:BUNCHSEL', 0, 935,
    FLNK = ddr_bunch_buffer,
    DESC = 'Select bunch for DDR readout')

boolIn('DDR:TRIG', SCAN = 'I/O Intr',
    FLNK = create_fanout('FAN',
        ddr_bunch_buffer,
        Waveform('DDR:SHORTWF', SHORT_TURN_WF_LENGTH, 'SHORT',
            DESC = 'Short turn by turn waveform'),
        boolOut('DDR:TRIGDONE', DESC = 'DDR trigger done')),
    DESC = 'DDR trigger event')


# The long waveform and its records are only processed in single shot mode.
turnsel_readback = longIn('DDR:TURNSEL', DESC = 'Turn selection readback')
longOut('DDR:TURNSEL', -BUFFER_TURN_COUNT, BUFFER_TURN_COUNT,
    DESC = 'Select start turn for readout')

long_ddr_turn_buffer = Waveform('DDR:LONGWF', LONG_TURN_WF_LENGTH, 'SHORT',
    SCAN = 'I/O Intr',
    FLNK = turnsel_readback,
    DESC = 'Long turn by turn waveform')
boolIn('DDR:READY', 'Not Ready', 'Triggered',
    SCAN = 'I/O Intr', DESC = 'Long buffer status')


boolOut('DDR:ARM', DESC = 'DDR arm trigger')
boolOut('DDR:SOFT_TRIG', DESC = 'DDR soft trigger')



# ------------------------------------------------------------------------------

stringIn('VERSION', PINI = 'YES', DESC = 'TMBF version')

WriteRecords(sys.argv[1])
