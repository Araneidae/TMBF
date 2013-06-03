import sys

# It is important to import support before importing iocbuilder, as the
# support module initialises iocbuilder (and determines which symbols it
# exports!)
from support import *
from iocbuilder import *


# Bunch number
longOut('BUNCH', 0, 1023, VAL = 1023)
longOut('NCO')
longOut('ADC_OFF_AB')
longOut('ADC_OFF_CD')

status = longIn('STATUS')

WaveformOut('COEFFS', 12)

WaveformOut('BB_GAINS', 936, 'FLOAT')
WaveformOut('BB_DACS',  936, 'SHORT')

# The following records are polled together at 200ms intervals
create_fanout('SCAN_ADC',
    Waveform('ADC_MINBUF', 936, 'SHORT',
        DESC = 'ADC min value per bunch'),
    Waveform('ADC_MAXBUF', 936, 'SHORT',
        DESC = 'ADC max value per bunch'),
    Waveform('ADC_DIFFBUF', 936, 'SHORT',
        DESC = 'ADC max min diff per bunch'),
    aIn('ADCMEAN', PREC = 2,
        DESC = 'Readback ADC diff mean'),
    aIn('ADCSTD', PREC = 2,
        DESC = 'Readback ADC diff variance'),
    SCAN = '.2 second')

create_fanout('SCAN_DAC',
    Waveform('DAC_MINBUF', 936, 'SHORT',
        DESC = 'DAC min value per bunch'),
    Waveform('DAC_MAXBUF', 936, 'SHORT',
        DESC = 'DAC max value per bunch'),
    Waveform('DAC_DIFFBUF', 936, 'SHORT',
        DESC = 'DAC max min diff per bunch'),
    aIn('DACMEAN', PREC = 2,
        DESC = 'Readback DAC diff mean'),
    aIn('DACSTD', PREC = 2,
        DESC = 'Readback DAC diff variance'),
    SCAN = 'Passive')



hb_buf_upper = Waveform('HB_BUF_UPPER', 16384, 'SHORT',
    FLNK = status,
    DESC = 'BRAM buffer upper 16-bits')
hb_buf_lower = Waveform('HB_BUF_LOWER', 16384, 'SHORT',
    DESC = 'BRAM buffer lower 16-bits',
    PINI = 'YES',   FLNK = hb_buf_upper)

mbbOut('DACOUT', 'Off', 'FIR', 'HOM+FIR', 'HOM',
    VAL  = 0,   FLNK = status,
    DESC = 'DAC output')
mbbOut('TEMPDACOUT', 'Off', 'FIR', 'HOM+FIR', 'HOM',
    VAL  = 0,   FLNK = status,
    DESC = 'DAC output')
mbbOut('FIRGAIN',
    VAL  = 15,  FLNK = status,
    DESC = 'FIR gain select',
    *['%sdB' % db for db in range(21, -25, -3)])
mbbOut('FIRINVERT', 'Off', 'On',
    VAL  = 0,   FLNK = status,
    DESC = 'FIR output invert')
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
mbbOut('DDRINPUT',  'ADC', 'DAC',
    VAL  = 0,   FLNK = status,
    DESC = 'DDR input select')

MAX_FIR_TAPS = 9
longOut('FIRCYCLES', 1, MAX_FIR_TAPS,
    VAL  = 2, DESC = 'Cycles in filter')
longOut('FIRLENGTH', 1, MAX_FIR_TAPS,
    VAL  = 9, DESC = 'Length of filter')
aOut('FIRPHASE', -360, 360, VAL  = 0,
    DESC = 'FIR phase')

dacdly_s = longOut('DACDLY', 4, 935,
    VAL  = 620, DESC = 'DAC output delay')

aOut('HOMFREQ', 0, 468, VAL  = 0.3, PREC = 4,
    DESC = 'NCO freq (tune)')

mbbOut('GROWDAMPMODE',
    'Off', 'On',  VAL  = 0,   FLNK = status,
    DESC = 'Grow/Damp enable')
mbbOut('BUNCHMODE',
    'Off', 'On',  VAL  = 0,   FLNK = status)

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

stringIn('VERSION', PINI = 'YES', DESC = 'TMBF version')

WriteRecords(sys.argv[1])
