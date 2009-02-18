import sys

# It is important to import support before importing iocbuilder, as the
# support module initialises iocbuilder (and determines which symbols it
# exports!)
from support import * 
from iocbuilder import *


# Parameters for this file
DACDLY = Parameter('DACDLY')
FIR_CYCLES = Parameter('FIR_CYCLES')
FIR_LENGTH = Parameter('FIR_LENGTH')
DDC_SKEW = Parameter('DDC_SKEW')


# Bunch number
longInOut('BUNCH', 0, 1023, VAL = 1023)
longInOut('NCO')
longInOut('ADC_OFF_AB')
longInOut('ADC_OFF_CD')

status = longIn('STATUS_R')

wInOut('COEFFS', 12)

Waveform('BB_GAINS_W', 936, 'FLOAT')
Waveform('BB_DACS_W',  936, 'SHORT')

# The following records are polled together at 200ms intervals
create_fanout('SCAN_ADC', 
    Waveform('ADC_MINBUF_R', 936, 'SHORT',
        DESC = 'ADC min value per bunch'),
    Waveform('ADC_MAXBUF_R', 936, 'SHORT',
        DESC = 'ADC max value per bunch'),
    Waveform('ADC_DIFFBUF_R', 936, 'SHORT',
        DESC = 'ADC max min diff per bunch'),
    aIn('ADCMEAN_R', PREC = 2,
        DESC = 'Readback ADC diff mean'),
    aIn('ADCSTD_R', PREC = 2,
        DESC = 'Readback ADC diff variance'),
    SCAN = '.2 second')

create_fanout('SCAN_DAC', 
    Waveform('DAC_MINBUF_R', 936, 'SHORT',
        DESC = 'DAC min value per bunch'),
    Waveform('DAC_MAXBUF_R', 936, 'SHORT',
        DESC = 'DAC max value per bunch'),
    Waveform('DAC_DIFFBUF_R', 936, 'SHORT',
        DESC = 'DAC max min diff per bunch'),
    aIn('DACMEAN_R', PREC = 2,
        DESC = 'Readback DAC diff mean'),
    aIn('DACSTD_R', PREC = 2,
        DESC = 'Readback DAC diff variance'),
    SCAN = 'Passive')


    
hb_buf_upper = Waveform('HB_BUF_UPPER_R', 16384, 'SHORT',
    FLNK = status,
    DESC = 'BRAM buffer upper 16-bits')
hb_buf_lower = Waveform('HB_BUF_LOWER_R', 16384, 'SHORT',
    DESC = 'BRAM buffer lower 16-bits',
    PINI = 'YES',   FLNK = hb_buf_upper)

mbbOut('DACOUT_S', 'Off', 'FIR', 'HOM+FIR', 'HOM', 
    VAL  = 0,   FLNK = status,
    DESC = 'DAC output')
mbbOut('TEMPDACOUT_S', 'Off', 'FIR', 'HOM+FIR', 'HOM', 
    VAL  = 0,   FLNK = status,
    DESC = 'DAC output')
mbbOut('FIRGAIN_S',
    VAL  = 15,  FLNK = status,
    DESC = 'FIR gain select',
    *['%sdB' % db for db in range(21, -25, -3)])
mbbOut('FIRINVERT_S', 'Off', 'On',
    VAL  = 0,   FLNK = status,
    DESC = 'FIR output invert')
mbbOut('HOMGAIN_S',
    VAL  = 15,  FLNK = status,
    DESC = 'HOM gain select',
    *['%sdB' % db for db in range(0, -46, -3)])
mbbOut('ARCHIVE_S',  'ADC+FIR', 'I+Q', 'DAC+FIR', 'N/A',
    VAL  = 1,   FLNK = status,
    DESC = 'Signal to archive')
mbbOut('CHSELECT_S', 'CH-A', 'CH-B', 'CH-C', 'CH-D',
    VAL  = 0,   FLNK = status,
    DESC = 'Readback channel select')
mbbOut('TRIGSEL_S',  'Soft', 'External',
    VAL  = 0,   FLNK = status,
    DESC = 'Trigger source select')
mbbOut('ARMSEL_S',   'Soft', 'External',
    VAL  = 0,   FLNK = status,
    DESC = 'Arm source select')
mbbOut('DDCINPUT_S',  'FIR', 'ADC',
    VAL  = 1,   FLNK = status,
    DESC = 'DDC input select')
mbbOut('DDRINPUT_S',  'ADC', 'DAC',
    VAL  = 0,   FLNK = status,
    DESC = 'DDR input select')

MAX_FIR_TAPS = 9
longOut('FIRCYCLES_S', 1, MAX_FIR_TAPS,
    VAL  = FIR_CYCLES,  PINI = 'YES',
    DESC = 'Cycles in filter')
longOut('FIRLENGTH_S', 1, MAX_FIR_TAPS,
    VAL  = FIR_LENGTH,  PINI = 'YES',
    DESC = 'Length of filter')
aOut('FIRPHASE_S', -360, 360, VAL  = 0,   
    DESC = 'FIR phase')

dacdly_s = longOut('DACDLY_S', 4, 935,
    VAL  = DACDLY,  PINI = 'YES',
    DESC = 'DAC output delay')

aOut('HOMFREQ_S', 0, 468, VAL  = 0.3, PREC = 4,
    DESC = 'NCO freq (tune)')

mbbOut('GROWDAMPMODE_S',
    'Off', 'On',  VAL  = 0,   FLNK = status,
    DESC = 'Grow/Damp enable')
mbbOut('BUNCHMODE_S',
    'Off', 'On',  VAL  = 0,   FLNK = status)

longOut('IQSCALE_S', 0, 7, VAL  = 0, FLNK = status,
    DESC = 'IQ output scaling')
longOut('GROWDAMPPERIOD_S', 0, 255, VAL  = 1,   
    DESC = 'Grow damp period')
longOut('PROGCLKVAL_S', VAL  = 12500,   
    DESC = 'Programmable clock value')

mbbOut('TUNESWEEPMODE_S',
    'Off', 'On', VAL  = 0,   
    DESC = 'Tune sweep enable')

# Processing the soft trigger will perform a triggered data capture.
softtrig = boolOut('SOFTTRIG_S', 'Trigger', DESC = 'Soft trigger')
# Processing the arm record will enable a hardware data capture.
mbbOut('SOFTARM_S', 'Arm', DESC = 'Soft Arm')

boolOut('BUNCHSYNC_S',
    'Sync', FLNK = status,
    DESC = 'Soft Arm')


# -----------------------------------------------------------------------------
# Tune measurement.


tunescale = Waveform('TUNESCALE', 4096, 'FLOAT',
    DESC = 'Scale for tune measurement')
startfreq = aOut('SWPSTARTFREQ_S', 0, 468, VAL  = 0.05, PREC = 4,
    DESC = 'Sweep start freq', MDEL = -1, FLNK = tunescale)
stopfreq  = aOut('SWPSTOPFREQ_S',  0, 468, VAL  = 0.45, PREC = 4,
    DESC = 'Sweep stop freq', MDEL = -1, FLNK = tunescale)
freqstep = aOut('SWPFREQSTEP_S',  VAL  = 1,  PREC = 4,
    DESC = 'Phase advance step', FLNK = tunescale)
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
create_fanout('TUNESCAN', SCAN = 'Passive', *tune_records)

longOut('DDCSKEW_S',
    VAL = DDC_SKEW, PINI = 'YES',
    DESC = 'Direct update of DDC skew')


WriteRecords(sys.argv[1])
