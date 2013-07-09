import sys

# It is important to import support before importing iocbuilder, as the support
# module initialises iocbuilder (and determines which symbols it exports!)
from support import *
from iocbuilder import *


SAMPLES_PER_TURN = 936

MAX_FIR_TAPS = 10

KB = 1024
MB = KB * KB


def Trigger(prefix, *pvs):
    done = boolOut('%s:DONE' % prefix,
        PINI = 'NO', DESC = '%s processing done' % prefix)
    return boolIn('%s:TRIG' % prefix,
        SCAN = 'I/O Intr', DESC = '%s processing trigger' % prefix,
        FLNK = create_fanout('%s:TRIG:FAN' % prefix, *pvs + (done,)))


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
    aIn('TUNE', 0, 0.5, '', 4, DESC = 'Measured tune'),
    aIn('TUNEPHASE', -180, 180, 'deg', 2, DESC = 'Phase at tune'),
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
    aIn('CUMSUMTUNE', 0, 0.5, '', 5, DESC = 'Measured tune using cumsum'),
    aIn('CUMSUMPHASE', -180, 180, 'deg', 2, DESC = 'Phase at cumsum tune'),

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
    boolOut('%s:SCAN' % source,
        DESC = 'Trigger %s scanning' % source, SCAN = '.2 second',
        FLNK = create_fanout('%s:FAN' % source, *pvs))


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

Trigger('DDR',
    ddr_bunch_buffer,
    Waveform('DDR:SHORTWF', SHORT_TURN_WF_LENGTH, 'SHORT',
        DESC = 'Short turn by turn waveform'))


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
# SE

# Aggregates the severity of all the given records into a single record.  The
# value of the record is constant, but its SEVR value reflects the maximum
# severity of all of the given records.
def AggregateSeverity(name, description, recs):
    assert len(recs) <= 12, 'Too many records to aggregate'
    return records.calc(name,
        CALC = 1, DESC = description,
        # Assign each record of interest to a calc record input with MS.
        # This then automatically propagates to the severity of the whole
        # record.
        **dict([
            ('INP' + c, MS(r))
            for c, r in zip ('ABCDEFGHIJKL', recs)]))


trigger_pvs = []        # All sensor records that need triggering, in order
health_pvs = []         # Records for reporting aggregate health

# Fans and temperatures
fan_set = longIn('SE:FAN_SET', 4000, 6000, 'RPM', DESC = 'Fan set speed')
fan_temp_pvs = [fan_set]
for i in (1, 2):
    fan_speed = longIn('SE:FAN%d' % i,     4000, 6000, 'RPM',
        LOLO = 100,     LLSV = 'MAJOR', LOW  = 4000,    LSV  = 'MINOR',
        DESC = 'Fan %d speed' % i)
    fan_temp_pvs.extend([
        fan_speed,
        records.calc('SE:FAN%d_ERR' % i,
            DESC = 'Fan %d speed error' % i,
            CALC = 'A-B',   INPA = fan_speed,   INPB = fan_set,
            EGU  = 'RPM',
            LOLO = -1000,   LLSV = 'MAJOR', LOW  = -500,    LSV  = 'MINOR',
            HIGH = 500,     HSV  = 'MINOR', HIHI = 1000,    HHSV = 'MAJOR')])
fan_temp_pvs.append(
    # Motherboard temperature
    longIn('SE:TEMP', 30, 60, 'deg C',
        DESC = 'Motherboard temperature',
        HIGH = 55,    HSV  = 'MINOR',
        HIHI = 60,    HHSV = 'MAJOR'))

trigger_pvs.extend(fan_temp_pvs)
health_pvs.append(
    AggregateSeverity('SE:FAN:OK', 'Fan controller health', fan_temp_pvs))


system_alarm_pvs = [
    # System memory and CPU usage
    aIn('SE:FREE', 0, 64, 'MB', 2,
        DESC = 'Free memory',
        LOW  = 12,      LSV  = 'MINOR',
        LOLO = 8,       LLSV = 'MAJOR'),
    aIn('SE:RAMFS', 0, 64, 'MB', 3,
        DESC = 'Temporary file usage',
        HIGH = 1,       HSV  = 'MINOR',
        HIHI = 16,      HHSV = 'MAJOR'),
    aIn('SE:CPU', 0, 100, '%', 1,
        DESC = 'CPU usage',
        HIGH = 80,      HSV  = 'MINOR',
        HIHI = 95,      HHSV = 'MAJOR'),

    # The following list must match the corresponding enum in sensors.c
    mbbIn('SE:NTPSTAT',
        ('Not monitored',   0,  'NO_ALARM'),    # Monitoring disabled
        ('No NTP server',   1,  'MAJOR'),       # Local NTP server not found
        ('Startup',         2,  'NO_ALARM'),    # No alarm during startup
        ('No Sync',         3,  'MINOR'),       # NTP server not synchronised
        ('Synchronised',    4,  'NO_ALARM'),    # Synchronised to remote server
        DESC = 'Status of NTP server'),
    longIn('SE:STRATUM',
        LOW  = 0,   LSV  = 'MAJOR',         # Probably does not occur now
        HIGH = 16,  HSV  = 'MAJOR',         # Unspecified stratum
        DESC = 'NTP stratum level')]
trigger_pvs.extend(system_alarm_pvs)
health_pvs.append(
    AggregateSeverity('SE:SYS:OK', 'System health', system_alarm_pvs))


# Sensor PVs without alarm status.
trigger_pvs.extend([
    # Time since booting
    aIn('SE:UPTIME',  0, 24*365, 'h', 2, DESC = 'Total system up time'),
    aIn('SE:EPICSUP', 0, 24*365, 'h', 2, DESC = 'Time since EPICS started'),

    # Channel access counters
    longIn('SE:CAPVS',  DESC = 'Number of connected PVs'),
    longIn('SE:CACLNT', DESC = 'Number of connected clients'),

    # Network statistics
    aIn('SE:NWBRX', 0, 1e4, 'kB/s', 3,  DESC = 'Kilobytes received per second'),
    aIn('SE:NWBTX', 0, 1e4, 'kB/s', 3,  DESC = 'Kilobytes sent per second'),
    aIn('SE:NWPRX', 0, 1e4, 'pkt/s', 1, DESC = 'Packets received per second'),
    aIn('SE:NWPTX', 0, 1e4, 'pkt/s', 1, DESC = 'Packets sent per second'),
    aIn('SE:NWMRX', 0, 1e4, 'pkt/s', 1, DESC = 'Multicast received per second'),
    aIn('SE:NWMTX', 0, 1e4, 'pkt/s', 1, DESC = 'Multicast sent per second'),

    stringIn('SE:SERVER', DESC = 'Synchronised NTP server')])

# Aggregate all the alarm generating records into a single "health" record.
# Only the alarm status of this record is meaningful.
trigger_pvs.extend(health_pvs)
trigger_pvs.append(
    AggregateSeverity('SE:HEALTH', 'Aggregated health', health_pvs))

Trigger('SE', *trigger_pvs)

longOut('SE:TEMP', 30, 60, 'deg', DESC = 'Target temperature')
longOut('SE:TEMP:KP', DESC = 'Fan controller KP')
longOut('SE:TEMP:KI', DESC = 'Fan controller KI')


# ------------------------------------------------------------------------------

stringIn('VERSION', PINI = 'YES', DESC = 'TMBF version')

WriteRecords(sys.argv[1])
