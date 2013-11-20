# SE

from common import *


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

    boolIn('SE:ADCCLK', 'Clock Ok', 'Clock Dropout',
        ZSV  = 'NO_ALARM', OSV = 'MAJOR', DESC = 'ADC clock dropout detect'),

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



# Overflow detection PVs
def overflow(name, desc):
    return boolIn(name, 'Ok', 'Overflow', OSV = 'MAJOR', DESC = desc)
overflows = [
    overflow('SE:OVF:FIR', 'FIR overflow'),
    overflow('SE:OVF:DAC', 'DAC overflow'),
    overflow('SE:OVF:COMP', 'DAC pre-emphasis overflow')]
overflows.append(
    AggregateSeverity('SE:OVF', 'Numerical overflow', overflows))

boolOut('SE:OVF:SCAN',
    SCAN = '.1 second',
    FLNK = create_fanout('SE:OVF:FAN', *overflows),
    DESC = 'Overflow detect scan')
