# Tune control

from common import *

MAX_EDGES = 20
MAX_PEAKS = 4


# Common controls for simple tune control
setting_changed = Action('TUNE:CHANGED', DESC = 'Record tune settings changed')
longOut('TUNE:HARMONIC', 0, BUNCHES_PER_TURN/2-1,
    FLNK = setting_changed, DESC = 'Select tune harmonic')
aOut('TUNE:CENTRE', 0, 1, PREC = 4,
    FLNK = setting_changed, DESC = 'Centre tune frequency')
aOut('TUNE:RANGE', 0, 0.5, PREC = 4,
    FLNK = setting_changed, DESC = 'Tune sweep range')
boolOut('TUNE:DIRECTION', 'Forwards', 'Backwards',
    FLNK = setting_changed, DESC = 'Sweep direction')
longOut('TUNE:BUNCH', 0, BUNCHES_PER_TURN-1,
    FLNK = setting_changed, DESC = 'Single bunch selection')
aOut('TUNE:ALARM', 0, 0.5, PREC = 4, DESC = 'Set tune alarm range')


Action('TUNE:SET', DESC = 'Set tune settings')
boolIn('TUNE:SETTING', 'Changed', 'As Set',
    ZSV = 'MINOR', OSV = 'NO_ALARM', SCAN = 'I/O Intr',
    DESC = 'Tune setup state check')


def peak_readbacks(suffix):
    return [
        Waveform('TUNE:POWER:%d' % suffix, TUNE_LENGTH/suffix, 'LONG',
            DESC = 'Smoothed tune'),
        Waveform('TUNE:EDGE:%d' % suffix, MAX_EDGES, 'LONG',
            DESC = 'Detected edges'),
        longIn('TUNE:ECOUNT:%d' % suffix, DESC = 'Number of edges'),
        Waveform('TUNE:PEAKIX:%d' % suffix, MAX_PEAKS, 'LONG',
            DESC = 'Peak indexes'),
        Waveform('TUNE:PEAKV:%d' % suffix, MAX_PEAKS, 'LONG',
            DESC = 'Peak values'),
        Waveform('TUNE:PEAKQ:%d' % suffix, MAX_PEAKS, 'FLOAT',
            DESC = 'Peak quality'),
    ]


tune_measurement = aIn('TUNE:TUNE', 0, 1, PREC = 4, DESC = 'Measured tune')
Trigger('TUNE',
    # Readouts for the selected tune control.  These are copies of the
    # appropriate detector waveforms.
    Waveform('TUNE:I', TUNE_LENGTH, 'SHORT', DESC = 'Tune I'),
    Waveform('TUNE:Q', TUNE_LENGTH, 'SHORT', DESC = 'Tune Q'),
    Waveform('TUNE:POWER', TUNE_LENGTH, 'LONG', DESC = 'Tune power'),
    Waveform('TUNE:PHASEWF', TUNE_LENGTH, 'FLOAT', DESC = 'Tune phase'),

    aIn('TUNE:MEANPOWER', DESC = 'Mean tune power'),
    longIn('TUNE:MAXPOWER', DESC = 'Maximum tune power'),

    # Cumsum waveforms for easy viewing of phase information
    Waveform('TUNE:CUMSUMI', TUNE_LENGTH, 'LONG', DESC = 'Cumsum I'),
    Waveform('TUNE:CUMSUMQ', TUNE_LENGTH, 'LONG', DESC = 'Cumsum Q'),

    # Tune measurement
    tune_measurement,
    aIn('TUNE:PHASE', -180, 180, 'deg', PREC = 1, DESC = 'Measured tune phase'),
    mbbIn('TUNE:STATUS',
        ('Invalid',     0,  'INVALID'),
        ('Ok',          1,  'NO_ALARM'),
        ('No peak',     2,  'MINOR'),
        ('Extra peaks', 3,  'MINOR'),
        ('Bad fit',     4,  'MINOR'),
        ('Overflow',    5,  'MAJOR'),
        ('Out of range', 6, 'NO_ALARM'),
        DESC = 'Status of last tune measurement'),

    # Peak detection support
    *peak_readbacks(4) + peak_readbacks(16) + peak_readbacks(64))

# Controls for tune peak finding
aOut('TUNE:THRESHOLD', 0, 1, PREC = 2, DESC = 'Peak fraction threshold')
longOut('TUNE:BLK:SEP', DESC = 'Minimum block separation')
longOut('TUNE:BLK:LEN', DESC = 'Minimum block length')

aOut('TUNE:PEAK:MIN', 0, 1, PREC = 2, DESC = 'Peak fraction threshold')
aOut('TUNE:PEAK:MAX', 0, 1, PREC = 2, DESC = 'Peak fraction threshold')


# Tune measurement alias for backwards compatibility
tune_measurement.add_alias('$(DEVICE):TUNE')
