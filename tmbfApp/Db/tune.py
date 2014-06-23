# Tune control

from common import *

MAX_PEAKS = 4


# Common controls for simple tune control
setting_changed = Action('TUNE:CHANGED', DESC = 'Record tune settings changed')
longOut('TUNE:HARMONIC', 0, BUNCHES_PER_TURN - 1,
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
        Waveform('TUNE:PEAKIX:%d' % suffix, MAX_PEAKS, 'LONG',
            DESC = 'Peak indexes'),
        Waveform('TUNE:PEAKV:%d' % suffix, MAX_PEAKS, 'LONG',
            DESC = 'Peak values'),
        Waveform('TUNE:PEAKL:%d' % suffix, MAX_PEAKS, 'LONG',
            DESC = 'Peak left'),
        Waveform('TUNE:PEAKR:%d' % suffix, MAX_PEAKS, 'LONG',
            DESC = 'Peak right'),
        Waveform('TUNE:PEAKQ:%d' % suffix, MAX_PEAKS, 'FLOAT',
            DESC = 'Peak quality'),
        Waveform('TUNE:PEAKH:%d' % suffix, MAX_PEAKS, 'FLOAT',
            DESC = 'Peak height ratio'),
        Waveform('TUNE:PDD:%d' % suffix, TUNE_LENGTH/suffix, 'LONG',
            DESC = 'Second derivative of power'),
        longIn('TUNE:PEAKC:%d' % suffix, DESC = 'Peaks detected'),
    ]

def tune_results(prefix = ''):
    return [
        aIn('TUNE%s:TUNE' % prefix, 0, 1, PREC = 4, DESC = 'Measured tune'),
        aIn('TUNE%s:PHASE' % prefix, -180, 180, 'deg',
            PREC = 1, DESC = 'Measured tune phase'),
        mbbIn('TUNE%s:STATUS' % prefix,
            ('Invalid',     0,  'INVALID'),
            ('Ok',          1,  'NO_ALARM'),
            ('No peak',     2,  'MINOR'),
            ('Extra peaks', 3,  'MINOR'),
            ('Bad fit',     4,  'MINOR'),
            ('Overflow',    5,  'MAJOR'),
            ('Out of range', 6, 'NO_ALARM'),
            DESC = 'Status of last tune measurement'),
    ]


# tune_measurement = aIn('TUNE:TUNE', 0, 1, PREC = 4, DESC = 'Measured tune')
tune_measurement = tune_results()
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
    *tune_measurement +

    # Peak detection support
    peak_readbacks(4) + peak_readbacks(16) + peak_readbacks(64) +
    tune_results(':PEAK'))

# Controls for tune peak finding
aOut('TUNE:THRESHOLD', 0, 1, PREC = 2, DESC = 'Peak fraction threshold')
longOut('TUNE:BLK:SEP', DESC = 'Minimum block separation')
longOut('TUNE:BLK:LEN', DESC = 'Minimum block length')

# Controls for new peak finding algorithm
aOut('TUNE:PEAK:MINQ', 0, 100, PREC = 1, DESC = 'Minimum peak quality')
aOut('TUNE:PEAK:MINH', 0, 1, PREC = 1, DESC = 'Minimum peak height')
aOut('TUNE:PEAK:MINR', 0, 100, PREC = 1, DESC = 'Minimum peak ratio')
aOut('TUNE:PEAK:FIT', 0, 1, PREC = 2, DESC = 'Fit threshold')
mbbOut('TUNE:PEAK:SEL', '/4', '/16', '/64', DESC = 'Select smoothing')

# Waveform for injecting test data into tune measurement.
WaveformOut('TUNE:INJECT:P', TUNE_LENGTH, 'LONG',
    PINI = 'NO', DESC = 'Test injection')


# Tune measurement alias for backwards compatibility
tune_measurement[0].add_alias('$(DEVICE):TUNE')
