# Tune control

from common import *

MAX_PEAKS = 3

PEAK_FIT_SIZE = 168


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
        Waveform('TUNE:PDD:%d' % suffix, TUNE_LENGTH/suffix, 'LONG',
            DESC = 'Second derivative of power'),
    ]

def tune_results(prefix, alias = None):
    tune = aIn('TUNE%s:TUNE' % prefix, 0, 1, PREC = 4, DESC = 'Measured tune')
    if alias is not None:
        tune.add_alias(alias)

    return [
        tune,
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

def peak_results(prefix):
    name = prefix.capitalize()
    return [
        aIn('TUNE:PEAK:%s' % prefix, 0, 1,
            PREC = 5, DESC = '%s peak frequency' % name),
        aIn('TUNE:PEAK:%s:PHASE' % prefix, -180, 180, 'deg',
            PREC = 1, DESC = '%s peak phase' % name),
        aIn('TUNE:PEAK:%s:AREA' % prefix,
            PREC = 3, DESC = '%s peak area' % name),
        aIn('TUNE:PEAK:%s:WIDTH' % prefix, 0, 1,
            PREC = 5, DESC = '%s peak width' % name),
        aIn('TUNE:PEAK:%s:HEIGHT' % prefix, DESC = '%s peak height' % name),
        boolIn('TUNE:PEAK:%s:VALID' % prefix, 'Invalid', 'Ok',
            ZSV = 'MINOR', DESC = '%s peak valid' % name),
    ]

def peak_rel_results(prefix):
    name = prefix.capitalize()
    return [
        aIn('TUNE:PEAK:%s:DTUNE' % prefix, 0, 1,
            PREC = 5, DESC = '%s delta tune' % name),
        aIn('TUNE:PEAK:%s:DPHASE' % prefix, -180, 180, 'deg',
            PREC = 1, DESC = '%s delta phase' % name),
        aIn('TUNE:PEAK:%s:RAREA' % prefix,
            PREC = 3, DESC = '%s relative area' % name),
        aIn('TUNE:PEAK:%s:RWIDTH' % prefix,
            PREC = 3, DESC = '%s relative width' % name),
        aIn('TUNE:PEAK:%s:RHEIGHT' % prefix,
            PREC = 3, DESC = '%s relative height' % name),
    ]


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

    # Waveforms for reading tune peak detection results
    Waveform('TUNE:PEAK:FIRSTFIT',
        PEAK_FIT_SIZE, 'CHAR', DESC = 'Raw first fit data'),
    Waveform('TUNE:PEAK:SECONDFIT',
        PEAK_FIT_SIZE, 'CHAR', DESC = 'Raw second fit data'),
    longIn('TUNE:PEAK:COUNT', DESC = 'Final fitted peak count'),

    # Synchrotron tune estimated from sidebands
    aIn('TUNE:PEAK:SYNCTUNE', 0, 1, PREC = 5, DESC = 'Synchrotron tune'),

*
    # Tune measurement.  We include an alias for :TUNE for backwards
    # compatibility.
    tune_results(':BASIC') +
    tune_results(':PEAK') +

    # Peak detection support
    peak_readbacks(16) + peak_readbacks(64) +
    peak_results('LEFT') + peak_results('CENTRE') + peak_results('RIGHT') +
    peak_rel_results('LEFT') + peak_rel_results('RIGHT')
)

Trigger('TUNE:RESULT', *tune_results('', '$(DEVICE):TUNE'))


mbbOut('TUNE:SELECT', 'Basic', 'Peak Fit', 'Tune PLL',
    DESC = 'Select tune measurement algorithm')

# Controls for tune peak finding
aOut('TUNE:THRESHOLD', 0, 1, PREC = 2, DESC = 'Peak fraction threshold')
longOut('TUNE:BLK:SEP', DESC = 'Minimum block separation')
longOut('TUNE:BLK:LEN', DESC = 'Minimum block length')

# Controls for new peak finding algorithm
aOut('TUNE:PEAK:THRESHOLD', 0, 1, PREC = 2,
    DESC = 'Fit data selection threshold')
aOut('TUNE:PEAK:MINWIDTH', 0, 1, PREC = 5, DESC = 'Minimum valid peak width')
aOut('TUNE:PEAK:MAXWIDTH', 0, 1, PREC = 5, DESC = 'Maximum valid peak width')
aOut('TUNE:PEAK:FITERROR', 0, 10, PREC = 3, DESC = 'Maximum fit error')

mbbOut('TUNE:PEAK:SEL', '/16', '/64', DESC = 'Select smoothing')

# Waveforms for injecting test data into tune measurement.
WaveformOut('TUNE:INJECT:I', TUNE_LENGTH, 'SHORT',
    PINI = 'NO', DESC = 'Test injection')
WaveformOut('TUNE:INJECT:Q', TUNE_LENGTH, 'SHORT',
    PINI = 'NO', DESC = 'Test injection')
WaveformOut('TUNE:INJECT:S', TUNE_LENGTH, 'DOUBLE',
    PINI = 'NO', DESC = 'Test injection')
Action('TUNE:INJECT', DESC = 'Process injected data')
