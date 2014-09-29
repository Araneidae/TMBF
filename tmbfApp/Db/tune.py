# Tune control

from common import *


# Common controls for simple tune control
setting_changed = ForwardLink(
    'TUNE:CHANGED', 'Record tune settings changed',

    longOut('TUNE:HARMONIC', 0, BUNCHES_PER_TURN - 1,
        DESC = 'Select tune harmonic'),
    aOut('TUNE:CENTRE', 0, 1, PREC = 4,
        DESC = 'Centre tune frequency'),
    aOut('TUNE:RANGE', 0, 0.5, PREC = 4,
        DESC = 'Tune sweep range'),
    boolOut('TUNE:DIRECTION', 'Forwards', 'Backwards',
        DESC = 'Sweep direction'),
    longOut('TUNE:BUNCH', 0, BUNCHES_PER_TURN-1,
        DESC = 'Single bunch selection'),
    boolOut('TUNE:FEEDBACK', 'No feedback', 'Keep feedback',
        DESC = 'Whether to keep existing DAC out control'))

aOut('TUNE:ALARM', 0, 0.5, PREC = 4, DESC = 'Set tune alarm range')


Action('TUNE:SET', DESC = 'Set tune settings')
boolIn('TUNE:SETTING', 'Changed', 'As Set',
    ZSV = 'MINOR', OSV = 'NO_ALARM', SCAN = 'I/O Intr',
    DESC = 'Tune setup state check')


def tune_results(prefix, alias = None):
    tune = aIn('%s:TUNE' % prefix, 0, 1, PREC = 4, DESC = 'Measured tune')
    if alias is not None:
        tune.add_alias(alias)

    return [
        tune,
        aIn('%s:PHASE' % prefix, -180, 180, 'deg',
            PREC = 1, DESC = 'Measured tune phase'),
        mbbIn('%s:STATUS' % prefix,
            ('Invalid',     0,  'INVALID'),
            ('Ok',          1,  'NO_ALARM'),
            ('No peak',     2,  'MINOR'),
            ('Extra peaks', 3,  'MINOR'),
            ('Bad fit',     4,  'MINOR'),
            ('Overflow',    5,  'MAJOR'),
            ('Out of range', 6, 'NO_ALARM'),
            DESC = 'Status of last tune measurement'),
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

    # Tune measurement.  We include an alias for :TUNE for backwards
    # compatibility.
    *tune_results('TUNE:BASIC') +
     tune_results('PEAK')
)

Trigger('TUNE:RESULT', *tune_results('TUNE', '$(DEVICE):TUNE'))


mbbOut('TUNE:SELECT', 'Basic', 'Peak Fit', 'Tune PLL',
    DESC = 'Select tune measurement algorithm')

# Controls for tune peak finding
aOut('TUNE:THRESHOLD', 0, 1, PREC = 2, DESC = 'Peak fraction threshold')
longOut('TUNE:BLK:SEP', DESC = 'Minimum block separation')
longOut('TUNE:BLK:LEN', DESC = 'Minimum block length')

# Waveforms for injecting test data into tune measurement.
WaveformOut('TUNE:INJECT:I', TUNE_LENGTH, 'SHORT',
    PINI = 'NO', DESC = 'Test injection')
WaveformOut('TUNE:INJECT:Q', TUNE_LENGTH, 'SHORT',
    PINI = 'NO', DESC = 'Test injection')
WaveformOut('TUNE:INJECT:S', TUNE_LENGTH, 'DOUBLE',
    PINI = 'NO', DESC = 'Test injection')
Action('TUNE:INJECT', DESC = 'Process injected data')
