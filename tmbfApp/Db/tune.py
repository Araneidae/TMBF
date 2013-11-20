# Tune control

from common import *


# Common controls for simple tune control
setting_changed = Action('TUNE:CHANGED', DESC = 'Record tune settings changed')
longOut('TUNE:HARMONIC', 0, SAMPLES_PER_TURN/2-1,
    FLNK = setting_changed, DESC = 'Select tune harmonic')
aOut('TUNE:CENTRE', 0, 1, PREC = 4,
    FLNK = setting_changed, DESC = 'Centre tune frequency')
aOut('TUNE:RANGE', 0, 0.5, PREC = 4,
    FLNK = setting_changed, DESC = 'Tune sweep range')
longOut('TUNE:BUNCH', 0, SAMPLES_PER_TURN-1,
    FLNK = setting_changed, DESC = 'Single bunch selection')


Action('TUNE:SET', DESC = 'Set tune settings')
boolIn('TUNE:SETTING', 'Changed', 'As Set',
    ZSV = 'MINOR', OSV = 'NO_ALARM', SCAN = 'I/O Intr',
    DESC = 'Tune setup state check')

Trigger('TUNE',
    # Readouts for the selected tune control.  These are copies of the
    # appropriate detector waveforms.
    Waveform('TUNE:I', TUNE_LENGTH, 'SHORT', DESC = 'Tune I'),
    Waveform('TUNE:Q', TUNE_LENGTH, 'SHORT', DESC = 'Tune Q'),
    Waveform('TUNE:POWER', TUNE_LENGTH, 'LONG', DESC = 'Tune power'),

    # Tune measurement
    aIn('TUNE:TUNE', 0, 1, PREC = 4, DESC = 'Measured tune'),
    aIn('TUNE:PHASE', -180, 180, 'deg', PREC = 1, DESC = 'Measured tune phase'),
    mbbIn('TUNE:STATUS',
        ('Invalid', 0,  'INVALID'),
        ('Ok',      1,  'NO_ALARM'))
)
