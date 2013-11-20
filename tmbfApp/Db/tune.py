# Tune control

from common import *


# Common controls for simple tune control
longOut('TUNE:HARMONIC', 0, SAMPLES_PER_TURN/2-1, DESC = 'Select tune harmonic')
aOut('TUNE:CENTRE', 0, 1, DESC = 'Centre tune frequency')
aOut('TUNE:RANGE', 0, 0.5, DESC = 'Tune sweep range')
boolOut('TUNE:MODE', 'All Bunches', 'Single Bunch',
    DESC = 'Tune detection mode')
longOut('TUNE:BUNCH', 0, SAMPLES_PER_TURN-1, DESC = 'Single bunch selection')

# Readouts for the selected tune control.  These are copies of the appropriate
# detector waveforms.
Waveform('TUNE:I', TUNE_LENGTH, 'SHORT', DESC = 'Tune I')
Waveform('TUNE:Q', TUNE_LENGTH, 'SHORT', DESC = 'Tune Q')
Waveform('TUNE:POWER', TUNE_LENGTH, 'LONG', DESC = 'Tune power')

# Tune measurement
aIn('TUNE:TUNE', 0, 1, PREC = 4, DESC = 'Measured tune')
aIn('TUNE:PHASE', -180, 180, 'deg', PREC = 1, DESC = 'Measured tune phase')

Action('TUNE:SET', DESC = 'Set tune settings')
boolIn('TUNE:STATUS', 'Set', 'Changed',
    ZSV = 'NO_ALARM', OSV = 'MINOR', DESC = 'Tune setup state check')
