# Definitions for fast tune following

from common import *

longOut('FTUN:DWELL', 1, 1<<16, DESC = 'Tune following dwell time')
boolOut('FTUN:ENABLE', 'Disabled', 'Enabled', DESC = 'Enable tune following')

longOut('FTUN:BUNCH', 0, BUNCHES_PER_TURN-1, DESC = 'Tune following bunch')
boolOut('FTUN:MULTIBUNCH', 'Single Bunch', 'Multi-Bunch',
    DESC = 'Tune following bunch mode')
mbbOut('FTUN:INPUT', 'ADC', 'FIR', DESC = 'Tune following input selection')
aOut('FTUN:ANGLE', -360, 360, 'deg', DESC = 'Phase rotation')
