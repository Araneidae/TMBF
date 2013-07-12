# DET

from common import *



mbbOut('DET:GAIN', DESC = 'Detector gain', *dBrange(8, -6))
boolOut('DET:MODE', 'Single Bunch', 'All Bunches', DESC = 'Detector mode')

update_bunch = Action('DET:WBUNCH', DESC = 'Update detector bunches')
for bunch in range(4):
    bunch_select = longOut('DET:BUNCH%d' % bunch, 0, SAMPLES_PER_TURN/4-1,
        DESC = 'Detector bunch select #%d' % bunch)
    bunch_select.FLNK = records.calc('DET:BUNCH%d' % bunch,
        CALC = '4*A+B',  INPA = bunch_select, INPB = bunch,
        FLNK = update_bunch,
        DESC = 'Selected bunch #%d' % bunch)


# This will be reprocessed when necessary, basically when the sequencer settings
# change and a detector trigger occurs.
Waveform('DET:SCALE', TUNE_LENGTH, 'FLOAT',
    SCAN = 'I/O Intr', DESC = 'Scale for frequency sweep')

longOut('DET:SKEW',
    EGU = 'turns', DESC = 'Detector skew group delay in turns')



# We have five IQ waveforms: one for each bunch and an aggregate consisting of
# the sum of all four.
def IQwf(name, desc):
    return [
        Waveform('DET:I:%s' % name, TUNE_LENGTH, 'SHORT',
            DESC = '%s I' % desc),
        Waveform('DET:Q:%s' % name, TUNE_LENGTH, 'SHORT',
            DESC = '%s Q' % desc)]

Trigger('DET',
    *concat([IQwf(b, 'Bunch %s' % b) for b in '0123']) +
    IQwf('M', 'Bunch mean'))


# Also put the fixed NCO control here
aOut('NCO:FREQ', -936, 936, 'tune', 5, DESC = 'Fixed NCO frequency')
mbbOut('NCO:GAIN', DESC = 'Fixed NCO gain', *['Off'] + dBrange(15, -3))
