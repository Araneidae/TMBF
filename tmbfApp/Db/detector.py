# DET

from common import *

import tune


mbbOut('DET:GAIN', DESC = 'Detector gain', *dBrange(7, -12) + ['Min'])
boolOut('DET:MODE', 'All Bunches', 'Single Bunch',
    FLNK = tune.setting_changed, DESC = 'Detector mode')
mbbOut('DET:INPUT', 'FIR', 'ADC', DESC = 'Detector input selection')
boolOut('DET:AUTOGAIN', 'Fixed Gain', 'Autogain',
    DESC = 'Detector automatic gain')

for bunch in range(4):
    bunch_select = longOut('DET:BUNCH%d' % bunch, 0, BUNCHES_PER_TURN/4-1,
        DESC = 'Detector bunch select #%d' % bunch)
    bunch_select.FLNK = records.calc('DET:BUNCH%d' % bunch,
        CALC = '4*A+B',  INPA = bunch_select, INPB = bunch,
        FLNK = tune.setting_changed,
        DESC = 'Selected bunch #%d' % bunch)


# This will be reprocessed when necessary, basically when the sequencer settings
# change and a detector trigger occurs.
Waveform('DET:SCALE', TUNE_LENGTH, 'DOUBLE',
    SCAN = 'I/O Intr', DESC = 'Scale for frequency sweep')
# A linear scale for convenience
Waveform('DET:LINEAR', TUNE_LENGTH, 'LONG', DESC = 'Linear scale')

# Two overflow detection bits are generated
overflows = [
    boolIn('DET:OVF:ACC', 'Ok', 'Overflow', OSV = 'MAJOR',
        DESC = 'Detector accumulator overflow'),
    boolIn('DET:OVF:IQ',  'Ok', 'Overflow', OSV = 'MAJOR',
        DESC = 'IQ scaling overflow')]
overflows.append(
    AggregateSeverity('DET:OVF', 'Detector overflow', overflows))

# We have five sweep channels: one for each bunch and an aggregate consisting of
# the sum of all four.
def SweepChannel(name, desc):
    def Name(field):
        return 'DET:%s:%s' % (field, name)
    return [
        # Basic I/Q waveforms and power spectrum
        Waveform(Name('I'), TUNE_LENGTH, 'SHORT', DESC = '%s I' % desc),
        Waveform(Name('Q'), TUNE_LENGTH, 'SHORT', DESC = '%s Q' % desc),
        Waveform(Name('POWER'), TUNE_LENGTH, 'LONG', DESC = '%s power' % desc),
    ]

bunch_channels = [SweepChannel(b, 'Bunch %s' % b) for b in '0123']
mean_channel = SweepChannel('M', 'Bunch mean')
Trigger('DET', *concat(bunch_channels) + mean_channel + overflows)


# Control over the internal detector window.
det_window = WaveformOut('DET:WINDOW', 1024, 'SHORT', DESC = 'Detector window')
boolOut('DET:RESET_WIN', FLNK = det_window, PINI = 'YES',
    DESC = 'Reset detector window to Hamming')

# Total loop delay in turns.
aOut('DET:LOOP:ADC', 1,
    EGU = 'turns', PREC = 1, DESC = 'Closed loop delay in turns')
aOut('DET:LOOP:FIR', 0,
    EGU = 'turns', PREC = 1, DESC = 'Extra FIR loop delay')


# Also put the fixed NCO control here
aOut('NCO:FREQ', -BUNCHES_PER_TURN, BUNCHES_PER_TURN, 'tune', 5,
    DESC = 'Fixed NCO frequency')
mbbOut('NCO:GAIN', DESC = 'Fixed NCO gain', *dBrange(8, -6))
