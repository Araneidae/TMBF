# DET

from common import *



mbbOut('DET:GAIN', DESC = 'Detector gain', *dBrange(7, -12) + ['Min'])
boolOut('DET:MODE', 'All Bunches', 'Single Bunch', DESC = 'Detector mode')
mbbOut('DET:INPUT', 'FIR', 'ADC', DESC = 'Detector input selection')

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

# Two overflow detection bits are generated
overflows = [
    boolIn('DET:OVF:ACC', 'Ok', 'Overflow', OSV = 'MAJOR',
        DESC = 'Detector accumulator overflow'),
    boolIn('DET:OVF:IQ',  'Ok', 'Overflow', OSV = 'MAJOR',
        DESC = 'IQ scaling overflow')]

# We have five sweep channels: one for each bunch and an aggregate consisting of
# the sum of all four.
def SweepChannel(name, desc):
    def Name(field):
        return 'DET:%s:%s' % (field, name)
    return [
        # Basic I/Q waveforms
        Waveform(Name('I'), TUNE_LENGTH, 'SHORT', DESC = '%s I' % desc),
        Waveform(Name('Q'), TUNE_LENGTH, 'SHORT',
            DESC = '%s Q' % desc),

        # Power spectrum and computed tune
        Waveform(Name('POWER'), TUNE_LENGTH, 'LONG', DESC = '%s power' % desc),
        aIn(Name('PTUNE'), 0, 1, PREC = 4, DESC = '%s tune from power'),
        aIn(Name('PPHASE'), -180, 180, 'deg', PREC = 1,
            DESC = '%s tune phase from power'),

        # Cumulative sume and computed tune
        Waveform(Name('CUMSUMI'), TUNE_LENGTH, 'LONG',
            DESC = '%s cumsum I' % desc),
        Waveform(Name('CUMSUMQ'), TUNE_LENGTH, 'LONG',
            DESC = '%s cumsum Q' % desc),
        aIn(Name('CTUNE'), 0, 1, PREC = 5, DESC = '%s tune from cumsum'),
        aIn(Name('CPHASE'), -180, 180, 'deg', PREC = 1,
            DESC = '%s tune phase from cumsum'),
    ]

bunch_channels = [SweepChannel(b, 'Bunch %s' % b) for b in '0123']
mean_channel = SweepChannel('M', 'Bunch mean')
Trigger('DET', *concat(bunch_channels) + mean_channel + overflows)


# Control over the internal detector window.
det_window = WaveformOut('DET:WINDOW', 1024, 'SHORT', DESC = 'Detector window')
boolOut('DET:RESET_WIN', FLNK = det_window, PINI = 'YES',
    DESC = 'Reset detector window to Hamming')


# Also put the fixed NCO control here
aOut('NCO:FREQ', -936, 936, 'tune', 5, DESC = 'Fixed NCO frequency')
mbbOut('NCO:GAIN', DESC = 'Fixed NCO gain', *dBrange(8, -6))
