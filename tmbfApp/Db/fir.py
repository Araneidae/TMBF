from common import *


FIR_LENGTH = Parameter('FIR_LENGTH', 'Max taps in FIR')

# FIR

# There are four banks of FIR coefficients, each can either be written directly
# as a waveform or by separately controlling phase and fractional frequency.
for bank in range(4):
    boolOut('FIR:%d:USEWF' % bank, 'Settings', 'Waveform',
        DESC = 'Use direct waveform or settings')

    # Direct settings of FIR parameters
    ForwardLink('FIR:%d:RELOAD' % bank, 'Reload filter',
        longOut('FIR:%d:LENGTH' % bank, 2, FIR_LENGTH,
            DESC = 'Length of filter'),
        longOut('FIR:%d:CYCLES' % bank, 1, FIR_LENGTH,
            DESC = 'Cycles in filter'),
        aOut('FIR:%d:PHASE' % bank, -360, 360, DESC = 'FIR phase'))

    # Waveform taps in two forms: TAPS_S is what is set directly as a waveform
    # write, TAPS is what is current loaded.
    fir_name = 'FIR:%d:TAPS' % bank
    WaveformOut(fir_name, FIR_LENGTH, 'FLOAT', address = fir_name + '_S',
        DESC = 'Set waveform taps')
    Waveform(fir_name, FIR_LENGTH, 'FLOAT',
        SCAN = 'I/O Intr', DESC = 'Current waveform taps')


mbbOut('FIR:GAIN',
    DESC = 'FIR gain select', *dBrange(8, -6))
records.longin('FIR:N_TAPS', VAL = FIR_LENGTH, PINI = 'YES',
    DESC = 'FIR filter length')

longOut('FIR:DECIMATION', 0, 127,
    DESC = 'Bunch by bunch decimation')
