from common import *


FIR_LENGTH = Parameter('FIR_LENGTH', 'Max taps in FIR')

# FIR

# There are four banks of FIR coefficients, each can either be written directly
# as a waveform or by separately controlling phase and fractional frequency.
for bank in range(4):
    boolOut('FIR:%d:USEWF' % bank, 'Settings', 'Waveform',
        DESC = 'Use direct waveform or settings')

    # Direct settings of FIR parameters
    reload = Action('FIR:%d:RELOAD' % bank, DESC = 'Reload filter')
    longOut('FIR:%d:LENGTH' % bank, 2, FIR_LENGTH,
        FLNK = reload, DESC = 'Length of filter')
    longOut('FIR:%d:CYCLES' % bank, 1, FIR_LENGTH,
        FLNK = reload, DESC = 'Cycles in filter')
    aOut('FIR:%d:PHASE' % bank, -360, 360,
        FLNK = reload, DESC = 'FIR phase')

    # Waveform taps in two forms: TAPS_S is what is set directly as a waveform
    # write, TAPS is what is current loaded.
    WaveformOut('FIR:%d:TAPS_S' % bank, FIR_LENGTH,
        DESC = 'Set waveform taps')
    Waveform('FIR:%d:TAPS' % bank, FIR_LENGTH,
        SCAN = 'I/O Intr', DESC = 'Current waveform taps')


mbbOut('FIR:GAIN',
    DESC = 'FIR gain select', *dBrange(16, -3, 21))