from common import *

# Bunch selection

# For each bunch setting there is an associated status string which is updated
# as the waveform is updated.
def BunchWaveforms(bank, name, FTVL, desc):
    name = 'BUN:%d:%s' % (bank, name)
    status = stringIn('%s:STA' % name,
        DESC = 'Bank %d %s status' % (bank, name))
    WaveformOut(name + '_S', BUNCHES_PER_TURN, FTVL,
        FLNK = status, DESC = 'Set %d %s' % (bank, desc))


# We have four banks and for each bank three waveforms of parameters to
# configure.   Very similar to FIR.
for bank in range(4):
    # Waveform settings with status update
    BunchWaveforms(bank, 'FIRWF', 'CHAR', 'FIR bank select')
    BunchWaveforms(bank, 'OUTWF', 'CHAR', 'DAC output select')
    BunchWaveforms(bank, 'GAINWF', 'LONG', 'DAC output gain')


# Bunch synchronisation.
Action('BUN:SYNC', DESC = 'Bunch synchronisation enable')
longOut('BUN:OFFSET', 0, 233, DESC = 'Zero bunch offset')
longIn('BUN:PHASE', SCAN = 'I/O Intr', LOW = 0, LSV = 'MINOR', MDEL = 0,
    DESC = 'Bunch phase detect')
