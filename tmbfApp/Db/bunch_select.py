from common import *

# Bunch selection

def BunchWaveforms(bank, name, FTVL, desc, FLNK):
    name = 'BUN:%d:%s' % (bank, name)
    WaveformOut(name + '_S', SAMPLES_PER_TURN, FTVL,
        FLNK = FLNK, DESC = 'Set %s' % desc)
    Waveform(name, SAMPLES_PER_TURN, FTVL,
        SCAN = 'I/O Intr', DESC = 'Current %s' % desc)


# We have four banks and for each bank three waveforms of parameters to
# configure.   Very similar to FIR.
for bank in range(4):
    boolOut('BUN:%d:USEWF' % bank, 'Settings', 'Waveform',
        DESC = 'Use direct waveform or settings')

    # Some canned settings
    reload = Action('BUN:%d:RELOAD' % bank, DESC = 'Reload bunch config')
    mbbOut('BUN:%d:FIR' % bank, 'FIR 0', 'FIR 1', 'FIR 2', 'FIR 3',
        FLNK = reload, DESC = 'FIR bank select')
    mbbOut('BUN:%d:OUT' % bank,
        'Off', 'FIR', 'NCO+FIR', 'NCO',
        'Sweep', 'NCO+Sweep', 'Sweep+FIR', 'NCO+Sweep+FIR',
        FLNK = reload, DESC = 'DAC output select')
    longOut('BUN:%d:GAIN' % bank, -(1<<10), (1<<10)-1,
        FLNK = reload, DESC = 'DAC output gain')

    # Waveform settings with readbacks
    reload_wf = Action('BUN:%d:RELOADWF' % bank,
        DESC = 'Reload bunch waveform config')
    BunchWaveforms(bank, 'FIRWF', 'CHAR', 'FIR bank select', FLNK = reload_wf)
    BunchWaveforms(bank, 'OUTWF', 'CHAR', 'DAC output select', FLNK = reload_wf)
    BunchWaveforms(bank, 'GAINWF', 'LONG', 'DAC output gain', FLNK = reload_wf)


Action('BUN:SYNC', DESC = 'Bunch synchronisation enable')
