# Tune peak detect

from common import *

MAX_PEAKS = 3

PEAK_FIT_SIZE = 168


def peak_readbacks(suffix):
    return [
        Waveform('PEAK:POWER:%d' % suffix, TUNE_LENGTH/suffix, 'LONG',
            DESC = 'Smoothed tune'),
        Waveform('PEAK:IX:%d' % suffix, MAX_PEAKS, 'LONG',
            DESC = 'Peak indexes'),
        Waveform('PEAK:V:%d' % suffix, MAX_PEAKS, 'LONG',
            DESC = 'Peak values'),
        Waveform('PEAK:L:%d' % suffix, MAX_PEAKS, 'LONG',
            DESC = 'Peak left'),
        Waveform('PEAK:R:%d' % suffix, MAX_PEAKS, 'LONG',
            DESC = 'Peak right'),
        Waveform('PEAK:PDD:%d' % suffix, TUNE_LENGTH/suffix, 'LONG',
            DESC = 'Second derivative of power'),
    ]

def peak_results(prefix):
    name = prefix.capitalize()
    return [
        aIn('PEAK:%s' % prefix, 0, 1,
            PREC = 5, DESC = '%s peak frequency' % name),
        aIn('PEAK:%s:PHASE' % prefix, -180, 180, 'deg',
            PREC = 1, DESC = '%s peak phase' % name),
        aIn('PEAK:%s:AREA' % prefix,
            PREC = 3, DESC = '%s peak area' % name),
        aIn('PEAK:%s:WIDTH' % prefix, 0, 1,
            PREC = 5, DESC = '%s peak width' % name),
        aIn('PEAK:%s:HEIGHT' % prefix, DESC = '%s peak height' % name),
        boolIn('PEAK:%s:VALID' % prefix, 'Invalid', 'Ok',
            ZSV = 'MINOR', DESC = '%s peak valid' % name),
    ]

def peak_rel_results(prefix):
    name = prefix.capitalize()
    return [
        aIn('PEAK:%s:DTUNE' % prefix, 0, 1,
            PREC = 5, DESC = '%s delta tune' % name),
        aIn('PEAK:%s:DPHASE' % prefix, -180, 180, 'deg',
            PREC = 1, DESC = '%s delta phase' % name),
        aIn('PEAK:%s:RAREA' % prefix,
            PREC = 3, DESC = '%s relative area' % name),
        aIn('PEAK:%s:RWIDTH' % prefix,
            PREC = 3, DESC = '%s relative width' % name),
        aIn('PEAK:%s:RHEIGHT' % prefix,
            PREC = 3, DESC = '%s relative height' % name),
    ]


Trigger('PEAK',
    # Waveforms for reading tune peak detection results
    Waveform('PEAK:FIRSTFIT',
        PEAK_FIT_SIZE, 'CHAR', DESC = 'Raw first fit data'),
    Waveform('PEAK:SECONDFIT',
        PEAK_FIT_SIZE, 'CHAR', DESC = 'Raw second fit data'),
    longIn('PEAK:COUNT', DESC = 'Final fitted peak count'),

    # Synchrotron tune estimated from sidebands
    aIn('PEAK:SYNCTUNE', 0, 1, PREC = 5, DESC = 'Synchrotron tune'),

*
    # Peak detection support
    peak_readbacks(16) + peak_readbacks(64) +
    peak_results('LEFT') + peak_results('CENTRE') + peak_results('RIGHT') +
    peak_rel_results('LEFT') + peak_rel_results('RIGHT')
)

# Controls for new peak finding algorithm
aOut('PEAK:THRESHOLD', 0, 1, PREC = 2,
    DESC = 'Fit data selection threshold')
aOut('PEAK:MINWIDTH', 0, 1, PREC = 2, DESC = 'Minimum valid peak width')
aOut('PEAK:MAXWIDTH', 0, 1, PREC = 2, DESC = 'Maximum valid peak width')
aOut('PEAK:FITERROR', 0, 10, PREC = 3, DESC = 'Maximum fit error')

mbbOut('PEAK:SEL', '/16', '/64', DESC = 'Select smoothing')
