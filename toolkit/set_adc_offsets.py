#!/usr/bin/env dls-python

from pkg_resources import require
require('numpy')
require('cothread')

import sys
import numpy

from tmbf import TMBF


tmbf = TMBF(sys.argv[1])


# For ADC offsets should be enough to turn DAC output off and grab a single
# ADC waveform.  Note that we need to set the ADC skew to 0ns first (or else we
# need to apply intelligence to our measurements), as the offsets are applied to
# the raw input.
tmbf.set_save('DAC:ENABLE_S', 'Off')
tmbf.set_save('ADC:DELAY_S', '0 ns')
current_offset = tmbf.get('ADC:OFFSET_S')

# Configure DDR capture for single shot soft capture.
tmbf.set('DDR:INPUT_S', 'ADC')
tmbf.set('TRG:DDR:SEL_S', 'Soft 1')
tmbf.set('TRG:S1:MODE_S', 'One Shot')

# tmbf.set('TRG:S1:FIRE_S.PROC', 0)
tmbf.s1.arm()
adc = tmbf.get('DDR:SHORTWF')
offsets = -numpy.round(adc.reshape(-1, 4).mean(0))

new_offset = current_offset + offsets
print 'Setting offsets by', offsets, 'to', new_offset
tmbf.set('ADC:OFFSET_S', new_offset)

tmbf.restore_saved()
