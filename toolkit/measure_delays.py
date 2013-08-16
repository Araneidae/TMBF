#!/usr/bin/env dls-python

from pkg_resources import require
require('cothread')

import sys
import time
import numpy

import cothread
from cothread.catools import *


class PV:
    def __init__(self, name, default_delay = 0, debug = False):
        self.name = name
        self.default_delay = default_delay
        self.debug = debug
        self.event = cothread.Event()
        camonitor(name, self.on_update, format = FORMAT_TIME)

    def on_update(self, value):
        if self.debug:
            print 'on_update', self.name
        self.value = value
        self.timestamp = value.timestamp
        self.event.Signal(value)

    def get_next(self, min_age = None):
        if min_age is None:
            min_age = self.default_delay
        now = time.time()
        while True:
            value = self.event.Wait()
            if min_age < 0 or value.timestamp - now >= min_age:
                break
        return value

    def reset(self):
        self.event.Reset()


def pv(name):
    return '%s:%s' % (tmbf, name)


def one_bunch(bunch_value, other_values, bunch = 0, dtype = int):
    result = all_bunches(other_values, dtype)
    result[bunch] = bunch_value
    return result

def all_bunches(bunch_value, dtype = int):
    result = numpy.empty(936, dtype = dtype)
    result[:] = bunch_value
    return result

def get_buffer(source):
    caput(pv('DDR:INPUT_S'), source)
    return ddr_buf.get_next()

def get_one_peak(source, min_age = None):
    data = source.get_next(min_age)[:936]
    hits = numpy.nonzero(data)[0]
    assert len(hits) == 1, 'Hits = %s' % hits
    offset = hits[0]
    assert offset % 4 == 0, 'Offset = %d' % offset
    return offset

def get_second_peak(source):
    data = source.get_next()
    tops = numpy.argsort(data)[-2:]
    print tops, data[tops]
    return tops[0]

def get_det_power():
    return det_power.get_next().mean()


tmbf = 'TS-DI-TMBF-01'

max_adc  = PV(pv('ADC:MAXBUF'), 0.5)
max_dac  = PV(pv('DAC:MAXBUF'), 0.5)
buffer_a = PV(pv('BUF:WFA'), 0.5)
buffer_b = PV(pv('BUF:WFB'), 0.5)
ddr_buf  = PV(pv('DDR:SHORTWF'), 0.5)

det_power = PV(pv('DET:POWER:0'), 1)



# Put system into base configuration
caput(pv('LOOPBACK_S'), 1)                  # Loopback for predicable results
caput(pv('COMPENSATE_S'), 1)                # Disable delay compensation

caput(pv('DAC:PRECOMP_S'), [0, 0, 16384])   # Minimum precompensator delay
caput(pv('DAC:DELAY_S'), 0)                 #  and minimum DAC
caput(pv('ADC:DELAY_S'), 0)                 #  and ADC delays

caput(pv('DAC:SCAN_S.SCAN'), '.1 second')   # Ensure minmax data is lively
caput(pv('ADC:SCAN_S.SCAN'), '.1 second')


# Start with NCO as a DC source.
caput(pv('NCO:FREQ_S'), 0)
caput(pv('NCO:GAIN_S'), '0dB')

# Disable sequencer and use bank 0, configure DDR and buffer for external
# trigger (maybe need to use internal trigger...)
caput(pv('TRG:SEQ:ENA_S'), 'Disabled')
caput(pv('TRG:DDR:SEL_S'), 'External')
caput(pv('TRG:BUF:SEL_S'), 'External')
caput(pv('TRG:EXT:MODE_S'), 'Retrigger')
caput(pv('TRG:EXT:ARM_S.PROC'), 0)

caput(pv('SEQ:0:BANK_S'), 0)


fir = caget(pv('FIR:0:TAPS_S'))

# Configure zero FIR in FIR 0
fir[:] = 0
caput(pv('FIR:0:TAPS_S'), fir)
caput(pv('FIR:0:USEWF_S'), 'Waveform')

# Configure impulse FIR in FIR 1
fir[-1] = 32767
caput(pv('FIR:1:TAPS_S'), fir)
caput(pv('FIR:1:USEWF_S'), 'Waveform')

caput(pv('FIR:GAIN_S'), '-24dB')


# Let's first figure out where bunch DAC select 0 goes.  Both OUT and GAIN
# should produce the same result, but we'll test one at a time.
caput(pv('BUN:0:GAINWF_S'), all_bunches(1023))  # Max gain all bunches
caput(pv('BUN:0:FIRWF_S'), all_bunches(1))      # Impulse FIR all bunches
caput(pv('BUN:0:OUTWF_S'), one_bunch(2, 0))     # NCO in bunch 0, all else off
caput(pv('BUN:0:USEWF_S'), 1)


# Read the minmax buffers
max_dac.get_next()
dac_minmax_offset = get_one_peak(max_dac)
adc_minmax_offset = get_one_peak(max_adc)
print 'DAC bunch select to DAC minmax =', dac_minmax_offset
print 'DAC bunch select to ADC minmax =', adc_minmax_offset

# Read the DDR buffers
caput(pv('DDR:INPUT_S'), 'ADC')
ddr_adc_offset = get_one_peak(ddr_buf)
caput(pv('DDR:INPUT_S'), 'FIR')
ddr_fir_offset = get_one_peak(ddr_buf)
caput(pv('DDR:INPUT_S'), 'Raw DAC')
ddr_rawdac_offset = get_one_peak(ddr_buf)
caput(pv('DDR:INPUT_S'), 'DAC')
ddr_dac_offset = get_one_peak(ddr_buf)
print 'DAC bunch select to DDR ADC =', ddr_adc_offset
print 'DAC bunch select to DDR FIR =', ddr_fir_offset
print 'DAC bunch select to DDR Raw DAC =', ddr_rawdac_offset
print 'DAC bunch select to DDR DAC =', ddr_dac_offset

# Read the fast internal buffer
caput(pv('BUF:SELECT_S'), 'FIR+ADC')
buf_fir_offset = get_one_peak(buffer_a)
buf_adc_offset = get_one_peak(buffer_b)
caput(pv('BUF:SELECT_S'), 'ADC+DAC')
buf_dac_offset = get_one_peak(buffer_b)
print 'DAC bunch select to buffer FIR =', buf_fir_offset
print 'DAC bunch select to buffer ADC =', buf_adc_offset
print 'DAC bunch select to buffer DAC =', buf_dac_offset


# Now let's do a quick sanity check with gain control instead of output control.
# Just need to check the DAC output for this.
caput(pv('BUN:0:GAINWF_S'), one_bunch(1023, 0))
caput(pv('BUN:0:OUTWF_S'), all_bunches(2))

assert get_one_peak(max_dac) == dac_minmax_offset


# Next figure out which bunch the FIR control is operating on.  Output all
# bunches but look at position of selected FIR.
caput(pv('BUN:0:GAINWF_S'), all_bunches(1023))
caput(pv('BUN:0:FIRWF_S'), one_bunch(1, 0))

caput(pv('BUF:SELECT_S'), 'FIR+ADC')
fir_offset = get_one_peak(buffer_a)
print 'FIR offset =', fir_offset


# Now let's discover the closed loop delay.  Configure DAC for FIR everywhere
# except bunch zero which is NCO.
caput(pv('BUN:0:GAINWF_S'), all_bunches(1023))
caput(pv('BUN:0:FIRWF_S'), all_bunches(1))
caput(pv('BUN:0:OUTWF_S'), one_bunch(2, 1))

closed_loop_delay_loopback = get_second_peak(max_dac)
print 'Closed loop delay (loopback) =', closed_loop_delay_loopback


# Now repeat DDR and buffer measurements with soft triggers
caput(pv('TRG:EXT:MODE_S'), 'One Shot')
caput(pv('TRG:SEQ:ENA_S'), 'Disabled')
caput(pv('TRG:DDR:SEL_S'), 'Soft 1')
caput(pv('TRG:BUF:SEL_S'), 'Soft 1')

caput(pv('BUN:0:GAINWF_S'), all_bunches(1023))  # Max gain all bunches
caput(pv('BUN:0:FIRWF_S'), all_bunches(1))      # Impulse FIR all bunches
caput(pv('BUN:0:OUTWF_S'), one_bunch(2, 0))     # NCO in bunch 0, all else off

caput(pv('BUF:SELECT_S'), 'ADC+DAC')
caput(pv('DDR:INPUT_S'), 'DAC')

cothread.Sleep(1)
buffer_a.reset()
ddr_buf.reset()

caput(pv('TRG:S1:FIRE_S.PROC'), 0)
soft_buf = get_one_peak(buffer_a, -1)
soft_ddr = get_one_peak(ddr_buf, -1)
print 'Soft peaks:', soft_buf, soft_ddr



# Put triggers back for detector sweeping.
caput(pv('TRG:SEQ:ENA_S'), 'Enabled')
caput(pv('TRG:DDR:SEL_S'), 'External')
caput(pv('TRG:BUF:SEL_S'), 'External')
caput(pv('TRG:EXT:MODE_S'), 'Retrigger')
caput(pv('TRG:EXT:ARM_S.PROC'), 0)



# Now we need to figure out where the detector bunch selects go.  This requires
# more work as we're going to have to do a binary search.

# Set bank 0 to be quiescent
caput(pv('BUN:0:OUT_S'), 'Off')
caput(pv('BUN:0:USEWF_S'), 'Settings')


# Set up bank 1 for sweep excitation.  Start with all bunches.
caput(pv('BUN:1:GAINWF_S'), all_bunches(1023))
caput(pv('BUN:1:FIRWF_S'), all_bunches(1))
caput(pv('BUN:1:OUTWF_S'), all_bunches(4))
caput(pv('BUN:1:USEWF_S'), 'Waveform')

# Configure sequencer etc as appropriate.  We'll do a fairly narrow sweep to
# avoid funny frequency effects.
caput(pv('SEQ:1:START_FREQ_S'), 1.3)
caput(pv('SEQ:1:STEP_FREQ_S'), 0.00001) # Try to avoid corners
caput(pv('SEQ:1:DWELL_S'), 100)
caput(pv('SEQ:1:COUNT_S'), 4096)
caput(pv('SEQ:1:GAIN_S'), '0dB')        # Anomalies to chase at higher gain
caput(pv('SEQ:1:BANK_S'), 1)
caput(pv('SEQ:1:ENWIN_S'), 1)
caput(pv('SEQ:WRITE_S.PROC'), 0)
caput(pv('SEQ:PC_S'), 1)

caput(pv('TRG:SEQ:ENA_S'), 'Enabled')

caput(pv('DET:RESET_WIN_S.PROC'), 0)    # Ensure we're using the standard window
caput(pv('BUF:SELECT_S'), 'IQ')
caput(pv('DET:BUNCH0_S'), 0)
caput(pv('DET:BUNCH1_S'), 0)
caput(pv('DET:BUNCH2_S'), 0)
caput(pv('DET:BUNCH3_S'), 0)
caput(pv('DET:MODE_S'), 'Single Bunch')
caput(pv('DET:GAIN_S'), '-36dB')

# Binary search.
outwf = numpy.zeros(936)
start, end = 0, 936
while end - start > 1:
    middle = (start + end) / 2
    outwf[:start] = 0
    outwf[start:middle] = 4
    outwf[middle:] = 0
    caput(pv('BUN:1:OUTWF_S'), outwf)
    power = get_det_power()

    print '>', start, middle, end, power
    if power > 0:
        # Hit, keep trying this half
        end = middle
    else:
        # Miss, try the other half
        start = middle
print 'Detector bunch offset =', start


# Finally let's measure closed loop delay with external connection
caput(pv('TRG:SEQ:ENA_S'), 'Disabled')
caput(pv('LOOPBACK_S'), 'Normal')

caput(pv('BUN:0:GAINWF_S'), all_bunches(1023))
caput(pv('BUN:0:FIRWF_S'), all_bunches(1))
caput(pv('BUN:0:OUTWF_S'), one_bunch(2, 1))
caput(pv('BUN:0:USEWF_S'), 'Waveform')
caput(pv('FIR:GAIN_S'), '-6dB')

closed_loop_delay_normal = get_second_peak(max_dac)
print 'Closed loop delay (normal) =', closed_loop_delay_normal


# Notes:
# - Refactor for useful code
# - Set DET:INPUT_S
# - Sweep for both detector bunch sources
