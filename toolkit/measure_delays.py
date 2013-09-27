#!/usr/bin/env dls-python

from pkg_resources import require
require('cothread')

import sys
import time
import numpy

import cothread
from cothread.catools import *


# Common support code.

BUNCH_COUNT = 936

class DAC_OUT:
    # Any combination of the values below is valid for a DAC output selection
    OFF = 0
    FIR = 1
    NCO = 2
    SWEEP = 4

    # Maximum DAC output gain
    MAX_GAIN = 1023


class PV:
    def __init__(self, name):
        self.monitor = camonitor(name, self.__on_update, format = FORMAT_TIME)
        self.event = cothread.Event()
        self.value = None

    def close(self):
        self.monitor.close()

    def __on_update(self, value):
        self.value = value
        self.event.Signal(value)

    # Waits for a reasonably fresh value to arrive.  No guarantees, but it will
    # be fresher than the last value!
    def get(self):
        return self.event.Wait()

    # Waits for a value at least as old as the given age to arrive.
    def get_new(self, age = 0, now = None):
        if now is None:
            now = time.time()
        now = now + age
        while True:
            value = self.get()
            if value.timestamp >= now:
                return value


class TMBF:
    def __init__(self, name):
        self.tmbf = name

        self.n_taps = self.PV('FIR:0:TAPS_S.NORD').get()

    def pv(self, name):
        return '%s:%s' % (self.tmbf, name)

    def PV(self, name):
        return PV(self.pv(name))

    def set(self, name, value):
        caput(self.pv(name), value)


# Ensures sequencer is disabled and ensures given bank is selected
def disable_sequencer(tmbf, bank):
    tmbf.set('TRG:SEQ:ENA_S', 'Disabled')
    tmbf.set('SEQ:RESET_S.PROC', 0)
    tmbf.set('SEQ:0:BANK_S', bank)


# Configure trigger targets for DDR, BUF and sequencer accordingly.
def configure_trigger_targets(tmbf, ddr, buf, seq):
    tmbf.set('TRG:DDR:SEL_S', ddr)
    tmbf.set('TRG:BUF:SEL_S', buf)
    tmbf.set('TRG:SEQ:ENA_S', seq)


# Configure selected bank for waveform control with given gain, fir and output
# waveform or constant values.
def configure_bank_wf(tmbf, bank, gain, fir, output):
    tmbf.set('BUN:%d:GAINWF_S' % bank, bunches(gain))
    tmbf.set('BUN:%d:FIRWF_S' % bank, bunches(fir))
    tmbf.set('BUN:%d:OUTWF_S' % bank, bunches(output))
    tmbf.set('BUN:%d:USEWF_S' % bank, 'Waveform')

def configure_bank(tmbf, bank, gain, fir, output):
    tmbf.set('BUN:%d:GAIN_S' % bank, gain)
    tmbf.set('BUN:%d:FIR_S' % bank, fir)
    tmbf.set('BUN:%d:OUT_S' % bank, output)
    tmbf.set('BUN:%d:USEWF_S' % bank, 'Settings')

def configure_state(tmbf, state,
        start, step, dwell, bank, gain, count=4096, window=True):
    tmbf.set('SEQ:%d:START_FREQ_S' % state, start)
    tmbf.set('SEQ:%d:STEP_FREQ_S' % state, step)
    tmbf.set('SEQ:%d:DWELL_S' % state, dwell)
    tmbf.set('SEQ:%d:COUNT_S' % state, count)
    tmbf.set('SEQ:%d:GAIN_S' % state, gain)
    tmbf.set('SEQ:%d:BANK_S' % state, bank)
    tmbf.set('SEQ:%d:ENWIN_S' % state, window)

def configure_nco(tmbf, frequency, gain):
    tmbf.set('NCO:FREQ_S', frequency)
    tmbf.set('NCO:GAIN_S', gain)

def configure_fir_wf(tmbf, fir, taps):
    tmbf.set('FIR:%d:TAPS_S' % fir, taps)
    tmbf.set('FIR:%d:USEWF_S' % fir, 'Waveform')


# Configure for tune sweep.
def configure_sweep(tmbf,
        start, step, dwell, bank, gain,
        count=4096, window=True):
    pass


# ------------------------------------------------------------------------------
# Data shaping and analysis

# Converts a single value into an array, otherwise validates length of array.
def bunches(value, length = BUNCH_COUNT):
    value = numpy.array(value)
    if value.size == 1:
        value = numpy.repeat(value, length)
    assert value.size == length, 'Invalid array length'
    return value

# Returns a waveform with one bunch set to value, all other bunches set to
# other.  Used for single bunch control.
def one_bunch(value, other, bunch = 0, length = BUNCH_COUNT, dtype = int):
    result = numpy.empty(length, dtype = dtype)
    result[:] = other
    result[bunch] = value
    return result

# Searches for one non-zero value in waveform
def find_one_peak(value):
    hits = numpy.nonzero(value)[0]
    assert len(hits) == 1, 'Unexpected data: %s' % value
    return hits[0]

# Searches for second highest value in waveform
def find_second_peak(value):
    return numpy.argsort(value)[-2]


# ------------------------------------------------------------------------------
# Timing test setup


# Configures basic set up for timing test: we use loopback with internal delay
# compensation turned off and a standard DAC preemphasis.
def configure_timing_test(tmbf, compensate = False):
    # Disable delay compensation so we meaure raw delays
    tmbf.set('COMPENSATE_S', 'Normal' if compensate else 'Disabled')
    # This is our nominal zero delay for the DAC pre-emphasis
    tmbf.set('DAC:PRECOMP_S', [0, 0, 1<<14])

    # For inputs use internal loopback
    tmbf.set('LOOPBACK_S', 'Loopback')
    # Turn off DAC output delay (until we need it...)
    tmbf.set('DAC:DELAY_S', 0)
    # Disable ADC input skew
    tmbf.set('ADC:DELAY_S', '0 ns')

    # For a sensible starting state, disable the sequencer and select bank 0
    disable_sequencer(tmbf, 0)

    # Reset detector window in case it's been messed with
    tmbf.set('DET:RESET_WIN_S.PROC', 0)

    # Configure NCO for DC output at maximum gain: this makes a useful pulse
    # source when routed into a single bunch.
    configure_nco(tmbf, 0, '0dB')


# Configure bank for single pulse output from NCO
def configure_dac_single_pulse(tmbf, bank = 0):
    configure_bank_wf(tmbf, bank,
        DAC_OUT.MAX_GAIN, 0, one_bunch(DAC_OUT.NCO, DAC_OUT.OFF))


def configure_buf_soft_trigger(tmbf):
    tmbf.set('TRG:S1:MODE_S', 'One Shot')
    tmbf.set('TRG:BUF:SEL_S', 'Soft 1')

def configure_ddr_soft_trigger(tmbf):
    tmbf.set('TRG:S1:MODE_S', 'One Shot')
    tmbf.set('TRG:DDR:SEL_S', 'Soft 1')

def fire_soft_trigger_1(tmbf):
    tmbf.set('TRG:S1:FIRE_S.PROC', 0)

def configure_detector(tmbf, bank=1, fir=0):
    # Configure the sequencer for a simple scan over a narrow range.
    configure_state(tmbf, 1, 1.3, 1e-5, 20, bank, '0dB')
    tmbf.set('SEQ:WRITE_S.PROC', 0)
    tmbf.set('SEQ:PC_S', 1)

    # Configure the selected bank for sweep
    configure_bank_wf(tmbf, bank, DAC_OUT.MAX_GAIN, fir, DAC_OUT.SWEEP)

    # Configure detector for single bunch capture
    tmbf.set('DET:MODE_S', 'Single Bunch')
    tmbf.set('DET:GAIN_S', '0dB')
    tmbf.set('DET:BUNCH0_S', 0)

    # Configure triggering and data capture
    tmbf.set('TRG:SEQ:ENA_S', 'Enabled')
    tmbf.set('TRG:BUF:SEL_S', 'Soft 1')
    tmbf.set('BUF:SELECT_S', 'IQ')


# Simplest test of all: measure delay from DAC reference bunch to minmax
# waveform.
def dac_to_minmax(tmbf, maxval):
    # Bunch zero will be our reference pulse
    configure_dac_single_pulse(tmbf)

    # Now fetch the next waveform with this data
    return find_one_peak(maxval.get_new(0.25))


# Measures closed loop delay.  For this configuration we need to pass data
# through the FIR for all bunches except the reference bunch.
def dac_to_dac_closed_loop(tmbf, maxdac):
    # Configure FIR 0 for passthrough
    configure_fir_wf(tmbf, 0, one_bunch(32767, 0, tmbf.n_taps-1, tmbf.n_taps))

    # Need quite strong attenuation to ensure the peak dies down before it comes
    # around again.
    tmbf.set('FIR:GAIN_S', '-24dB')

    # Configure bank 0 for fir 0 on all bunches, NCO output on bunch 0, FIR
    # passthrough on all other bunches.
    configure_bank_wf(tmbf, 0,
        DAC_OUT.MAX_GAIN, 0, one_bunch(DAC_OUT.NCO, DAC_OUT.FIR))

    # Fetch and process next valid waveform
    return find_second_peak(maxdac.get_new(0.25))


# Measures delay from reference bunch to selected buffer waveforms
def dac_to_buf(tmbf, bufa, bufb, sources):
    tmbf.set('BUF:SELECT_S', sources)
    fire_soft_trigger_1(tmbf)
    a = bufa.get()[:BUNCH_COUNT]
    b = bufb.get()[:BUNCH_COUNT]
    return find_one_peak(a), find_one_peak(b)

# Measures delay from reference bunch to selected DDR waveform
def dac_to_ddr(tmbf, ddr_buf, source):
    tmbf.set('DDR:INPUT_S', source)
    fire_soft_trigger_1(tmbf)
    return find_one_peak(ddr_buf.get()[:BUNCH_COUNT])


# Measures which bunch has its gain controlled by bunch select 0
def gain_delay(tmbf, maxdac):
    configure_bank_wf(tmbf, 0,
        one_bunch(DAC_OUT.MAX_GAIN, 0), 0, DAC_OUT.NCO)
    return find_one_peak(maxdac.get_new(0.25))


# Measures which bunch has its FIR controlled by bunch select 0
def fir_to_ddr(tmbf, ddr_buf):
    configure_fir_wf(tmbf, 1, bunches(0, tmbf.n_taps))
    configure_bank_wf(tmbf, 0,
        DAC_OUT.MAX_GAIN, one_bunch(0, 1), DAC_OUT.NCO)

    tmbf.set('DDR:INPUT_S', 'FIR')
    fire_soft_trigger_1(tmbf)
    return find_one_peak(ddr_buf.get()[:BUNCH_COUNT])


# Classical binary search.  test(a,b) should return true iff the target is in
# the half open range [a,b), and the test range is [start,end).
def binary_search(start, end, test):
    while end - start > 1:
        middle = (start + end) / 2
        if test(start, middle):
            end = middle
        else:
            start = middle
    return start


# Performs binary search to discover which bunch is detected for bunch zero
def search_det_bunch(tmbf, det_power, source):
    # Configure source
    tmbf.set('DET:INPUT_S', source)

    # Binary search.
    outwf = numpy.zeros(BUNCH_COUNT)

    def test(start, end):
        outwf[:start] = DAC_OUT.OFF
        outwf[start:end] = DAC_OUT.SWEEP
        outwf[end:] = DAC_OUT.OFF
        tmbf.set('BUN:1:OUTWF_S', outwf)

        fire_soft_trigger_1(tmbf)
        return det_power.get().mean() > 0

    return binary_search(0, BUNCH_COUNT, test)


# Computes group delay from scaled IQ.
def compute_group_delay(scale, iq):
    angle = numpy.unwrap(numpy.angle(iq))
    fit = numpy.polyfit(scale, angle)
    slope = fit[0]
    mean_error = numpy.std(angle - numpy.polyval(fit, scale))

    assert mean_error < 0.001, 'Unexpected error in slope fit'

    print 'slope', slope, 'error', mean_error

    # Convert slope into revolutions per bunch to get group delay (frequency
    # scale is in units of tune).
    return int(round(slope/2/numpy.pi * BUNCH_COUNT))


def search_det_delay(tmbf, det_i, det_q, source):
    # Configure source
    tmbf.set('DET:INPUT_S', source)

    # Capture data
    fire_soft_trigger_1(tmbf)
    iq = numpy.dot([1, 1j], [det_i.get(), det_q.get()])
    return compute_group_delay(scale, iq)


# ------------------------------------------------------------------------------
# Timing test

# We'll gather final configuration parameters for printout at the end.
class Results:
    def __init__(self):
        self.__dict__['items'] = []

    def __setattr__(self, name, value):
        # Ensure value is positive
        if value < 0:
            value += BUNCH_COUNT
        assert 0 <= value < BUNCH_COUNT, \
            'Value %s = %d out of range' % (name, value)
        assert value % 4 == 0, \
            'Value %s = %d out of phase' % (name, value)
        self.items.append(name)
        self.__dict__[name] = value / 4

    def print_results(self):
        for item in self.items:
            print item, '=', getattr(self, item)


# Performs initial calibration: measures DAC to ADC and DAC maxbuf and closes
# the loop for the remaining measurements.
def measure_maxbuf(tmbf, results):
    print >>sys.stderr, 'Measuring DAC/ADC maxbuf'

    maxdac = tmbf.PV('DAC:MAXBUF')
    maxadc = tmbf.PV('ADC:MAXBUF')

    # Start by taking DAC output mux select bunch zero as the reference bunch
    # and measuring delays to DAC max.  This allows us to measure the closed
    # loop delay and close the loop for all remaining measurements.
    dac_minmax_delay = dac_to_minmax(tmbf, maxdac)
    loop_delay = dac_to_dac_closed_loop(tmbf, maxdac)
    results.MINMAX_DAC_DELAY = dac_minmax_delay

    # Check that the gain control already acts on bunch zero: this is programmed
    # to be thus in the FPGA at present.
    results.BUNCH_GAIN_OFFSET = dac_minmax_delay - gain_delay(tmbf, maxdac)

    # Close the loop
    tmbf.set('DAC:DELAY_S', BUNCH_COUNT - loop_delay + dac_minmax_delay)

    # Now we can capture the max ADC offset
    results.MINMAX_ADC_DELAY = dac_to_minmax(tmbf, maxadc)

    maxdac.close()
    maxadc.close()


# Measures delays for BUF
def measure_buf(tmbf, results):
    print >>sys.stderr, 'Measuring BUF'

    configure_buf_soft_trigger(tmbf)
    buf_a = tmbf.PV('BUF:WFA')
    buf_b = tmbf.PV('BUF:WFB')
    buf_a.get()
    buf_b.get()

    results.BUF_ADC_DELAY, results.BUF_DAC_DELAY = \
        dac_to_buf(tmbf, buf_a, buf_b, 'ADC+DAC')
    results.BUF_FIR_DELAY, _ = \
        dac_to_buf(tmbf, buf_a, buf_b, 'FIR+DAC')

    buf_a.close()
    buf_b.close()


# Similarly, measure triggered DDR data.  Also measure FIR bunch zero control
# while we have the DDR buffer open.
def measure_ddr(tmbf, results):
    print >>sys.stderr, 'Measuring DDR'

    configure_ddr_soft_trigger(tmbf)
    ddr_buf = tmbf.PV('DDR:SHORTWF')
    ddr_buf.get()

    results.DDR_ADC_DELAY = dac_to_ddr(tmbf, ddr_buf, 'ADC')
    dac_to_ddr_fir_delay = dac_to_ddr(tmbf, ddr_buf, 'FIR')
    results.DDR_FIR_DELAY = dac_to_ddr_fir_delay
    results.DDR_RAW_DAC_DELAY = dac_to_ddr(tmbf, ddr_buf, 'Raw DAC')
    results.DDR_DAC_DELAY = dac_to_ddr(tmbf, ddr_buf, 'DAC')

    fir_bunch_zero = fir_to_ddr(tmbf, ddr_buf)
    results.BUNCH_FIR_OFFSET = dac_to_ddr_fir_delay - fir_bunch_zero

    ddr_buf.close()


# Discover the detector bunch offset by binary search.
def measure_detector_bunch(tmbf, results):
    print >>sys.stderr, 'Measuring Detector Bunch Offsets'

    # Configure detector for sweep and soft trigger
    configure_detector(tmbf)

    det_power = tmbf.PV('DET:POWER:0')
    det_power.get()

    results.DET_ADC_OFFSET = search_det_bunch(tmbf, det_power, 'ADC')
    results.DET_FIR_OFFSET = search_det_bunch(tmbf, det_power, 'FIR')
    det_power.close()


def measure_detector_delay(tmbf, results):
    print >>sys.stderr, 'Measuring Detector Group Delays'

    det_i = tmbf.PV('DET:I:M')
    det_q = tmbf.PV('DET:Q:M')

    results.DET_ADC_DELAY = search_det_delay(tmbf, det_i, det_q, 'ADC')
    results.DET_FIR_DELAY = search_det_delay(tmbf, det_i, det_q, 'FIR')

    det_i.close()
    det_q.close()


# ------------------------------------------------------------------------------


tmbf = TMBF('TS-DI-TMBF-01')
results = Results()


configure_timing_test(tmbf)

measure_maxbuf(tmbf, results)
measure_buf(tmbf, results)
measure_ddr(tmbf, results)
measure_detector_bunch(tmbf, results)

results.print_results()
