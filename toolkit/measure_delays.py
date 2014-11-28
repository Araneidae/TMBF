#!/usr/bin/env dls-python

from pkg_resources import require
require('cothread')

import sys
import time
import numpy

import cothread
from cothread.catools import *

import configure


# Common support code.


class DAC_OUT:
    # Any combination of the values below is valid for a DAC output selection
    OFF = 0
    FIR = 1
    NCO = 2
    SWEEP = 4



# ------------------------------------------------------------------------------

# ------------------------------------------------------------------------------
# Basic TMBF setup support



# Configures basic set up for timing test: we use loopback with internal delay
# compensation turned off and a standard DAC preemphasis.  This establishes a
# setup that can be assumed by all other tests.
def configure_timing_test(tmbf, compensate = False):
    # Turn DAC output in case there's something connected
    tmbf.set('DAC:ENABLE_S', 'Off')

    # Ensure nothing is running
    tmbf.set('TRG:DDR:RESET_S', 0)
    tmbf.set('TRG:BUF:RESET_S', 0)
    tmbf.set('SEQ:RESET_S', 0)

    # Disable delay compensation so we meaure raw delays and use loopback data
    tmbf.set('COMPENSATE_S', 'Normal' if compensate else 'Disabled')
    tmbf.set('LOOPBACK_S', 'Loopback')

    # Configure ADC and DAC filters with identity filters.
    tmbf.set('ADC:OFFSET_S', 4 * [0])
    tmbf.set('ADC:FILTER_S', 4 * [0, 1, 0])
    tmbf.set('ADC:FILTER:DELAY_S', '0 ns')
    tmbf.set('DAC:PREEMPH_S', [0, 1, 0])
    tmbf.set('DAC:PREEMPH:DELAY_S', '0 ns')

    # For a sensible starting state, disable the sequencer and select bank 0.
    # Each test will need to configure the bank setup it needs.
    configure.state0(tmbf, 0)
    configure.sequencer_disable(tmbf)
    # Configure FIR 0 to default pass-through mode
    configure.fir_wf(tmbf, 0, 1)

    # Configure NCO for DC output at maximum gain: this makes a useful pulse
    # source when routed into a single bunch.
    tmbf.set('NCO:FREQ_S', 0)
    tmbf.set('NCO:GAIN_S', '0dB')

    # We'll be using soft triggering.  Ensure it's in one shot state
    tmbf.set('TRG:SYNC_S', 'Separate')
    tmbf.set('TRG:DDR:MODE_S', 'One Shot')
    tmbf.set('TRG:BUF:MODE_S', 'One Shot')
    tmbf.set('TRG:DDR:SEL_S', 'Soft')
    tmbf.set('TRG:BUF:SEL_S', 'Soft')

    # Reset detector window in case it's been messed with
    tmbf.set('DET:RESET_WIN_S.PROC', 0)

    # When we use the sequencer it's always for a single state
    tmbf.set('SEQ:PC_S', 1)
    tmbf.set('SEQ:SUPER:COUNT_S', 1)



# Configure bank for single pulse output from NCO
def configure_dac_single_pulse(tmbf):
    configure.bank_wf(tmbf, 0, 1, 0, one_bunch(tmbf, DAC_OUT.NCO, DAC_OUT.OFF))


# Configure for single one-shot bunch:
def configure_one_shot_pulse(tmbf, trigger):
    configure.bank_wf(tmbf, 0, 1, 0, DAC_OUT.OFF)
    configure.bank_wf(tmbf, 1, 1, 0, one_bunch(tmbf, DAC_OUT.NCO, DAC_OUT.OFF))
    configure.state(tmbf, dwell = 1, count = 1)

    configure.sequencer_enable(tmbf, trigger)



def setup_detector_single_bunch(tmbf, bank=1, fir=0):
    # Configure the sequencer for a simple scan over a narrow range.
    configure.state(tmbf, 1, 1.31, 1e-5, 20, '0dB', bank = bank)

    # Configure the selected bank for sweep
    configure.bank_wf(tmbf, bank, 1, fir, DAC_OUT.SWEEP)

    # Configure detector for single bunch capture
    tmbf.set('DET:MODE_S', 'Single Bunch')
    tmbf.set('DET:GAIN_S', '0dB')
    tmbf.set('DET:AUTOGAIN_S', 'Fixed Gain')
    tmbf.set('DET:BUNCH0_S', 0)

    # Configure triggering and data capture
    tmbf.set('TRG:SEQ:SEL_S', 'BUF trigger')
    tmbf.set('TRG:BUF:SEL_S', 'Soft 1')
    tmbf.set('BUF:SELECT_S', 'IQ')


def setup_tune_sweep(tmbf, *args, **kargs):
    configure.state(tmbf, 1, *args, **kargs)
    tmbf.set('BUF:SELECT_S', 'IQ')
    configure.sequencer_enable(tmbf, 'BUF')


# ------------------------------------------------------------------------------
# Data shaping and analysis

# Returns a waveform with one bunch set to value, all other bunches set to
# other.  Used for single bunch control.
def one_bunch(tmbf, value, other, bunch = 0):
    result = numpy.empty(tmbf.bunches, dtype = int)
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


# Measures closed loop delay.  For this configuration we need to pass data
# through the FIR for all bunches except the reference bunch.
def dac_to_dac_closed_loop(tmbf, maxdac):
    # Need quite strong attenuation to ensure the peak dies down before it comes
    # around again.
    tmbf.set('FIR:GAIN_S', '-24dB')

    # Configure bank 0 for fir 0 on all bunches, NCO output on bunch 0, FIR
    # passthrough on all other bunches.
    configure.bank_wf(tmbf, 0, 1, 0, one_bunch(tmbf, DAC_OUT.NCO, DAC_OUT.FIR))

    # Fetch and process next valid waveform
    return find_second_peak(maxdac.get_new(0.25))


# Measures delay from reference bunch to selected buffer waveforms
def dac_to_buf(tmbf, bufa, bufb, sources, length = -1):
    tmbf.set('BUF:SELECT_S', sources)
    tmbf.set('TRG:BUF:ARM_S', 0)
    a = bufa.get()[tmbf.bunches:][:length]
    b = bufb.get()[tmbf.bunches:][:length]
    return find_one_peak(a), find_one_peak(b)

# Measures delay from reference bunch to selected DDR waveform
def dac_to_ddr(tmbf, ddr_buf, source, length = -1):
    tmbf.set('DDR:INPUT_S', source)
    tmbf.set('TRG:DDR:ARM_S', 0)
    return find_one_peak(ddr_buf.get()[:length])


# Measures which bunch has its gain controlled by bunch select 0
def gain_delay(tmbf, maxdac):
    configure.bank_wf(tmbf, 0, one_bunch(tmbf, 1, 0), 0, DAC_OUT.NCO)
    return find_one_peak(maxdac.get_new(0.25))


# Measures which bunch has its FIR controlled by bunch select 0
def fir_to_ddr(tmbf, ddr_buf):
    configure.fir_wf(tmbf, 1, 0)
    configure.bank_wf(tmbf, 0, 1, one_bunch(tmbf, 0, 1), DAC_OUT.NCO)

    tmbf.set('DDR:INPUT_S', 'FIR')
    tmbf.set('TRG:DDR:ARM_S.PROC', 0)
    return find_one_peak(ddr_buf.get()[:tmbf.bunches])


# Classical binary search.  test(a,b) should return true iff the target is in
# the half open range [a,b), and the test range is [start,end).
def binary_search(start, end, test):
    assert test(start, end), 'Nothing present to search for!'
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
    outwf = numpy.zeros(tmbf.bunches)

    def test(start, end):
        outwf[:start] = DAC_OUT.OFF
        outwf[start:end] = DAC_OUT.SWEEP
        outwf[end:] = DAC_OUT.OFF
        tmbf.set('BUN:1:OUTWF_S', outwf)

        tmbf.set('TRG:BUF:ARM_S.PROC', 0)
        return det_power.get().mean() > 0

    return binary_search(0, tmbf.bunches, test)

def search_ftun_bunch(tmbf, mag, source):
    tmbf.set('FTUN:INPUT_S', source)

    outwf = numpy.zeros(tmbf.bunches)

    def test(start, end):
        outwf[:start] = DAC_OUT.OFF
        outwf[start:end] = DAC_OUT.NCO
        outwf[end:] = DAC_OUT.OFF
        tmbf.set('BUN:0:OUTWF_S', outwf)

        return mag.get_new(0.3) > 0

    return binary_search(0, tmbf.bunches, test)


# Computes group delay from scaled IQ.
def compute_group_delay(tmbf, scale, iq):
    angle = numpy.unwrap(numpy.angle(iq))
    fit = numpy.polyfit(scale, angle, 1)
    slope = fit[0]
    mean_error = numpy.std(angle - numpy.polyval(fit, scale))

    assert mean_error < 0.001, 'Unexpected error in slope fit'

    # Convert slope into revolutions per bunch to get group delay (frequency
    # scale is in units of tune).
    return int(round(slope/2/numpy.pi * tmbf.bunches))


def search_det_delay(tmbf, det_i, det_q, source):
    # Configure source
    tmbf.set('DET:INPUT_S', source)

    # Capture data
    tmbf.set('TRG:BUF:ARM_S', 0)
    iq = numpy.dot([1, 1j], [det_i.get(), det_q.get()])
    scale = tmbf.get('DET:SCALE')
    return compute_group_delay(tmbf, scale, iq)


def search_ftun_delay(tmbf, angle, source):
    tmbf.set('FTUN:INPUT_S', source)

    scale = 10.1 + 0.01 * numpy.arange(20)
    angles = numpy.zeros(len(scale))
    for n, f in enumerate(scale):
        tmbf.set('NCO:FREQ_S', f)
        angles[n] = angle.get_new(0.3)
    iq = numpy.exp(1j * numpy.pi * angles / 180.)
    return compute_group_delay(tmbf, scale, iq)


# ------------------------------------------------------------------------------
# Timing test

# We'll gather final configuration parameters for printout at the end.
class Results:
    def __init__(self, tmbf):
        self.__dict__['bunches'] = tmbf.bunches
        self.__dict__['items'] = []

    def set(self, name, value):
        self.items.append(name)
        self.__dict__[name] = value

    def set4(self, name, value):
        assert value % 4 == 0, \
            'Value %s = %d out of phase' % (name, value)
        self.set(name, value / 4)

    def __setattr__(self, name, value):
        # Ensure value is positive
        if value < 0:
            print 'adjusting', name, value
            value += self.bunches
        assert 0 <= value < self.bunches, \
            'Value %s = %d out of range' % (name, value)
        assert value % 4 == 0, \
            'Value %s = %d out of phase' % (name, value)
        self.set(name, value / 4)

    def print_results(self):
        for item in self.items:
            print item, '=', getattr(self, item)


# Performs initial calibration: measures DAC to ADC and DAC min/max buffer and
# closes the loop for the remaining measurements.
def measure_minmax(tmbf, results):
    print >>sys.stderr, 'Measuring DAC/ADC min/max buffer'

    configure_dac_single_pulse(tmbf)
    # Turn off DAC output delay for initial measurement.
    tmbf.set('DAC:DELAY_S', 0)

    maxdac = tmbf.PV('DAC:MAXBUF')
    maxadc = tmbf.PV('ADC:MAXBUF')

    # Start by taking DAC output mux select bunch zero as the reference bunch
    # and measuring delays to DAC max.  This allows us to measure the closed
    # loop delay and close the loop for all remaining measurements.
    dac_minmax_delay = find_one_peak(maxdac.get_new(0.25))
    results.MINMAX_DAC_DELAY = dac_minmax_delay

    # Check that the gain control already acts on bunch zero: this is programmed
    # to be thus in the FPGA at present.
    results.BUNCH_GAIN_OFFSET = dac_minmax_delay - gain_delay(tmbf, maxdac)

    # Measure loop delay and close the loop
    loop_delay = dac_to_dac_closed_loop(tmbf, maxdac)
    tmbf.set('DAC:DELAY_S', tmbf.bunches - loop_delay + dac_minmax_delay)

    # Now we can capture the max ADC offset
    configure_dac_single_pulse(tmbf)
    results.MINMAX_ADC_DELAY = find_one_peak(maxadc.get_new(0.25))

    maxdac.close()
    maxadc.close()


# Measures delays for BUF
def measure_buf(tmbf, results):
    print >>sys.stderr, 'Measuring BUF'

    buf_a = tmbf.PV('BUF:WFA')
    buf_b = tmbf.PV('BUF:WFB')
    buf_a.get()
    buf_b.get()

    # First get the baseline DAC delay.
    configure_dac_single_pulse(tmbf)

    _, base_dac_delay = dac_to_buf(tmbf, buf_a, buf_b, 'ADC+DAC', tmbf.bunches)
    results.set4('BUF_DAC_DELAY', base_dac_delay)

    # Now grab all the delays including processing time.
    configure_one_shot_pulse(tmbf, 'BUF')

    adc_delay, dac_delay = dac_to_buf(tmbf, buf_a, buf_b, 'ADC+DAC')
    fir_delay, _ = dac_to_buf(tmbf, buf_a, buf_b, 'FIR+DAC')

    # Figure out how much delay there is in the DAC pulse, and add one more turn
    # to this for the loop delay for measuring ADC and FIR.
    loop_delay = dac_delay - base_dac_delay + tmbf.bunches
    assert loop_delay % tmbf.bunches == 0, \
        'Unexpected loop_delay: %d' % loop_delay
    results.set4('BUF_ADC_DELAY', adc_delay - loop_delay)
    results.set4('BUF_FIR_DELAY', fir_delay - loop_delay)

    buf_a.close()
    buf_b.close()


# Similarly, measure triggered DDR data.  Also measure FIR bunch zero control
# while we have the DDR buffer open.
def measure_ddr(tmbf, results):
    print >>sys.stderr, 'Measuring DDR'

    ddr_buf = tmbf.PV('DDR:SHORTWF')
    ddr_buf.get()

    # As for BUF, first get the baseline DAC delay.
    configure_dac_single_pulse(tmbf)
    configure.sequencer_disable(tmbf)
    base_dac_delay = dac_to_ddr(tmbf, ddr_buf, 'DAC', tmbf.bunches)
    results.set4('DDR_DAC_DELAY', base_dac_delay)

    dac_to_ddr_fir_delay = dac_to_ddr(tmbf, ddr_buf, 'FIR', tmbf.bunches)
    fir_bunch_zero = fir_to_ddr(tmbf, ddr_buf)
    results.BUNCH_FIR_OFFSET = dac_to_ddr_fir_delay - fir_bunch_zero

    # Now get all delays using one shot sequencer
    configure_one_shot_pulse(tmbf, 'DDR')

    dac_delay = dac_to_ddr(tmbf, ddr_buf, 'DAC')
    raw_dac_delay = dac_to_ddr(tmbf, ddr_buf, 'Raw DAC')
    adc_delay = dac_to_ddr(tmbf, ddr_buf, 'ADC')
    fir_delay = dac_to_ddr(tmbf, ddr_buf, 'FIR')
    loop_delay = dac_delay - base_dac_delay + tmbf.bunches

    results.set4('DDR_ADC_DELAY', adc_delay - loop_delay)
    results.set4('DDR_FIR_DELAY', fir_delay - loop_delay)
    results.set4('DDR_RAW_DAC_DELAY', raw_dac_delay - loop_delay + tmbf.bunches)

    ddr_buf.close()


# Discover the detector bunch offset by binary search.
def measure_detector_bunch(tmbf, results):
    print >>sys.stderr, 'Measuring Detector Bunch Offsets'

    # Configure detector for narrow band sweep and soft trigger
    setup_tune_sweep(tmbf, 1.31, 1e-5, 20, '0dB')
    configure.detector_bunches(tmbf, 0)
    configure.detector_gain(tmbf, '-24dB')

    configure.bank(tmbf, 0, 0, 0, DAC_OUT.OFF)
    configure.bank_wf(tmbf, 1, 1, 0, DAC_OUT.SWEEP)

    det_power = tmbf.PV('DET:POWER:0')
    det_power.get()

    results.DET_ADC_OFFSET = search_det_bunch(tmbf, det_power, 'ADC')
    results.DET_FIR_OFFSET = search_det_bunch(tmbf, det_power, 'FIR')
    det_power.close()


def measure_detector_delay(tmbf, results):
    print >>sys.stderr, 'Measuring Detector Group Delays'

    setup_tune_sweep(tmbf, 1, 0.1, 20, '0dB')
    configure.detector_bunches(tmbf)
    configure.detector_gain(tmbf, '-72dB')

    configure.bank(tmbf, 1, 1, 0, DAC_OUT.SWEEP)

    tmbf.set('DET:LOOP:ADC_S', 1)

    det_i = tmbf.PV('DET:I:M')
    det_q = tmbf.PV('DET:Q:M')
    det_i.get(); det_q.get()

    results.set('DET_ADC_DELAY',
        search_det_delay(tmbf, det_i, det_q, 'ADC'))
    results.set('DET_FIR_DELAY',
        search_det_delay(tmbf, det_i, det_q, 'FIR') - tmbf.bunches)

    det_i.close()
    det_q.close()


def measure_tune_follow_bunch(tmbf, results):
    print >>sys.stderr, 'Measuring Tune Follow Bunch Offset'

    configure.bank(tmbf, 0, 1, 0, DAC_OUT.SWEEP)

    tmbf.set('FTUN:DWELL_S', 10)
    tmbf.set('FTUN:BUNCH_S', 0)
    tmbf.set('FTUN:MULTIBUNCH_S', 'Single Bunch')
    tmbf.set('FTUN:GAIN_S', '-24dB')
    tmbf.set('FTUN:BLANKING_S', 'Off')

    mag = tmbf.PV('FTUN:MAG')
    results.FTUN_ADC_OFFSET = search_ftun_bunch(tmbf, mag, 'ADC')
    results.FTUN_FIR_OFFSET = search_ftun_bunch(tmbf, mag, 'FIR')
    mag.close()


def measure_tune_follow_delay(tmbf, results):
    print >>sys.stderr, 'Measuring Tune Follow Delay'

    configure.bank(tmbf, 0, 1, 0, DAC_OUT.NCO)

    tmbf.set('FTUN:DWELL_S', 100)
    tmbf.set('FTUN:BUNCH_S', 0)
    tmbf.set('FTUN:MULTIBUNCH_S', 'All Bunches')
    tmbf.set('FTUN:GAIN_S', '-48dB')
    tmbf.set('FTUN:BLANKING_S', 'Off')
    tmbf.set('NCO:GAIN_S', '-36dB')
    tmbf.set('FTUN:IQ:IIR_S', '2^6')
    tmbf.set('FTUN:LOOP:DELAY_S', 0)

    angle = tmbf.PV('FTUN:ANGLE')
    results.set('FTUN_ADC_DELAY', search_ftun_delay(tmbf, angle, 'ADC'))
    results.set('FTUN_FIR_DELAY', search_ftun_delay(tmbf, angle, 'FIR'))
    angle.close()


# ------------------------------------------------------------------------------

actions = [
    'minmax',
    'buf',
    'ddr',
    'detector_bunch',
    'detector_delay',
    'tune_follow_bunch',
    'tune_follow_delay',
]


import argparse
parser = argparse.ArgumentParser(description = 'Measure internal TMBF delays')
parser.add_argument('-t', '--test', action = 'store_true',
    help = 'Verify current delays')
parser.add_argument('tmbf', nargs = '?', default = 'TS-DI-TMBF-01',
    help = 'TMBF machine name to test')
parser.add_argument('-m', '--measure', action = 'append',
    help = 'Specify measurements to perform, any of: ' + ', '.join(actions))
args = parser.parse_args()


tmbf = configure.TMBF(args.tmbf)
results = Results(tmbf)

configure_timing_test(tmbf, args.test)

if args.measure:
    actions = args.measure
for action in actions:
    globals()['measure_' + action](tmbf, results)

results.print_results()
