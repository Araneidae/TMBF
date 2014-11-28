# TMBF configuration functions

import numpy
import time

import cothread
from cothread.catools import *


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
    def __init__(self, name, debug = False):
        self.tmbf = name
        self.debug = debug

        self.n_taps = self.get('FIR:N_TAPS')
        self.bunches = self.get('BUNCHES')

    def pv(self, name):
        return '%s:%s' % (self.tmbf, name)

    def PV(self, name):
        return PV(self.pv(name))

    def set(self, name, value):
        if self.debug:
            raw_input('set %s <= %s ' % (name, value))
        caput(self.pv(name), value, wait=True)

    def get(self, name):
        return caget(self.pv(name), format = FORMAT_TIME)



# ------------------------------------------------------------------------------
# FIR setup

# Configure FIR with internally computed waveform
def fir(tmbf, fir, freq, length, phase):
    tmbf.set('FIR:%d:CYCLES_S' % fir, freq)
    tmbf.set('FIR:%d:LENGTH_S' % fir, length)
    tmbf.set('FIR:%d:PHASE_S' % fir, phase)
    tmbf.set('FIR:%d:USEWF_S' % fir, 'Settings')

# Configure FIR with given taps waveform
def fir_wf(tmbf, fir, taps):
    # Zero extend taps to full size
    taps = numpy.array(taps)
    taps_out = numpy.zeros(tmbf.n_taps)
    taps_out[:numpy.size(taps)] = taps
    tmbf.set('FIR:%d:TAPS_S' % fir, taps_out)
    tmbf.set('FIR:%d:USEWF_S' % fir, 'Waveform')

def fir_gain(tmbf, gain):
    tmbf.set('FIR:GAIN_S', gain)


# ------------------------------------------------------------------------------
# Bunch Bank setup

# Configure selected bank for waveform control with given gain, fir and output
# waveform or constant values.
def bank_wf(tmbf, bank, gain, fir, output):
    # A bunch configuration can be either a single value or else an array of the
    # correct length.
    def bunches(value):
        value = numpy.array(value)
        if value.size == 1:
            value = numpy.repeat(value, tmbf.bunches)
        assert value.size == tmbf.bunches, 'Invalid array length'
        return value

    tmbf.set('BUN:%d:GAINWF_S' % bank, bunches(gain))
    tmbf.set('BUN:%d:FIRWF_S' % bank, bunches(fir))
    tmbf.set('BUN:%d:OUTWF_S' % bank, bunches(output))

def bank(tmbf, bank, gain, fir, output):
    bank_wf(tmbf, bank, gain, fir, output)


# ------------------------------------------------------------------------------
# Sequencer setup

# Configures bunch bank for quiescent sequencer state
def state0(tmbf, bank = 0):
    tmbf.set('SEQ:0:BANK_S', bank)

# Programs a single sequencer state
def state(tmbf, state = 1,
        start = 0, step = 0, dwell = 20, gain = '0dB',
        count = 4096, bank = 1, window = True, holdoff = 0):
    tmbf.set('SEQ:%d:START_FREQ_S' % state, start)
    tmbf.set('SEQ:%d:STEP_FREQ_S' % state, step)
    tmbf.set('SEQ:%d:DWELL_S' % state, dwell)
    tmbf.set('SEQ:%d:COUNT_S' % state, count)
    tmbf.set('SEQ:%d:BANK_S' % state, bank)
    tmbf.set('SEQ:%d:GAIN_S' % state, gain)
    tmbf.set('SEQ:%d:ENWIN_S' % state, window)
    tmbf.set('SEQ:%d:HOLDOFF_S' % state, holdoff)
    tmbf.set('SEQ:%d:BLANK_S' % state, 'Off')



# ------------------------------------------------------------------------------
# Detector setup

def detector_gain(tmbf, gain):
    tmbf.set('DET:AUTOGAIN_S', 'Fixed Gain')
    tmbf.set('DET:GAIN_S', gain)

def detector_bunches(tmbf, *bunches):
    if bunches:
        for channel, bunch in enumerate(bunches):
            tmbf.set('DET:BUNCH%d_S' % channel, bunch)
        tmbf.set('DET:MODE_S', 'Single Bunch')
    else:
        tmbf.set('DET:MODE_S', 'All Bunches')


def sequencer_disable(tmbf):
    tmbf.set('TRG:SEQ:SEL_S', 'Disabled')
    tmbf.set('SEQ:RESET_S.PROC', 0)

def sequencer_enable(tmbf, trigger):
    tmbf.set('TRG:SEQ:SEL_S', '%s trigger' % trigger)
