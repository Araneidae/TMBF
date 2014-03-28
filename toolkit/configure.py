# TMBF configuration functions

import numpy
import time

import cothread
from cothread.catools import *


BUNCH_COUNT = 936   # Repeated elsewhere


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

        self.n_taps = self.get('FIR:0:TAPS_S.NORD')

    def pv(self, name):
        return '%s:%s' % (self.tmbf, name)

    def PV(self, name):
        return PV(self.pv(name))

    def set(self, name, value):
        caput(self.pv(name), value, wait=True)

    def get(self, name):
        return caget(self.pv(name), format = FORMAT_TIME)



class MinMax:
    def __init__(self, tmbf, source):
        self.tmbf = tmbf
        self.source = source

    def wait(self):
        '''Waits for a fresh value from the specified source.'''

    def get_max(self):
        '''Returns MAX waveform.'''
        return self.tmbf.get('%s:MAXBUF' % self.source)

    def get_min(self):
        '''Returns MIN waveform.'''
        return self.tmbf.get('%s:MINBUF' % self.source)


class Trigger:
    def __init__(self, tmbf, name):
        self.tmbf = tmbf
        self.name = name


class DataSource:
    def __init__(self):
        pass


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
            value = numpy.repeat(value, BUNCH_COUNT)
        assert value.size == BUNCH_COUNT, 'Invalid array length'
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
def state(tmbf, state,
        start, step, dwell, gain, count=4096, bank=1, window=True, holdoff=2):
    tmbf.set('SEQ:%d:START_FREQ_S' % state, start)
    tmbf.set('SEQ:%d:STEP_FREQ_S' % state, step)
    tmbf.set('SEQ:%d:DWELL_S' % state, dwell)
    tmbf.set('SEQ:%d:COUNT_S' % state, count)
    tmbf.set('SEQ:%d:BANK_S' % state, bank)
    tmbf.set('SEQ:%d:GAIN_S' % state, gain)
    tmbf.set('SEQ:%d:ENWIN_S' % state, window)
    tmbf.set('SEQ:%d:HOLDOFF_S' % state, holdoff)

def sequencer_pc(tmbf, count):
    tmbf.set('SEQ:PC_S', count)


# ------------------------------------------------------------------------------
# Detector setup

def nco(tmbf, frequency, gain):
    tmbf.set('NCO:FREQ_S', frequency)
    tmbf.set('NCO:GAIN_S', gain)

def detector_input(tmbf, source):
    tmbf.set('DET:INPUT_S', source)

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


# ------------------------------------------------------------------------------
# Trigger setup

def trigger_ddr_source(tmbf, source):
    tmbf.set('TRG:DDR:SEL_S', source)

def trigger_buf_source(tmbf, source):
    tmbf.set('TRG:BUF:SEL_S', source)

def sequencer_enable(tmbf, enable):
    if enable:
        tmbf.set('TRG:SEQ:ENA_S', 'Enabled')
    else:
        tmbf.set('TRG:SEQ:ENA_S', 'Disabled')
        tmbf.set('SEQ:RESET_S.PROC', 0)

class WaitReady:
    def __init__(self, tmbf, status):
        self.monitor = camonitor(
            tmbf.pv(status), self.update, datatype = str, all_updates = True)
        self.state = 'Passive'
        self.event = cothread.Event()
        self.Wait = self.event.Wait
    def update(self, value):
#         print 'update', value.name, value
        if self.state == 'Passive':
            if value == 'Busy':
                self.state = 'Busy'
        elif self.state == 'Busy':
            if value == 'Ready':
                self.event.Signal()
                self.monitor.close()

def fire_and_wait(tmbf, target):
    waiter = WaitReady(tmbf, 'TRG:%s:STATUS' % target)
    tmbf.set('TRG:%s:ARM_S.PROC' % target, 0)
    waiter.Wait()
