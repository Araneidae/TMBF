#!/usr/bin/env dls-python

if __name__ == '__main__':
    from pkg_resources import require
    require('numpy')
    require('cothread')

from cothread.catools import *

class TMBF:
    def __init__(self, name):
        self.tmbf = name

    def pv(self, name):
        return '%s:%s' % (self.tmbf, name)

    def PV(self, name):
        return PV(self.pv(name))

    def set(self, name, value):
        caput(self.pv(name), value, wait=True)

    def get(self, name):
        return caget(self.pv(name), format = FORMAT_TIME)

def configure_feedback(tmbf, cycles, length, gain=None):
    # Configure FIR 0 to cycles/length
    tmbf.set('FIR:0:CYCLES_S', cycles)
    tmbf.set('FIR:0:LENGTH_S', length)
    tmbf.set('FIR:0:USEWF_S', 'Settings')
    if gain:
        tmbf.set('FIR:GAIN_S', gain)

    # Configure bank 0 for FIR only
    tmbf.set('BUN:0:FIR_S', 'FIR 0')
    tmbf.set('BUN:0:OUT_S', 'FIR')
    tmbf.set('BUN:0:GAIN_S', 1023)
    tmbf.set('BUN:0:USEWF_S', 'Settings')

    # Configure quiescent operation using bank 0 and enable output
    tmbf.set('SEQ:0:BANK_S', 'Bank 0')
    tmbf.set('DAC:ENABLE_S', 'On')


def reset_tmbf(tmbf):
    tmbf.set('COMPENSATE_S', 'Normal')
    tmbf.set('LOOPBACK_S', 'Normal')
    tmbf.set('BUF:SELECT_S', 'IQ')
    tmbf.set('DAC:ENABLE_S', 'On')

    tmbf.set('DET:RESET_WIN_S.PROC', 0)

    tmbf.set('SE:TEMP:KI_S', 40)
    tmbf.set('SE:TEMP:KP_S', 40)

    tmbf.set('SEQ:0:BANK_S', 'Bank 0')
    tmbf.set('SEQ:1:BANK_S', 'Bank 1')
    tmbf.set('SEQ:1:CAPTURE_S', 'Capture')
    tmbf.set('SEQ:1:ENWIN_S', 'Windowed')
    tmbf.set('SEQ:1:HOLDOFF_S', 2)
    tmbf.set('SEQ:1:COUNT_S', 4096)
    tmbf.set('SEQ:1:GAIN_S', ??)
    tmbf.set('SEQ:1:START_FREQ_S', ??)
    tmbf.set('SEQ:1:END_FREQ_S', ??)
    tmbf.set('SEQ:PC_S', 1)


if __name__ == '__main__':
    import sys
    tmbf = TMBF(sys.argv[1])
    reset_tmbf(tmbf)
