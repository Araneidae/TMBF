# Helper classes for TMBF control

import cothread
from cothread.catools import *

class TMBF:
    def __init__(self, name):
        self.tmbf = name
        self.s1 = Trigger(self, 'S1')
        self.s2 = Trigger(self, 'S2')
        self.ext = Trigger(self, 'EXT')

        self.saves = {}

    def pv(self, name):
        return '%s:%s' % (self.tmbf, name)

    def PV(self, name):
        return PV(self.pv(name))

    def set_save(self, name, value):
        self.saves[name] = self.get(name)
        caput(self.pv(name), value, wait=True)

    def set(self, name, value):
        caput(self.pv(name), value, wait=True)

    def get(self, name):
        return caget(self.pv(name), format = FORMAT_TIME)

    def monitor(self, name, on_update, **kargs):
        return camonitor(self.pv(name), on_update, **kargs)

    def restore_saved(self):
        for name, value in self.saves.items():
            self.set(name, value)

    def reset_saved(self):
        self.saves = {}


class Trigger:
    def __init__(self, tmbf, trigger):
        assert trigger in ['S1', 'S2', 'EXT']
        self.tmbf = tmbf
        self.trigger = trigger

        tmbf.monitor('TRG:%s:STATUS' % trigger,
            self.__update_status, datatype = str, all_updates = True)
        self.state = 'Unknown'
        self.event = cothread.Event()

    def __update_status(self, value):
        self.state = value
        self.event.Signal()

    def wait_for(self, state, timeout=5):
        timeout = cothread.AbsTimeout(timeout)
        while self.state != state:
            self.event.Wait(timeout)

    def arm(self, timeout=5):
        timeout = cothread.AbsTimeout(timeout)
        self.state = 'Waiting'
        arm_name = {'S1':'FIRE', 'S2':'FIRE', 'EXT':'ARM'}[self.trigger]
        self.tmbf.set('TRG:%s:%s_S.PROC' % (self.trigger, arm_name), 0)

        self.wait_for('Busy', timeout)
        self.wait_for('Ready', timeout)

