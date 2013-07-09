# Common definitions for all TMBF records

# It is important to import support before importing iocbuilder, as the support
# module initialises iocbuilder (and determines which symbols it exports!)
from support import *
from iocbuilder import *

KB = 1024
MB = KB * KB

SAMPLES_PER_TURN = 936


def Action(name, **kargs):
    return boolOut(name, PINI = 'NO', **kargs)

def Trigger(prefix, *pvs):
    done = Action('%s:DONE' % prefix, DESC = '%s processing done' % prefix)
    return boolIn('%s:TRIG' % prefix,
        SCAN = 'I/O Intr', DESC = '%s processing trigger' % prefix,
        FLNK = create_fanout('%s:TRIG:FAN' % prefix, *pvs + (done,)))
