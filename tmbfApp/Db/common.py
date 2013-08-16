# Common definitions for all TMBF records

# It is important to import support before importing iocbuilder, as the support
# module initialises iocbuilder (and determines which symbols it exports!)
from support import *
from iocbuilder import *

KB = 1024
MB = KB * KB

SAMPLES_PER_TURN = 936
CHANNEL_COUNT = 4

BUF_DATA_LENGTH = 16384
TUNE_LENGTH = BUF_DATA_LENGTH / CHANNEL_COUNT



def dBrange(count, step, start = 0):
    return ['%sdB' % db for db in range(start, start + count*step, step)]


def Action(name, **kargs):
    return boolOut(name, PINI = 'NO', **kargs)

def Trigger(prefix, *pvs):
    done = Action('%s:DONE' % prefix, DESC = '%s processing done' % prefix)
    return boolIn('%s:TRIG' % prefix,
        SCAN = 'I/O Intr', DESC = '%s processing trigger' % prefix,
        FLNK = create_fanout('%s:TRIG:FAN' % prefix, *pvs + (done,)))

# Concatenates a list of lists
def concat(ll):
    return [x for l in ll for x in l]
