# Common definitions for all TMBF records

import os

# It is important to import support before importing iocbuilder, as the support
# module initialises iocbuilder (and determines which symbols it exports!)
from support import *
from iocbuilder import *

KB = 1024
MB = KB * KB

BUNCHES_PER_TURN = int(os.environ['BUNCHES_PER_TURN'])
CHANNEL_COUNT = 4

BUF_DATA_LENGTH = 16384
TUNE_LENGTH = BUF_DATA_LENGTH / CHANNEL_COUNT



def dBrange(count, step, start = 0):
    return ['%sdB' % db for db in range(start, start + count*step, step)]
