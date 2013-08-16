import sys

from common import *

import adcdac       # ADC, DAC
import buffer       # BUF
import bunch_select # BUN
import ddr          # DDR
import detector     # DET
import fir          # FIR
import sensors      # SE
import sequencer    # SEQ
import triggers     # TRG
import tune         # TUNE


stringIn('VERSION', PINI = 'YES', DESC = 'TMBF version')
longIn('FPGAVER', PINI = 'YES', DESC = 'FPGA version')
boolOut('LOOPBACK', 'Normal', 'Loopback', VAL = 0)

WriteRecords(sys.argv[1])
