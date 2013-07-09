import sys

from common import *

import adcdac       # ADC, DAC
import buffer       # BUF
import bunch_select # BUN
import ddr          # DDR
import fir          # FIR
import sensors      # SE
import sequencer    # SEQ
import triggers     # TRG
import tune         # TUNE


stringIn('VERSION', PINI = 'YES', DESC = 'TMBF version')

WriteRecords(sys.argv[1])
