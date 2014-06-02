import sys

from common import *

import adcdac       # ADC, DAC
import bunch_select # BUN
import ddr          # DDR
import detector     # DET, BUF
import fir          # FIR
import sensors      # SE
import sequencer    # SEQ
import triggers     # TRG
import tune         # TUNE
import tune_follow  # FTUN


stringIn('VERSION', PINI = 'YES', DESC = 'TMBF version')
longIn('FPGAVER', PINI = 'YES', DESC = 'FPGA version')

Action('RESTART', DESC = 'Restart EPICS driver')
Action('REBOOT', DESC = 'Reboot IOC')

WriteRecords(sys.argv[1])
