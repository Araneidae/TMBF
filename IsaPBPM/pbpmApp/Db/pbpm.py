import sys

# It is important to import support before importing iocbuilder, as the
# support module initialises iocbuilder (and determines which symbols it
# exports!)
from support import * 
from iocbuilder import *

MAX_ID = 2**8 - 1

# HB waveforms
fa_waveforms = [
    Waveform('FA%d:%s' % (n, a), 16384)
    for a in 'ABCDSQXY'
    for n in range(2)]
boolOut('FA:PROC', FLNK = create_fanout('FA:FAN', *fa_waveforms))

# CTRL space registers
longOut('FAI:CFGVAL')  # FAI configuration value
mbbOut('FAI:XPOSSEL', ('A', 0), ('B', 1), ('C', 2), ('D', 3), ('S', 4), ('Q', 5), ('X', 6), ('Y', 7), DESC = 'X Payload Select')
mbbOut('FAI:YPOSSEL', ('A', 0), ('B', 1), ('C', 2), ('D', 3), ('S', 4), ('Q', 5), ('X', 6), ('Y', 7), DESC = 'Y Payload Select')

# FA space registers
longOut('FA0:XOFS')
longOut('FA0:YOFS')
longOut('FA0:QOFS')
longOut('FA0:KX')
longOut('FA0:KY')
longOut('FA0:DBG')
boolOut('FA0:ADCOUT', 'Absolute', 'Signed', DESC = 'ADC Data Output Type')

# SA space registers
boolOut('SA0:RESET', 'Trigger', DESC = 'Reser SA0 ')
longOut('SA0:CLEAR')                            # FAI configuration value
longIn('SA0:XPOS', SCAN = '.1 second')          # SA rate x position data
longIn('SA0:YPOS', SCAN = '.1 second')          # SA rate x position data
longIn('SA0:TEST', SCAN = '1 second')           # SA rate x position data

# CC space
longOut('FF:BPMID', 0, MAX_ID, DESC = 'BPM id')
longOut('FF:FRAMELEN', DESC = 'Timeframe length')
boolOut('FF:LINK1:ENABLE', 'Disabled', 'Enabled', DESC = 'Link (Rocket IO) enable')
mbbOut('FF:LINK1:LOOPBACK', ('No loopback', 0), ('Serial', 1), ('Parallel', 2),DESC = 'Rocket IO loopback')

boolOut('FF:DATA_SELECT', 'Positions', 'Timestamps', DESC = 'Position data select')

inputs = [
    longIn('FF:VERSION'),
    longIn('FF:LINK1:PARTNER'),
    longIn('FF:LINK2:PARTNER'),
    longIn('FF:LINK3:PARTNER'),
    longIn('FF:LINK4:PARTNER'),
    boolIn('FF:LINK1:TX_UP','Link Down','Link Up', DESC = 'Transmit link status'),
    boolIn('FF:LINK1:RX_UP','Link Down','Link Up', DESC = 'Receive link status'),
    longIn('FF:TIMEFRAME'),
    longIn('FF:BPM_COUNT'),
    longIn('FF:LINK1:RX_CNT'),
    longIn('FF:LINK1:TX_CNT'),
    longIn('FF:LINK1:HARD_ERR'),
    longIn('FF:LINK1:SOFT_ERR'),
    longIn('FF:LINK1:FRAME_ERR'),
    longIn('FF:PROCESS_TIME', DESC = 'Total communication time')
]

boolOut('FF:PROCESS', DESC = 'Update FF fields', SCAN = '1 second', FLNK = create_fanout('FANOUT', *inputs))

WriteRecords(sys.argv[1])
