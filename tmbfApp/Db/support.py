# Support code for Python epics database generation.

import sys
import os

from pkg_resources import require
require("dls.builder==1.5")

from dls.builder import TemplateRecordNames, Configure
Configure(recordnames = TemplateRecordNames())
from dls.builder import *


# The generic device module is in our own root.
HomeDir = os.path.realpath(
    os.path.join(os.path.dirname(sys.argv[0]), '../..'))
ModuleVersion('GenericDevice', home=HomeDir, use_name=False)


# This class wraps the creation of records which talk directly to the
# Libera device driver.
class GenericDevice(hardware.Device):
    DbdFileList = ['generic.dbd']

    class makeRecord:
        def __init__(self, builder, addr_name):
            self.builder = getattr(records, builder)
            self.addr_name = addr_name

        def __call__(self, name, address=None, **fields):
            if address is None:
                address = name
            record = self.builder(name, **fields)
            record.DTYP = 'Generic'
            setattr(record, self.addr_name, '@' + address)

            # Check for a description, make a report if none given.
            if 'DESC' not in fields:
                print 'No description for', name
                
            return record

    @classmethod
    def init(cls):
        for name, addr in [
                ('ai',        'INP'), ('ao',        'OUT'),
                ('waveform',  'INP'), ('mbbo',      'OUT')]:
            setattr(cls, name, cls.makeRecord(name, addr))

GenericDevice.init()
GenericDevice()



# ----------------------------------------------------------------------------
#           Record Generation Support
# ----------------------------------------------------------------------------

# Functions for creating tmbf records


def aIn(name, **fields):
    return GenericDevice.ai(name, **fields)

def aOut(name, DRVL=None, DRVH=None, **fields):
    result = GenericDevice.ao(name, address=name,
        OMSL = 'supervisory', 
        DRVL = DRVL,  DRVH = DRVH,
#        EGUL = DRVL,  EGUF = DRVH,
        **fields)
    if 'VAL' in fields:
        result.PINI = 'YES'
    return result



# Field name prefixes for mbbi/mbbo records.
_mbbPrefixes = [
    'ZR', 'ON', 'TW', 'TH', 'FR', 'FV', 'SX', 'SV',     # 0-7
    'EI', 'NI', 'TE', 'EL', 'TV', 'TT', 'FT', 'FF']     # 8-15

def mbbOut(name, *option_values, **fields):
    for prefix, (default, option_value) in \
            zip(_mbbPrefixes, enumerate(option_values)):
        if isinstance(option_value, tuple):
            option, value = option_value
        else:
            option, value = option_value, default
        fields[prefix + 'ST'] = option
        fields[prefix + 'VL'] = value
    result = GenericDevice.mbbo(name, address=name,
        OMSL = 'supervisory', **fields)
    if 'VAL' in fields:
        result.PINI = 'YES'
    return result
    
def Waveform(name, length, FTVL='LONG', **fields):
    return GenericDevice.waveform(name, name,
        NELM = length,  FTVL = FTVL, **fields)


def aInOut(name, DRVL = None, DRVH = None, VAL = None):
    input = aIn(name + '_R')
    output = aOut(name + '_W', DRVL, DRVH, FLNK = input)
    if VAL is None:
        # No initial value given, so make input auto initialise
        input.PINI = 'YES'
    else:
        # Initial value given, so write this on startup
        output.VAL  = VAL
        output.PINI = 'YES'

def wInOut(name, NELM):
    input = Waveform(name + '_R', NELM, PINI = 'YES')
    Waveform(name + '_W', NELM)


    
__all__ = [
    'aIn',      'aOut',     'mbbOut',   'Waveform',
    'aInOut',   'wInOut']
