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
#            setattr(record, self.addr_name, '@' + address)
            setattr(record, self.addr_name, address)

            # Check for a description, make a report if none given.
            if 'DESC' not in fields:
                print 'No description for', name
                
            return record

    @classmethod
    def init(cls):
        for name, addr in [
                ('longin',    'INP'), ('longout',   'OUT'),
                ('ai',        'INP'), ('ao',        'OUT'),
                ('bi',        'INP'), ('bo',        'OUT'),
                ('stringin',  'INP'), ('stringout', 'OUT'),
                ('mbbi',      'INP'), ('mbbo',      'OUT'),
                ('waveform',  'INP')]:
            setattr(cls, name, cls.makeRecord(name, addr))

GenericDevice.init()
GenericDevice()



# ----------------------------------------------------------------------------
#           Record Generation Support
# ----------------------------------------------------------------------------

# Functions for creating tmbf records

def add_PINI(result):
    # Not nice: there isn't a clean way of asking the builder whether a field
    # is present!  So we have to try reading the value, and catching the key
    # error that arises if it's not there!
    try:
        result.VAL.Value()
    except KeyError:
        # VAL field not defined, so don't add 
        pass
    else:
        result.PINI = 'YES'
    return result
    

def aIn(name, **fields):
    return GenericDevice.ai(name, **fields)

def aOut(name, DRVL=None, DRVH=None, **fields):
    return add_PINI(GenericDevice.ao(name, address=name,
        OMSL = 'supervisory', 
        DRVL = DRVL,  DRVH = DRVH,
#        EGUL = DRVL,  EGUF = DRVH,
        **fields))


def boolIn(name, ZNAM=None, ONAM=None, **fields):
    return GenericDevice.bi(name, ZNAM = ZNAM, ONAM = ONAM, **fields)

def boolOut(name, ZNAM=None, ONAM=None, **fields):
    return add_PINI(GenericDevice.bo(name, address=name,
        OMSL = 'supervisory', 
        ZNAM = ZNAM, ONAM = ONAM, **fields))


def longIn(name, LOPR=None, HOPR=None, EGU=None, MDEL=-1, **fields):
    return GenericDevice.longin(name,
        MDEL = MDEL,  EGU  = EGU,
        LOPR = LOPR,  HOPR = HOPR, **fields)

def longOut(name, DRVL=None, DRVH=None, **fields):
    return add_PINI(GenericDevice.longout(name, address=name,
        OMSL = 'supervisory',
        DRVL = DRVL, DRVH = DRVH, **fields))


# Field name prefixes for mbbi/mbbo records.
_mbbPrefixes = [
    'ZR', 'ON', 'TW', 'TH', 'FR', 'FV', 'SX', 'SV',     # 0-7
    'EI', 'NI', 'TE', 'EL', 'TV', 'TT', 'FT', 'FF']     # 8-15

# Adds a list of (option, value [,severity]) tuples into field settings
# suitable for mbbi and mbbo records.
def process_mbb_values(fields, option_values):
#     def process_value(fields, prefix, option, value, severity=None):
#         fields[prefix + 'ST'] = option
#         fields[prefix + 'VL'] = value
#         if severity:
#             fields[prefix + 'SV'] = severity
#     for prefix, value in zip(_mbbPrefixes, option_values):
#         process_value(fields, prefix, *value)
    for prefix, (default, option_value) in \
            zip(_mbbPrefixes, enumerate(option_values)):
        if isinstance(option_value, tuple):
            option, value = option_value
        else:
            option, value = option_value, default
        fields[prefix + 'ST'] = option
        fields[prefix + 'VL'] = value
        
def mbbIn(name, *option_values, **fields):
    process_mbb_values(fields, option_values)
    return GenericDevice.mbbi(name, **fields)

def mbbOut(name, *option_values, **fields):
    process_mbb_values(fields, option_values)
    return add_PINI(GenericDevice.mbbo(name, address=name,
        OMSL = 'supervisory', **fields))


def stringIn(name, **fields):
    return GenericDevice.stringin(name, **fields)
    
def stringOut(name, **fields):
    return add_PINI(GenericDevice.stringin(name, **fields))

    
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

def longInOut(name, DRVL = None, DRVH = None, VAL = None):
    input = longIn(name + '_R')
    output = longOut(name + '_W', DRVL, DRVH, FLNK = input)
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
    'aIn',      'aOut',     'boolIn',   'boolOut',  'longIn',   'longOut',
    'mbbOut',   'mbbOut',   'stringIn', 'stringOut',    'Waveform',
    'aInOut',   'longInOut',    'wInOut']
