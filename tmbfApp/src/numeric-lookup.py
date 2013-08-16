# This file is part of the Libera EPICS Driver,
# Copyright (C) 2005-2009  Michael Abbott, Diamond Light Source Ltd.
#
# The Libera EPICS Driver is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or (at your
# option) any later version.
#
# The Libera EPICS Driver is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
# Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc., 51
# Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
#
# Contact:
#      Dr. Michael Abbott,
#      Diamond Light Source Ltd,
#      Diamond House,
#      Chilton,
#      Didcot,
#      Oxfordshire,
#      OX11 0DE
#      michael.abbott@diamond.ac.uk

# Python script to compute support lookup tables.  These tables are used to
# implement fast arithmetic operations.  See support.c for the use of
# definitions in this file.

import math

def lround(x):
    return int(round(x))


print '/* Support lookup tables for fast numerical computations.'
print ' * Automatically generated by support-lookup.py. */'
print
print



# -----------------------------------------------------------------------------
#  Reciprocal

DIV_N = 256


# Computes division lookup entry.
#   A is an 8 bit signed number representing 2**8 + A
def Division(A):
    A_0 = 2*(2**8 + A) + 1
    D_A = (2**19 / A_0 + 1) / 2
    return D_A & 0xFF


print '/* Division lookup table. */'
print
print 'static const unsigned char DivideLookup[%d] =' % DIV_N
print '{'

for i in range(DIV_N):
    if i % 8 == 0:
        print '   ',
    print '0x%02x,' % Division(i),
    if (i+1) % 8 == 0:
        print

print '};'
print
print



# -----------------------------------------------------------------------------
#  log2

# Number of bits to generate lookup for.  This value is a trade-off between
# memory consumption and number of bits of precision according to the
# following formulae:
#                                          N+2
#   Precision = 2*N + 2 bits,    Memory = 2    bytes
#
# So N=10 implies 22 bits of precision at a cost of 4K bytes lookup table.


LOG2_N = 10
LOG2_M = 31 - LOG2_N

ln2 = math.log(2)


# Computes log2 lookup entries, which should be
#   2^27 log_2 a   and   2^28 / (a ln 2)
def Log2(A):
    a = 1 + 2**-LOG2_N * (A + 0.5)
    return lround(2**27 * math.log(a, 2)), lround(2**28 / (a * ln2))



print '/* log2 lookup table. */'
print
print '#define LOG2_N_BITS   %d' % LOG2_N
print '#define LOG2_M_BITS   %d' % LOG2_M
print '#define LOG2_N_MASK   %d' % ((1 << LOG2_N) - 1)
print '#define LOG2_M_MASK   %d' % ((1 << LOG2_M) - 1)
print '#define LOG2_B_OFFSET %d' % (1 << (LOG2_M-1))
print
print 'struct LOG2_LOOKUP { int Log; int Scale; };'
print 'static const struct LOG2_LOOKUP Log2Lookup[%d] =' % (2**LOG2_N)
print '{'

for i in range(2**LOG2_N):
    if i % 2 == 0:
        print '   ',
    print '{ 0x%08x, 0x%08x },   ' % Log2(i),
    if i % 2 == 1:
        print

print '};'
print
print



# -----------------------------------------------------------------------------
#  exp2

EXP2_N = 10
EXP2_M = 27 - EXP2_N


# Computes exp2 lookup entry, which should be
#   2^31 2^a
def Exp2(A):
    a = 2**-EXP2_N * (A + 0.5)
    return lround(2**(31 + a))

print '/* exp2 lookup table. */'
print
print '#define EXP2_N_BITS   %d' % EXP2_N
print '#define EXP2_M_BITS   %d' % EXP2_M
print '#define EXP2_N_MASK   %d' % ((1 << EXP2_N) - 1)
print '#define EXP2_M_MASK   %d' % ((1 << EXP2_M) - 1)
print '#define EXP2_B_OFFSET %d' % (1 << (EXP2_M-1))
print
print '#define EXP2_LN2      %d' % lround(2**31 * ln2)
print
print 'static const unsigned int Exp2Lookup[%d] =' % (2**EXP2_N)
print '{'

for i in range(2**EXP2_N):
    if i % 4 == 0:
        print '   ',
    print '0x%08x,' % Exp2(i),
    if i % 4 == 3:
        print

print '};'
print
print



# -----------------------------------------------------------------------------
#  Miscellaneous constants


print '/* Constants for db() calculation. */'
print

toDB_multiplier = 2e7 / math.log(10, 2)

print '#define TO_DB_OFFSET %d' % int(round(16 * toDB_multiplier))
print '#define TO_DB_FACTOR %d' % int(round(2**5 * toDB_multiplier))
print
print '#define FROM_DB_FACTOR %dU' % int(round(
    2**27 * 2**27 * math.log(10, 2) / 2e7))
print
print



# -----------------------------------------------------------------------------
#  cos_sin

# N = 10 gives us close to 24 bits of precision at the cost of an 8K lookup
# table, and a processing time under 300ns.  N = 12 gives just over 27 bits
# of precision.
COS_SIN_N = 10

print '/* sin & cos lookup table. */'
print
print '#define COS_SIN_N     %d' % COS_SIN_N
print '#define COS_SIN_PI    %d' % lround(2**28 * 2*math.pi)
print

print 'struct COS_SIN_LOOKUP { int cos, sin; };'
print 'static const struct COS_SIN_LOOKUP cos_sin_lookup_table[%d] = {' % \
    (2**COS_SIN_N)
for n in range(2**COS_SIN_N):
    angle = (n + 0.5) * math.pi / 4 / 2**COS_SIN_N
    if n % 2 == 0:
        print '    ',
    print '{ 0x%08x, 0x%08x },   ' % (
        lround(2**30 * math.cos(angle)), lround(2**30 * math.sin(angle))),
    if n % 2 == 1:
        print
print
print '};'
