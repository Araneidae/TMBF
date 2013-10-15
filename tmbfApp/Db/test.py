from common import *

longOut("TEST:LONGOUT", PINI='NO')
longOut("WRITE:LONGOUT", PINI='NO')
longOut("TEST:ULONGOUT", PINI='NO')
longOut("WRITE:ULONGOUT", PINI='NO')
aOut("TEST:AO", PINI='NO')
aOut("WRITE:AO", PINI='NO')
boolOut("TEST:BO", "False", "True", PINI='NO')
boolOut("WRITE:BO", "False", "True", PINI='NO')
stringOut("TEST:STRINGOUT", PINI='NO')
stringOut("WRITE:STRINGOUT", PINI='NO')
mbbOut("TEST:MBBO", "Zero", "One", "Two", PINI='NO')
mbbOut("WRITE:MBBO", "Zero", "One", "Two", PINI='NO')

longOut("TEST:LOOP1", PINI='NO')
longOut("TEST:LOOP2", PINI='NO')
