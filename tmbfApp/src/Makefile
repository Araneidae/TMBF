TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#USR_CFLAGS += -S
#USR_CFLAGS += -E
USR_CFLAGS += -std=gnu99
USR_CFLAGS += -Werror -Wall -Wextra -Wno-unused-parameter

# Make sure that we pick up the version from install/CONFIG.
USR_CFLAGS += -DTMBF_VERSION='"$(shell \
    sed -n '/^VERSION=/{s///;p}' $(TOP)/install/CONFIG)"'
# Compute a sensible format of build date
USR_CFLAGS += -DBUILD_DATE_TIME='"$(shell date '+%Y-%m-%d %H:%M')"'


STATIC_BUILD = YES

#=============================
# build an ioc application

PROD_IOC = tmbf

DBD += tmbf.dbd
# tmbf.dbd will be made up from these files:
tmbf_DBD += base.dbd
tmbf_DBD += generic.dbd

DBD += generic.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
tmbf_SRCS += tmbf_registerRecordDeviceDriver.c 
tmbf_SRCS += device.c
tmbf_SRCS += GenericDevice.c
tmbf_SRCS += epics_device.c
tmbf_SRCS += hardware.c
tmbf_SRCS += tmbfMain.c

tmbf_LIBS += $(EPICS_BASE_IOC_LIBS)


#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

# Force tmbfMain to be rebuilt every time so that the build date and time is
# current. 
tmbfMain.o: EMPTY
EMPTY:

device.o: device_EPICS.c

%_EPICS.c: %.c epics_device.h
	$(CPP) -D__DEFINE_EPICS__ $(INCLUDES) $< | ../make_EPICS >$@
