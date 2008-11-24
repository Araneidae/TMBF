TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

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
tmbf_SRCS += tmbfMain.c
tmbf_SRCS_vxWorks += -nil-

#The following adds support from base/src/vxWorks
tmbf_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

tmbf_LIBS += $(EPICS_BASE_IOC_LIBS)


#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

# Force tmbfMain to be rebuilt every time so that the build date and time is
# current. 
tmbfMain.o: EMPTY
EMPTY:
