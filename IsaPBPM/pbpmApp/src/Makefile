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
USR_CFLAGS += -DPBPM_VERSION='"$(shell \
    sed -n '/^VERSION=/{s///;p}' $(TOP)/install/CONFIG)"'
# Compute a sensible format of build date
USR_CFLAGS += -DBUILD_DATE_TIME='"$(shell date '+%Y-%m-%d %H:%M')"'


STATIC_BUILD = YES

#=============================
# build an ioc application

PROD_IOC = pbpm

DBD += pbpm.dbd
# pbpm.dbd will be made up from these files:
pbpm_DBD += base.dbd
pbpm_DBD += generic.dbd

DBD += generic.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
pbpm_SRCS += pbpm_registerRecordDeviceDriver.c 
pbpm_SRCS += device.c				# This is your basic example
pbpm_SRCS += epics_device.c			# EPICS driver!
pbpm_SRCS += hardware.c				# Good idea to separate h/w interface fns
pbpm_SRCS += pbpmMain.c				# Entry point?

pbpm_LIBS += $(EPICS_BASE_IOC_LIBS)


# The following sources have automatically generated PUBLISH statements.
EPICS_EXPORTS = device


#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

# Force pbpmMain to be rebuilt every time so that the build date and time is
# current. 
pbpmMain.o: EMPTY
EMPTY:

# Make everything listed in EPICS_EXPORTS depend on its associated .EPICS file
$(EPICS_EXPORTS:=.o): %.o: %.EPICS

%.EPICS: %.c epics_device.h
	$(CPP) -D__DEFINE_EPICS__ $(INCLUDES) $< | ../make_EPICS >$@
