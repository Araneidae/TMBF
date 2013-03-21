TOP=..
include $(TOP)/configure/CONFIG

USR_CFLAGS += -march=armv5te
USR_CFLAGS += -std=gnu99
USR_CFLAGS += -Werror -Wall -Wextra -Wno-unused-parameter
USR_CFLAGS += -Wundef
USR_CFLAGS += -Wshadow
USR_CFLAGS += -Wcast-align
USR_CFLAGS += -Wcast-qual
USR_CFLAGS += -Wwrite-strings
USR_CFLAGS += -Wredundant-decls
USR_CFLAGS += -Wmissing-prototypes
USR_CFLAGS += -Wmissing-declarations
USR_CFLAGS += -Wstrict-prototypes

PROD_IOC = fp ddrInit

fp_SRCS += fp.c

ddrInit_SRCS += ddrInit.c

include $(TOP)/configure/RULES
