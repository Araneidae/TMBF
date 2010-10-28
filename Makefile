#Makefile at top of application tree
TOP = .
include $(TOP)/configure/CONFIG
DIRS += configure
DIRS += tmbfApp
DIRS += install_d
include $(TOP)/configure/RULES_TOP
