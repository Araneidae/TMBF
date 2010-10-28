dbLoadDatabase("dbd/pbpm.dbd", 0, 0)
pbpm_registerRecordDeviceDriver(pdbbase)

epicsEnvSet("EPICS_CA_MAX_ARRAY_BYTES", 1000000)

## Load record instances
dbLoadRecords("db/pbpm.db", "DEVICE=${HOSTNAME}")

epicsEnvSet "IOCSH_PS1" "${HOSTNAME}> "
iocInit()
