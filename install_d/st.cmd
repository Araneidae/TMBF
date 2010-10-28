dbLoadDatabase("dbd/tmbf.dbd",0,0)
tmbf_registerRecordDeviceDriver(pdbbase) 

epicsEnvSet("EPICS_CA_MAX_ARRAY_BYTES",1000000)

## Load record instances
dbLoadRecords("db/tmbf.db", "${DB_PARAMETERS}")

epicsEnvSet "IOCSH_PS1" "${HOSTNAME}> "
iocInit()
