/* EPICS device driver interface.
 *
 * This file implements generic EPICS device support.
 *
 * The following record types are supported:
 *      longin, longout, ai, ao, bi, bo, stringin, stringout, mbbi, mbbo,
 *      waveform. */

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <devSup.h>
#include <recSup.h>
#include <dbScan.h>
#include <epicsExport.h>
#include <gpHash.h>

#include <alarm.h>
#include <dbFldTypes.h>
#include <recGbl.h>
#include <dbCommon.h>
#include <longinRecord.h>
#include <longoutRecord.h>
#include <aiRecord.h>
#include <aoRecord.h>
#include <biRecord.h>
#include <boRecord.h>
#include <stringinRecord.h>
#include <stringoutRecord.h>
#include <mbbiRecord.h>
#include <mbboRecord.h>
#include <waveformRecord.h>

#include "epics_device.h"


/* Special casting operation to bypass strict aliasing warnings. */
#define CAST(type, value) \
    ( { \
        union { typeof(value) a; type b; } __u; \
        __u.a = (value); \
        __u.b; \
    } )


/* Epics processing return codes. */
#define OK              0
#define ERROR           1
#define NO_CONVERT      2       // Special code for ai/ao conversion



void CopyEpicsString(const EPICS_STRING in, EPICS_STRING *out)
{
    /* Don't be foolish, there is no guarantee that strings are word aligned
     * -- use memcpy which knows how to handle this. */
    memcpy(out, in, sizeof(EPICS_STRING));
}



/****************************************************************************/
/*                       I_RECORD publish and lookup                        */
/****************************************************************************/

#define TABLESIZE   256

static struct gphPvt * hash_table = NULL;


/* Common information about records: this is written to the dpvt field. */
typedef struct RECORD_BASE
{
    const I_RECORD * iRecord;
    IOSCANPVT ioscanpvt;
    char WriteData[];
} RECORD_BASE;


/* Returns the size of data to be reserved for the RECORD_BASE::WriteData
 * field.  This is only used for output records. */
#define CASE_DATA_SIZE(record, VAL) \
    case RECORD_TYPE_##record: \
        return sizeof(((record##Record *)0)->VAL)
static size_t WriteDataSize(RECORD_TYPE record_type)
{
    switch (record_type)
    {
        CASE_DATA_SIZE(longout,     val);
        CASE_DATA_SIZE(ao,          val);
        CASE_DATA_SIZE(bo,          rval);
        CASE_DATA_SIZE(stringout,   val);
        CASE_DATA_SIZE(mbbo,        rval);
        default: return 0;
    }
}


static RECORD_BASE * create_RECORD_BASE(const I_RECORD *iRecord)
{
    size_t WriteSize = WriteDataSize(iRecord->record_type);
    RECORD_BASE * base = malloc(sizeof(RECORD_BASE) + WriteSize);
    if (base != NULL)
    {
        base->iRecord = iRecord;
        if (iRecord->io_intr)
            scanIoInit(&base->ioscanpvt);
        else
            base->ioscanpvt = NULL;
    }
    return base;
}


/* Looks up the given record by name. */
RECORD_HANDLE LookupRecord(const char * Name)
{
    GPHENTRY * entry = gphFind(hash_table, Name, NULL);
    if (entry == NULL)
    {
        printf("No record found for %s\n", Name);
        return NULL;
    }
    else
        return (RECORD_HANDLE) entry->userPvt;
}


RECORD_HANDLE PublishDynamic(const I_RECORD* iRecord)
{
    if (hash_table == NULL)
        gphInitPvt(&hash_table, TABLESIZE);
    
    GPHENTRY * entry = gphAdd(hash_table, iRecord->name, NULL);
    if (entry == NULL)
    {
        printf("Failed to add entry \"%s\"\n", iRecord->name);
        return NULL;
    }
    else
    {
        RECORD_BASE * base = create_RECORD_BASE(iRecord);
        entry->userPvt = (void *) base;
        return base;
    }
}


bool PublishEpicsData(const I_RECORD* publish_data[])
{
    for (int i = 0; publish_data[i] != NULL; i ++)
    {
        if (PublishDynamic(publish_data[i]) == NULL)
            return false;
    }
    return true;
}



/*****************************************************************************/
/*                                                                           */
/*                        Common Record Implementation                       */
/*                                                                           */
/*****************************************************************************/



static epicsAlarmSeverity get_alarm_status(I_RECORD *iRecord)
{
    if (iRecord->get_alarm_status)
        return iRecord->get_alarm_status(iRecord->context);
    else
        return epicsSevNone;
}

static bool get_timestamp(I_RECORD *iRecord, struct timespec *time)
{
    if (iRecord->get_timestamp)
        return iRecord->get_timestamp(iRecord->context, time);
    else
        return false;
}


/* Routine called (possibly in signal handler context, or in an arbitrary
 * thread) to notify that I/O Intr processing should occur. */
bool SignalRecord(RECORD_HANDLE base)
{
    if (base == NULL)
        return false;
    else if (base->ioscanpvt == NULL)
        return false;
    else
    {
        scanIoRequest(base->ioscanpvt);
        return true;
    }
}



/* Common I/O Intr scanning support: uses the fact that pr->dpvt always
 * contains the appropriate GetIoInt implementation. */

static long get_ioint_(int cmd, dbCommon *pr, IOSCANPVT *pIoscanpvt) 
{ 
    RECORD_BASE * base = (RECORD_BASE *) pr->dpvt; 
    if (base == NULL) 
        return ERROR; 
    else 
    {
        *pIoscanpvt = base->ioscanpvt;
        return OK; 
    } 
}



static void SetTimestamp(dbCommon *pr, struct timespec *Timestamp)
{
    /* Convert the standard Unix timespec value into the EPICS epoch
     * timestamp (subtract 20 years). */
    epicsTimeFromTimespec(&pr->time, Timestamp);
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                  Common macros for record definitions                     */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/* Reads a value from the appropriate record interface.  An intermediate
 * value is used so that the record interface type doesn't need to precisely
 * match the data type stored in the EPICS record. */
#define ACTION_READ_INDIRECT(record, iRecord, action, field) \
    ( { \
        TYPEOF(record) Value; \
        bool Ok = iRecord->action(iRecord->context, &Value); \
        if (Ok) field = Value; \
        Ok; \
    } )

#define ACTION_READ_DIRECT(record, iRecord, action, field) \
    iRecord->action(iRecord->context, &field)

/* Note that ACTION_READ_##record expands to either ACTION_READ_DIRECT or
 * ACTION_READ_INDIRECT, depending on the record in question. */
#define ACTION_read(record, pr, iRecord, action, field) \
    ( { \
        bool Ok = ACTION_READ_##record(record, iRecord, action, field); \
        pr->udf = ! Ok; \
        Ok; \
    } )


/* Writing is a bit more involved: if it fails then we need to restore the
 * value we had before. */
#define ACTION_write(record, pr, iRecord, action, field) \
    ( { \
        bool Ok = iRecord->action(iRecord->context, field); \
        if (Ok) \
            memcpy(base->WriteData, &field, sizeof(field)); \
        else \
            memcpy(&field, base->WriteData, sizeof(field)); \
        Ok; \
    } )



/* Helper code for extracting the appropriate I_record from the record. */
#define GET_RECORD(record, pr, base, var) \
    RECORD_BASE * base = (RECORD_BASE *) pr->dpvt; \
    if (base == NULL) \
        return ERROR; \
    I_##record * var = (I_##record *) base->iRecord



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                      init_record_##record definitions                     */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/* Common record initialisation.  Looks up and validates the record base. */

static bool init_record_(
    dbCommon *pr, const char * Name, RECORD_TYPE record_type)
{
    RECORD_BASE * base = LookupRecord(Name);
    if (base == NULL)
        return false;
    else if (base->iRecord->record_type != record_type)
    {
        printf("Record %s type mismatch: %d != %d\n",
            Name, base->iRecord->record_type, record_type);
        return false;
    }
    else
    {
        pr->dpvt = base;
        return true;
    }
}



/* Record initialisation post processing: ensures that the EPICS data
 * structures are appropriately initialised.  The data has already been read,
 * but we still need to set up the alarm state and give the data a sensible
 * initial timestamp. */

static void post_init_record_out(dbCommon *pr, I_RECORD *iRecord)
{
    (void) recGblSetSevr(pr, READ_ALARM, get_alarm_status(iRecord));
    recGblResetAlarms(pr); 
    struct timespec Timestamp;
    if (!get_timestamp(iRecord, &Timestamp))
        /* If the record doesn't have its own timestamp then synthesise one
         * instead from the real time clock. */
        clock_gettime(CLOCK_REALTIME, &Timestamp);
    SetTimestamp(pr, &Timestamp);
}



/* For inp records there is no special post initialisation processing. */
#define POST_INIT_inp(record, pr, field, INIT_OK)   return INIT_OK;

/* For out records we need to read the current underlying device value and
 * save a good copy as part of initialisation. */
#define POST_INIT_out(record, pr, field, INIT_OK) \
    { \
        GET_RECORD(record, pr, base, iRecord); \
        if (iRecord->init != NULL) \
            (void) ACTION_read(record, pr, iRecord, init, field); \
        memcpy(base->WriteData, &field, sizeof(field)); \
        post_init_record_out((dbCommon*)pr, (I_RECORD *) iRecord); \
        return INIT_OK; \
    }


/* Record initialisation is simply a matter of constructing an instance of
 * the appropriate record type.  This is then followed by record specific
 * extra initialisation. */
#define INIT_RECORD(record, VAL, inOrOut, post_init, INIT_OK) \
    static long init_record_##record(record##Record *pr) \
    { \
        if (init_record_( \
                (dbCommon *) pr, pr->inOrOut.value.constantStr, \
                RECORD_TYPE_##record)) \
            POST_INIT_##post_init(record, pr, pr->VAL, INIT_OK) \
        else \
            return ERROR; \
    }



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                          Record processing support                        */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* Common record post-processing.  Updates the alarm state as appropriate,
 * and checks if the timestamp should be written here. */

static void post_process(dbCommon *pr, epicsEnum16 nsta, I_RECORD *iRecord)
{
    (void) recGblSetSevr(pr, nsta, get_alarm_status(iRecord));
    struct timespec Timestamp;
    if (get_timestamp(iRecord, &Timestamp))
        SetTimestamp(pr, &Timestamp);
}




/* Standard boiler-plate default record processing action.  The val field is
 * either read or written and the alarm state is set by interrogating the
 * record interface.  This processing is adequate for most record types. */
#define DEFINE_DEFAULT_PROCESS(record, VAL, action, PROC_OK) \
    static long action##_##record(record##Record * pr) \
    { \
        GET_RECORD(record, pr, base, iRecord); \
        bool Ok = ACTION_##action( \
            record, pr, iRecord, action, pr->VAL); \
        post_process((dbCommon *)pr, action##_ALARM, (I_RECORD *) iRecord); \
        return Ok ? PROC_OK : ERROR; \
    }

#define read_ALARM      READ_ALARM
#define write_ALARM     WRITE_ALARM




#define DEFINE_DEFAULT_READ(record, VAL, INIT_OK, PROC_OK) \
    INIT_RECORD(record, VAL, inp, inp, INIT_OK) \
    DEFINE_DEFAULT_PROCESS(record, VAL, read, PROC_OK) 
#define DEFINE_DEFAULT_WRITE(record, VAL, INIT_OK, PROC_OK) \
    INIT_RECORD(record, VAL, out, out, INIT_OK) \
    DEFINE_DEFAULT_PROCESS(record, VAL, write, PROC_OK)


#if 0
/* Redefine epicsExportAddress to avoid strict aliasing warnings. */
#undef epicsExportAddress
#define epicsExportAddress(typ,obj) \
    epicsShareExtern typeof(obj) *EPICS_EXPORT_POBJ(typ, obj); \
    epicsShareDef typeof(obj) *EPICS_EXPORT_POBJ(typ, obj) = &obj
#endif


#define DEFINE_DEVICE(record, length, args...) \
    static struct record##Device record##Generic = \
    { \
        length, \
        NULL, \
        NULL, \
        init_record_##record, \
        get_ioint_, \
        args \
    }; \
    epicsExportAddress(dset, record##Generic)



/*****************************************************************************/
/*                                                                           */
/*                        Device Driver Implementations                      */
/*                                                                           */
/*****************************************************************************/

/* Type adapters.  Some types need to be read directly, others need to be
 * read via the read adapter. */
#define ACTION_READ_longin     ACTION_READ_DIRECT
#define ACTION_READ_longout    ACTION_READ_DIRECT
#define ACTION_READ_ai         ACTION_READ_DIRECT
#define ACTION_READ_ao         ACTION_READ_DIRECT
#define ACTION_READ_bi         ACTION_READ_INDIRECT
#define ACTION_READ_bo         ACTION_READ_INDIRECT
#define ACTION_READ_stringin   ACTION_READ_DIRECT
#define ACTION_READ_stringout  ACTION_READ_DIRECT
#define ACTION_READ_mbbi       ACTION_READ_INDIRECT
#define ACTION_READ_mbbo       ACTION_READ_INDIRECT

    

/* Mostly we can use simple boilerplate for the process routines. */
DEFINE_DEFAULT_READ (longin,    val,    OK,         OK)
DEFINE_DEFAULT_WRITE(longout,   val,    OK,         OK)
DEFINE_DEFAULT_READ (ai,        val,    OK,         NO_CONVERT)
DEFINE_DEFAULT_WRITE(ao,        val,    NO_CONVERT, OK)
DEFINE_DEFAULT_READ (bi,        rval,   OK,         OK)
DEFINE_DEFAULT_WRITE(bo,        rval,   OK,         OK)
DEFINE_DEFAULT_READ (stringin,  val,    OK,         OK)
DEFINE_DEFAULT_WRITE(stringout, val,    OK,         OK)
DEFINE_DEFAULT_READ (mbbi,      rval,   OK,         OK)
DEFINE_DEFAULT_WRITE(mbbo,      rval,   OK,         OK)



/* Reading a waveform doesn't fit into the fairly uniform pattern established
 * for the other record types. */

/* Routine to validate record type: ensure that we don't mismatch the record
 * declarations in the code and in the database. */
static bool CheckWaveformType(waveformRecord * pr, I_waveform * iRecord)
{
    epicsEnum16 expected = DBF_NOACCESS;
    switch (iRecord->field_type)
    {
        case waveform_TYPE_void:    break;
        case waveform_TYPE_short:   expected = DBF_SHORT;   break;
        case waveform_TYPE_int:     expected = DBF_LONG;    break;
        case waveform_TYPE_float:   expected = DBF_FLOAT;   break;
        case waveform_TYPE_double:  expected = DBF_DOUBLE;  break;
    }
    if (pr->ftvl == expected)
        return true;
    else
    {
        printf("Array %s.FTVL mismatch %d != %d (%d)\n",
            iRecord->name, pr->ftvl, expected, iRecord->field_type);
        pr->dpvt = NULL;
        return false;
    }
}


#define POST_INIT_waveform(record, pr, field, INIT_OK) \
    { \
        GET_RECORD(record, pr, base, iRecord); \
        if (!CheckWaveformType(pr, iRecord)) \
            return ERROR; \
        if (iRecord->init != NULL) \
            pr->udf = iRecord->init( \
                iRecord->context, pr->bptr, CAST(size_t*, &pr->nord)); \
        post_init_record_out((dbCommon*) pr, (I_RECORD *) iRecord); \
        return INIT_OK; \
    }

INIT_RECORD(waveform, (unused), inp, waveform, OK)

static long process_waveform(waveformRecord * pr)
{
    GET_RECORD(waveform, pr, base, i_waveform);
    /* Naughty cast: I want to a reference to size_t, pr->nord is actually an
     * unsigned int.  Force the two to match! */
    bool Ok = i_waveform->process(
        i_waveform->context, pr->bptr, pr->nelm, CAST(size_t*, &pr->nord));
    post_process((dbCommon *) pr, READ_ALARM, (I_RECORD *) i_waveform);
    /* Note, by the way, that the waveform record support carefully ignores
     * my return code! */
    return Ok ? OK : ERROR;
}


/* Also need dummy special_linconv routines for ai and ao. */

static long linconv_ai(aiRecord *pr, int cmd) { return OK; }
static long linconv_ao(aoRecord *pr, int cmd) { return OK; }


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                          Device Driver Definitions                        */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "recordDevice.h"

DEFINE_DEVICE(longin,    5, read_longin);
DEFINE_DEVICE(longout,   5, write_longout);
DEFINE_DEVICE(ai,        6, read_ai,  linconv_ai);
DEFINE_DEVICE(ao,        6, write_ao, linconv_ao);
DEFINE_DEVICE(bi,        5, read_bi);
DEFINE_DEVICE(bo,        5, write_bo);
DEFINE_DEVICE(stringin,  5, read_stringin);
DEFINE_DEVICE(stringout, 5, write_stringout);
DEFINE_DEVICE(mbbi,      5, read_mbbi);
DEFINE_DEVICE(mbbo,      5, write_mbbo);
DEFINE_DEVICE(waveform,  5, process_waveform);
