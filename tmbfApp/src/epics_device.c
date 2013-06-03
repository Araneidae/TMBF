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

#include "error.h"
#include "hashtable.h"
#include "persistence.h"

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
    memcpy(out, in, sizeof(EPICS_STRING));
}



/****************************************************************************/
/*                   struct i_record publish and lookup                     */
/****************************************************************************/

static struct hash_table *hash_table = NULL;


/* Common information about records: this is written to the dpvt field. */
struct record_base
{
    const struct i_record *iRecord;
    IOSCANPVT ioscanpvt;
    char WriteData[];
};


/* Returns the size of data to be reserved for the record_base::WriteData
 * field.  This is only used for output records. */
#define CASE_DATA_SIZE(record, VAL) \
    case RECORD_TYPE_##record: \
        return sizeof(((record##Record *)0)->VAL)
static size_t WriteDataSize(enum record_type record_type)
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
#undef CASE_DATA_SIZE


/* Returns record persistence flag from iRecord. */
#define CASE_TAG(record) \
    case RECORD_TYPE_##record: \
        return ((const struct i_##record *) iRecord)->persist
static bool get_persistent(const struct i_record *iRecord)
{
    switch (iRecord->record_type)
    {
        CASE_TAG(longout);
        CASE_TAG(ao);
        CASE_TAG(bo);
        CASE_TAG(stringout);
        CASE_TAG(mbbo);
        case RECORD_TYPE_waveform:
            return ((const struct i_waveform_void *) iRecord)->persist;
        default: return NULL;
    }
}
#undef CASE_TAG

/* Returns associated persistence type. */
#define CASE_TYPE(record, type) \
    case RECORD_TYPE_##record: \
        return PERSISTENT_##type
#define CASE_WF_TYPE(type) \
    case waveform_TYPE_##type: return PERSISTENT_##type
static enum PERSISTENCE_TYPES get_persistent_type(
    const struct i_record *iRecord)
{
    switch (iRecord->record_type)
    {
        CASE_TYPE(longout, int);
        CASE_TYPE(ao, double);
        CASE_TYPE(bo, int);
        CASE_TYPE(stringout, string);
        CASE_TYPE(mbbo, int);
        case RECORD_TYPE_waveform:
            switch (((const struct i_waveform_void *) iRecord)->field_type)
            {
                CASE_WF_TYPE(char);
                CASE_WF_TYPE(short);
                CASE_WF_TYPE(int);
                CASE_WF_TYPE(float);
                CASE_WF_TYPE(double);
                default: return PERSISTENT_int;
            }
        default: return PERSISTENT_int;     // Safe, will not happen
    }
}
#undef CASE_TYPE
#undef CASE_WF_TYPE

static size_t get_record_length(const struct i_record *iRecord)
{
    switch (iRecord->record_type)
    {
        case RECORD_TYPE_waveform:
            return ((const struct i_waveform_void *) iRecord)->max_length;
        default:
            return 1;
    }
}


static struct record_base *create_record_base(const struct i_record *iRecord)
{
    size_t WriteSize = WriteDataSize(iRecord->record_type);
    struct record_base *base = malloc(sizeof(struct record_base) + WriteSize);
    if (base != NULL)
    {
        base->iRecord = iRecord;
        if (iRecord->io_intr)
            scanIoInit(&base->ioscanpvt);
        else
            base->ioscanpvt = NULL;

        if (get_persistent(iRecord))
            create_persistent_waveform(
                iRecord->name, get_persistent_type(iRecord),
                get_record_length(iRecord));
    }
    return base;
}


/* Looks up the given record by name. */
struct record_base *LookupRecord(const char *Name)
{
    struct record_base *handle = hash_table_lookup(hash_table, Name);
    if (handle == NULL)
        printf("No record found for %s\n", Name);
    return handle;
}


struct record_base *PublishDynamic(const struct i_record *iRecord)
{
    if (hash_table == NULL)
        hash_table = hash_table_create(false);

    /* Check entry doesn't already exist. */
    if (hash_table_lookup(hash_table, iRecord->name))
    {
        printf("Entry \"%s\" already exists!\n", iRecord->name);
        return NULL;
    }

    struct record_base *base = create_record_base(iRecord);
    hash_table_insert(hash_table, iRecord->name, base);
    return base;
}


bool PublishEpicsData(const struct i_record *publish_data[])
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



static epicsAlarmSeverity get_alarm_status(const struct i_record *iRecord)
{
    if (iRecord->get_alarm_status)
        return iRecord->get_alarm_status(iRecord->context);
    else
        return epicsSevNone;
}

static bool get_timestamp(const struct i_record *iRecord, struct timespec *ts)
{
    if (iRecord->get_timestamp)
        return iRecord->get_timestamp(iRecord->context, ts);
    else
        return false;
}


/* Routine called (possibly in signal handler context, or in an arbitrary
 * thread) to notify that I/O Intr processing should occur. */
bool SignalRecord(struct record_base *base)
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
    struct record_base *base = pr->dpvt;
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
        bool Ok_ = iRecord->action(iRecord->context, &Value); \
        if (Ok_) field = Value; \
        Ok_; \
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
 * value we had before, and the persistence layer may want to know. */
#define ACTION_write(record, pr, iRecord, action, field) \
    ( { \
        bool Ok = iRecord->action(iRecord->context, field); \
        if (Ok) \
        { \
            memcpy(base->WriteData, &field, sizeof(field)); \
            if (iRecord->persist) \
                write_persistent_variable(iRecord->name, &field); \
        } \
        else \
            memcpy(&field, base->WriteData, sizeof(field)); \
        Ok; \
    } )



/* Helper code for extracting the appropriate i_record from the record. */
#define GET_RECORD(record, pr, base, var) \
    struct record_base *base = pr->dpvt; \
    if (base == NULL) \
        return ERROR; \
    const struct i_##record *var = (const struct i_##record *) base->iRecord



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                      init_record_##record definitions                     */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/* Common record initialisation.  Looks up and validates the record base. */

static bool init_record_(
    dbCommon *pr, const char *Name, enum record_type record_type)
{
    struct record_base *base = LookupRecord(Name);
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

static void post_init_record_out(dbCommon *pr, const struct i_record *iRecord)
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
 * save a good copy as part of initialisation.  The logic is a little tricky: we
 * only invoke the init method if we can't read a persistent value. */
#define POST_INIT_out(record, pr, field, INIT_OK) \
    { \
        GET_RECORD(record, pr, base, iRecord); \
        if (!(iRecord->persist  && \
              read_persistent_variable(iRecord->name, &field))  && \
            iRecord->init != NULL) \
            (void) ACTION_read(record, pr, iRecord, init, field); \
        memcpy(base->WriteData, &field, sizeof(field)); \
        record##_MLST(pr->mlst = field); \
        post_init_record_out( \
            (dbCommon*)pr, (const struct i_record *) iRecord); \
        return INIT_OK; \
    }
/* The MLST field update is only for selected record types. */
#define do_MLST(action) action
#define no_MLST(action)


/* Record initialisation is simply a matter of constructing an instance of
 * the appropriate record type.  This is then followed by record specific
 * extra initialisation. */
#define INIT_RECORD(record, VAL, inOrOut, post_init, INIT_OK) \
    static long init_record_##record(record##Record *pr) \
    { \
        if (init_record_( \
                (dbCommon *) pr, pr->inOrOut.value.instio.string, \
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

static void post_process(
    dbCommon *pr, epicsEnum16 nsta, const struct i_record *iRecord)
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
    static long action##_##record(record##Record *pr) \
    { \
        GET_RECORD(record, pr, base, iRecord); \
        bool ok = ACTION_##action( \
            record, pr, iRecord, action, pr->VAL); \
        post_process((dbCommon *)pr, action##_ALARM, \
            (const struct i_record *) iRecord); \
        return ok ? PROC_OK : ERROR; \
    }

#define read_ALARM      READ_ALARM
#define write_ALARM     WRITE_ALARM




#define DEFINE_DEFAULT_READ(record, VAL, INIT_OK, PROC_OK) \
    INIT_RECORD(record, VAL, inp, inp, INIT_OK) \
    DEFINE_DEFAULT_PROCESS(record, VAL, read, PROC_OK)
#define DEFINE_DEFAULT_WRITE(record, VAL, INIT_OK, PROC_OK) \
    INIT_RECORD(record, VAL, out, out, INIT_OK) \
    DEFINE_DEFAULT_PROCESS(record, VAL, write, PROC_OK)


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

/* For out records define whether the MLST field is defined. */
#define longout_MLST    do_MLST
#define ao_MLST         do_MLST
#define bo_MLST         do_MLST
#define stringout_MLST  no_MLST
#define mbbo_MLST       do_MLST


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

/* Also need dummy special_linconv routines for ai and ao. */
static long linconv_ai(aiRecord *pr, int cmd) { return OK; }
static long linconv_ao(aoRecord *pr, int cmd) { return OK; }




/* Reading a waveform doesn't fit into the fairly uniform pattern established
 * for the other record types. */

/* Routine to validate record type: ensure that we don't mismatch the record
 * declarations in the code and in the database. */
static bool CheckWaveformType(
    waveformRecord *pr, const struct i_waveform_void *iRecord)
{
    epicsEnum16 expected = DBF_NOACCESS;
    switch (iRecord->field_type)
    {
        case waveform_TYPE_void:    break;
        case waveform_TYPE_char:    expected = DBF_CHAR;    break;
        case waveform_TYPE_short:   expected = DBF_SHORT;   break;
        case waveform_TYPE_int:     expected = DBF_LONG;    break;
        case waveform_TYPE_float:   expected = DBF_FLOAT;   break;
        case waveform_TYPE_double:  expected = DBF_DOUBLE;  break;
    }
    bool ok =
        TEST_OK_(pr->ftvl == expected,
            "Array %s.FTVL mismatch %d != %d (%d)\n",
            iRecord->name, pr->ftvl, expected, iRecord->field_type)  &&
        TEST_OK_(pr->nelm <= iRecord->max_length,
            "Array %s too long", iRecord->name);
    if (!ok)
        pr->dpvt = NULL;
    return ok;
}


static bool waveform_action(
    waveformRecord *pr, const struct i_waveform_void *i_waveform,
    bool (*action)(
        void *context, void *array, size_t max_length, size_t *new_length))
{
    size_t nord = pr->nord;
    bool ok = action(i_waveform->context, pr->bptr, pr->nelm, &nord);
    pr->nord = nord;
    return ok;
}

static long init_record_waveform(waveformRecord *pr)
{
    if (init_record_(
            (dbCommon *) pr, pr->inp.value.instio.string, RECORD_TYPE_waveform))
    {
        GET_RECORD(waveform_void, pr, base, iRecord);
        if (!CheckWaveformType(pr, iRecord))
            return ERROR;
        if (iRecord->persist)
        {
            size_t nord;
            pr->udf = !read_persistent_waveform(iRecord->name, pr->bptr, &nord);
            pr->nord = nord;
        }
        if (iRecord->init != NULL  &&  pr->udf)
            pr->udf = !waveform_action(pr, iRecord, iRecord->init);
        post_init_record_out(
            (dbCommon*) pr, (const struct i_record *) iRecord);
        return OK;
    }
    else
        return ERROR;
}

static long process_waveform(waveformRecord *pr)
{
    GET_RECORD(waveform_void, pr, base, i_waveform);
    /* Naughty cast: I want to a reference to size_t, pr->nord is actually an
     * unsigned int.  Force the two to match! */
    bool Ok = waveform_action(pr, i_waveform, i_waveform->process);
    if (i_waveform->persist)
        write_persistent_waveform(i_waveform->name, pr->bptr, pr->nord);
    post_process(
        (dbCommon *) pr, READ_ALARM, (const struct i_record *) i_waveform);
    /* Note, by the way, that the waveform record support carefully ignores
     * my return code! */
    return Ok ? OK : ERROR;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                          Device Driver Definitions                        */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "recordDevice.h"

/* The epicsExportAddress macro used in the definitions below casts a structure
 * pointer via (char*) and thus generates a cast alignment error.  We want to
 * just ignore this here. */
#pragma GCC diagnostic ignored "-Wcast-align"
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
