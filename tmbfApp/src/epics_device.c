#define USE_GENERIC

#ifndef USE_GENERIC

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


//#define PRINTF(args...) printf(args)
#define PRINTF(args...)




/****************************************************************************/
/*                       I_RECORD publish and lookup                        */
/****************************************************************************/

#define TABLESIZE   256

static void * hash_table = NULL;


bool PublishEpicsData(const I_RECORD* publish_data[])
{
    if (hash_table == NULL)
        gphInitPvt(&hash_table, TABLESIZE);
    
    for (const I_RECORD **data = publish_data; *data != NULL; data ++)
    {
        GPHENTRY * entry = gphAdd(hash_table, (*data)->name, NULL);
        if (entry == NULL)
        {
            printf("Failed to add entry \"%s\"\n", (*data)->name);
            return false;
        }

        entry->userPvt = (void *) *data;
    }
    return true;
}


/* Looks up the given record by name.  If it is found and matches the record
 * type then a record is returned, otherwise NULL is returned. */
static const I_RECORD * LookupRecord(
    const char * Name, RECORD_TYPE record_type)
{
    GPHENTRY * entry = gphFind(hash_table, Name, NULL);
    if (entry == NULL)
    {
        printf("No handler found for \"%s\"\n", Name);
        return NULL;
    }
    else
    {
        const I_RECORD * iRecord = (const I_RECORD *) entry->userPvt;
        if (iRecord->record_type == record_type)
            return iRecord;
        else
        {
            printf("Record \"%s\" of type %d should be %d\n",
                Name, iRecord->record_type, record_type);
            return NULL;
        }
    }
}



#if 0

/*****************************************************************************/
/*                                                                           */
/*                        Common Record Implementation                       */
/*                                                                           */
/*****************************************************************************/


/* All record implementations use this class. */

class RECORD_BASE : public I_INTR
{
public:
    RECORD_BASE(I_RECORD &iRecord) :
        iRecord(iRecord)
    {
        ioscanpvt = NULL;
        IOSCANPVT newIoscanpvt;
        if (iRecord.EnableIoIntr(*this))
        {
            scanIoInit(&newIoscanpvt);
            ioscanpvt = newIoscanpvt;
        }
    }
    
    /* Callback routine called (possibly in signal handler context, or in an
     * arbitrary thread) to notify that I/O Intr processing should occur. */
    bool IoIntr()
    {
        if (ioscanpvt == NULL)
            return false;
        else
        {
            scanIoRequest(ioscanpvt);
            return true;
        }
    }

    /* Used by record implementation to implement get_ioint functionality. */
    void GetIoInt(IOSCANPVT *pIoscanpvt)
    {
        *pIoscanpvt = ioscanpvt;
    }

    inline I_RECORD * GetRecord() { return & iRecord; }

    
private:
    I_RECORD & iRecord;
    IOSCANPVT ioscanpvt;
};



template<class T>
    bool I_WRITER<T>::_do_init(T &value)
{
    bool ok = init(value);
    _good_value = value;
    return ok;
}

template<class T>
    bool I_WRITER<T>::_do_write(T &value)
{
    bool ok = write(value);
    if (ok)
        _good_value = value;
    else
        value = _good_value;
    return ok;
}

template<>
bool I_WRITER<EPICS_STRING>::_do_init(EPICS_STRING &value)
{
    bool ok = init(value);
    CopyEpicsString(value, _good_value);
    return ok;
}

template<>
bool I_WRITER<EPICS_STRING>::_do_write(EPICS_STRING &value)
{
    bool ok = write(value);
    if (ok)
        CopyEpicsString(value, _good_value);
    else
        CopyEpicsString(_good_value, value);
    return ok;
}


template class I_WRITER<int>;
template class I_WRITER<bool>;
template class I_WRITER<EPICS_STRING>;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                               I_WAVEFORM                                  */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* Simple helper class for routine I_waveform implementations: implements
 * array field type validation. */


I_WAVEFORM::I_WAVEFORM(epicsEnum16 Type) :
    Type(Type)
{
}

bool I_WAVEFORM::BindRecord(dbCommon * p)
{
    waveformRecord * pr = (waveformRecord *) p;
    /* Check that the types match. */
    if (pr->ftvl == Type)
        return true;
    else
    {
        printf("Array FTVL mismatch %d != %d\n", Type, pr->ftvl);
        return false;
    }
}

#endif



/* Common information about records: this is written to the dpvt field. */
typedef struct
{
    void (*GetIoInt)();
    bool (*BindRecord)();
    const I_RECORD * iRecord;
} RECORD_BASE;


static RECORD_BASE * create_RECORD_BASE(const I_RECORD *iRecord)
{
    RECORD_BASE * base = malloc(sizeof(RECORD_BASE));
    if (base != NULL)
    {
        base->iRecord = iRecord;
    }
    return base;
}


static epicsAlarmSeverity get_alarm_status(I_RECORD *iRecord)
{
    if (iRecord->get_alarm_status)
        return iRecord->get_alarm_status();
    else
        return epicsSevNone;
}

static bool get_timestamp(I_RECORD *iRecord, struct timespec *time)
{
    if (iRecord->get_timestamp)
        return iRecord->get_timestamp(time);
    else
        return false;
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                        Common Device Driver Routines                      */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* Common I/O Intr scanning support: uses the fact that pr->dpvt always
 * contains the appropriate GetIoInt implementation. */

static long get_ioint_(int cmd, dbCommon *pr, IOSCANPVT *pIoscanpvt) 
{ 
    RECORD_BASE * base = (RECORD_BASE *) pr->dpvt; 
    if (base == NULL) 
        return ERROR; 
    else 
    {       
        base->GetIoInt(pIoscanpvt); 
        return OK; 
    } 
}



/* Common record initialisation.  Performs the appropriate record
 * initialisation once the record implementation has been found. */

static bool init_record_(
    RECORD_TYPE record_type, const char * Name, dbCommon *pr)
{
    const I_RECORD * iRecord = LookupRecord(Name, record_type);
    /* If we successfully found a record then try binding to it.  If this
     * fails then record initialisation fails, otherwise we're done. */
    if (iRecord == NULL)
    {
        printf("Libera record %d:%s not found\n", record_type, Name);
        return false;
    }
//     else if (!iRecord->BindRecord(pr))
//     {
//         printf("Error binding libera record %d:%s\n", record_type, Name);
//         return false;
//     }
    else
    {
        pr->dpvt = create_RECORD_BASE(iRecord);
        return true;
    }
}



static void SetTimestamp(dbCommon *pr, struct timespec *Timestamp)
{
    /* Convert the standard Unix timespec value into the EPICS epoch
     * timestamp (subtract 20 years). */
    epicsTimeFromTimespec(&pr->time, Timestamp);
}


/* Record initialisation post processed: ensures that the EPICS data
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



/* Common record post-processing.  Updates the alarm state as appropriate,
 * and checks if the timestamp should be written here. */

static void post_process(dbCommon *pr, epicsEnum16 nsta, I_RECORD *iRecord)
{
    (void) recGblSetSevr(pr, nsta, get_alarm_status(iRecord));
    struct timespec Timestamp;
    if (!get_timestamp(iRecord, &Timestamp))
        SetTimestamp(pr, &Timestamp);
}



/*****************************************************************************/
/*                                                                           */
/*                    Boilerplate generation support.                        */
/*                                                                           */
/*****************************************************************************/


/* Reads a value from the appropriate record interface.  An intermediate
 * value is used so that the record interface type doesn't need to precisely
 * match the data type stored in the EPICS record. */
#define ACTION_ADAPTER(record, action, field) \
    ( { \
        TYPEOF(record) Value = field; \
        bool Ok = action(Value); \
        field = Value; \
        Ok; \
    } )

#define ACTION_DIRECT(record, action, field) \
    action(field)

/* This macro expands to either ACTION_DIRECT or ACTION_ADAPTER, depending on
 * whether an intermediate temporary value is needed. */
#define ACTION_VALUE(record, action, field) \
    ACTION_##record(record, action, field)



/* For inp records there is no special post initialisation processing. */
#define POST_INIT_inp(record, pr)   return OK;

/* For out records we need to read the current underlying device value as
 * part of initialisation.  The precise processing performed depends on the
 * record type: this is hooked in by defining POST_INIT_##record. */
#define POST_INIT_out(record, pr) \
    { \
        GET_RECORD(record, pr, iRecord); \
        pr->udf = ! ACTION_VALUE(record, iRecord->_do_init, pr->VAL(record)); \
        post_init_record_out((dbCommon*)pr, iRecord); \
        return OK; \
    }



/* Record initialisation is simply a matter of constructing an instance of
 * the appropriate record type. */
#define INIT_RECORD(record, inOrOut, post_init) \
    static long init_record_##record(record##Record *pr) \
    { \
        const char * Name = pr->inOrOut.value.constantStr; \
        if (init_record_(RECORD_TYPE_##record, Name, (dbCommon *) pr)) \
            POST_INIT_##post_init(record, pr) \
        else \
            return ERROR; \
    }


/* Helper code for extracting the appropriate I_record from the record. */
#define GET_RECORD(record, pr, var) \
    RECORD_BASE * base = (RECORD_BASE *) pr->dpvt; \
    if (base == NULL) \
        return ERROR; \
    I_##record * var = (I_##record *) base->iRecord

#define SET_ALARM(pr, ACTION, iRecord) \
    (void) recGblSetSevr(pr, ACTION##_ALARM, get_alarm_status(iRecord))



/* Standard boiler-plate default record processing action.  The val field is
 * either read or written and the alarm state is set by interrogating the
 * record interface.  This processing is adequate for most record types. */
#define DEFINE_DEFAULT_PROCESS(record, action, method, ACTION) \
    static long action##_##record(record##Record * pr) \
    { \
        GET_RECORD(record, pr, iRecord); \
        bool Ok = ACTION_VALUE(record, iRecord->method, pr->VAL(record)); \
        post_process((dbCommon *)pr, ACTION##_ALARM, (I_RECORD *)iRecord); \
        return Ok ? OK : ERROR; \
    }

#define DEFINE_DEFAULT_READ(record) \
    INIT_RECORD(record, inp, inp) \
    DEFINE_DEFAULT_PROCESS(record, read,  read, READ) 
#define DEFINE_DEFAULT_WRITE(record) \
    INIT_RECORD(record, out, out) \
    DEFINE_DEFAULT_PROCESS(record, write, _do_write, WRITE) 


#define DEFINE_DEVICE(record, length, args...) \
    record##Device record##Libera = \
    { \
        length, \
        NULL, \
        NULL, \
        init_record_##record, \
        get_ioint_, \
        args \
    }; \
    epicsExportAddress(dset, record##Libera)

/* Redefine epicsExportAddress to avoid strict aliasing warnings. */
#undef epicsExportAddress
#define epicsExportAddress(typ,obj) \
    epicsShareExtern typeof(obj) *EPICS_EXPORT_POBJ(typ, obj); \
    epicsShareDef typeof(obj) *EPICS_EXPORT_POBJ(typ, obj) = &obj




/*****************************************************************************/
/*                                                                           */
/*                        Device Driver Implementations                      */
/*                                                                           */
/*****************************************************************************/


/* For some types we read and write the VAL field, for others the RVAL field.
 * Here we define which. */
#define longin_VAL      val
#define longout_VAL     val
#define ai_VAL          rval
#define ao_VAL          rval
#define bi_VAL          rval
#define bo_VAL          rval
#define stringin_VAL    val
#define stringout_VAL   val
#define mbbi_VAL        rval
#define mbbo_VAL        rval

#define VAL(record)     record##_VAL

/* Type adapters.  Some types need to be read directly, others need to be
 * read via the read adapter. */

#define ACTION_longin     ACTION_DIRECT
#define ACTION_longout    ACTION_DIRECT
#define ACTION_ai         ACTION_DIRECT
#define ACTION_ao         ACTION_DIRECT
#define ACTION_bi         ACTION_ADAPTER
#define ACTION_bo         ACTION_ADAPTER
#define ACTION_stringin   ACTION_DIRECT
#define ACTION_stringout  ACTION_DIRECT
#define ACTION_mbbi       ACTION_ADAPTER
#define ACTION_mbbo       ACTION_ADAPTER



/* Mostly we can use simple boilerplate for the process routines. */
DEFINE_DEFAULT_READ (longin)
#if 0
DEFINE_DEFAULT_WRITE(longout)
DEFINE_DEFAULT_READ (ai)
DEFINE_DEFAULT_WRITE(ao)
DEFINE_DEFAULT_READ (bi)
DEFINE_DEFAULT_WRITE(bo)
DEFINE_DEFAULT_READ (stringin)
DEFINE_DEFAULT_WRITE(stringout)
DEFINE_DEFAULT_READ (mbbi)
DEFINE_DEFAULT_WRITE(mbbo)


/* Reading a waveform doesn't fit into the fairly uniform pattern established
 * for the other record types. */

#define POST_INIT_waveform(record, pr) \
    { \
        GET_RECORD(record, pr, iRecord); \
        pr->udf = iRecord->init(pr->bptr, *CAST(size_t*, &pr->nord)); \
        post_init_record_out((dbCommon*)pr, iRecord); \
        return OK; \
    }

INIT_RECORD(waveform, inp, waveform)

static long process_waveform(waveformRecord * pr)
{
    GET_RECORD(waveform, pr, i_waveform);
    /* Naughty cast: I want to a reference to size_t, pr->nord is actually an
     * unsigned int.  Force the two to match! */
    bool Ok = i_waveform->process(
        pr->bptr, pr->nelm, *CAST(size_t*, &pr->nord));
    post_process((dbCommon *)pr, READ_ALARM, i_waveform);
    /* Note, by the way, that the waveform record support carefully ignores
     * my return code! */
    return Ok ? OK : ERROR;
}


/* Also need dummy special_linconv routines for ai and ao. */

static long linconv_ai(aiRecord *, int) { return OK; }
static long linconv_ao(aoRecord *, int) { return OK; }


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
#endif


#else


//============================================================================


#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "GenericDevice.h"

#include "epics_device.h"




#define GENERIC_REGISTER(record, action) \
    static void Publish_##record(const I_##record * Record) \
    { \
        GenericRegister((char *) Record->name, \
            generic_##action##_##record, (void*) Record); \
    }


#define DEFINE_PUBLISH_WRITER(record) \
    static void generic_write_##record(Variant *v) \
    { \
        I_##record * Record = (I_##record *) v->usr; \
        TYPEOF(record) value = ((TYPEOF(record) *) v->buffer)[0]; \
        Record->write(value); \
    } \
    GENERIC_REGISTER(record, write)

#define DEFINE_PUBLISH_READER(record) \
    static void generic_read_##record(Variant *v) \
    { \
        I_##record * Record = (I_##record *) v->usr; \
        TYPEOF(record) value; \
        if (Record->read(&value)) \
            ((TYPEOF(record) *) v->buffer)[0] = value; \
    } \
    GENERIC_REGISTER(record, read)


DEFINE_PUBLISH_WRITER(mbbo)
DEFINE_PUBLISH_WRITER(ao)
DEFINE_PUBLISH_READER(ai)



static void generic_write_longout(Variant *v)
{
    I_longout * Record = (I_longout *) v->usr;
    int value = (int) v->buffer[0];
    Record->write(value);
}
GENERIC_REGISTER(longout, write)

static void generic_read_longin(Variant *v)
{
    I_longin * Record = (I_longin *) v->usr;
    int value;
    if (Record->read(&value))
        v->buffer[0] = value;
}
GENERIC_REGISTER(longin, read)


static void generic_process_waveform(Variant *v)
{
    I_waveform * Record = (I_waveform *) v->usr;
    size_t new_length;
    Record->process(v->buffer, v->length, &new_length);
}
GENERIC_REGISTER(waveform, process)


#define DO_PUBLISH_CASE(record, data) \
    case RECORD_TYPE_##record: \
        Publish_##record((I_##record*) *data); \
        break

bool PublishEpicsData(const I_RECORD* publish_data[])
{
    for (const I_RECORD **data = publish_data; *data != NULL; data ++)
    {
        switch ((*data)->record_type)
        {
            DO_PUBLISH_CASE(longin, data);
            DO_PUBLISH_CASE(longout, data);
            DO_PUBLISH_CASE(ai, data);
            DO_PUBLISH_CASE(ao, data);
//             DO_PUBLISH_CASE(bi, data);
//             DO_PUBLISH_CASE(bo, data);
//             DO_PUBLISH_CASE(stringin, data);
//             DO_PUBLISH_CASE(stringout, data);
//             DO_PUBLISH_CASE(mbbi, data);
            DO_PUBLISH_CASE(mbbo, data);
            DO_PUBLISH_CASE(waveform, data);
            default:
                printf("Invalid record type: %d\n", (*data)->record_type);
                return false;
        }
    }
    return true;
}

#endif
