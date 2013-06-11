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


/* Epics processing return codes. */
#define EPICS_OK        0
#define EPICS_ERROR     1
#define NO_CONVERT      2       // Special code for ai/ao conversion



void CopyEpicsString(const EPICS_STRING in, EPICS_STRING out)
{
    memcpy(out, in, sizeof(EPICS_STRING));
}



/****************************************************************************/
/*                   Core Record Publishing and Lookup                      */
/****************************************************************************/

static struct hash_table *hash_table = NULL;


/* This is the core of the generic EPICS record implementation.  There are
 * essentially three underlying classes of record: IN records, OUT records and
 * WAVEFORM records, each with slightly different support. */
struct epics_record {
    char *key;                      // Name of lookup for record
    enum record_type record_type;
    bool bound;                     // Ensures at most one bound record

    /* The following fields are shared between pairs of record classes. */
    IOSCANPVT ioscanpvt;            // Used for I/O intr enabled records
    bool persist;                   // Set for persistently written data
    epicsAlarmSeverity severity;    // Reported record status
    void *context;                  // Context for all user callbacks

    /* The following fields are record class specific. */
    union {
        // IN record support
        struct {
            bool (*read)(void *context, void *result);
            struct timespec timestamp;  // Timestamp explicitly set
            bool set_time;              // Whether to use timestamp
        } in;
        // OUT record support
        struct {
            bool (*write)(void *context, const void *value);
            bool (*init)(void *context, void *result);
            void *save_value;       // Used to restore after rejected write
        } out;
        // WAVEFORM record support
        struct {
            enum waveform_type field_type;
            size_t max_length;
            void (*process)(void *context, void *array, size_t *length);
            void (*init)(void *context, void *array, size_t *length);
        } waveform;
    };
};


/* Generic argument types. */
DECLARE_IN_ARGS_(in, void);
DECLARE_OUT_ARGS_(out, void);


/* Returns the size of data to be reserved for the record_base::WriteData
 * field.  This is only used for output records. */
static size_t write_data_size(enum record_type record_type)
{
    switch (record_type)
    {
        case RECORD_TYPE_longout:   return sizeof(TYPEOF(longout));
        case RECORD_TYPE_ulongout:  return sizeof(TYPEOF(ulongout));
        case RECORD_TYPE_ao:        return sizeof(TYPEOF(ao));
        case RECORD_TYPE_bo:        return sizeof(TYPEOF(bo));
        case RECORD_TYPE_stringout: return sizeof(TYPEOF(stringout));
        case RECORD_TYPE_mbbo:      return sizeof(TYPEOF(mbbo));
        default: ASSERT_FAIL();
    }
}


/* The types used here must match the types used for record interfacing. */
static enum PERSISTENCE_TYPES record_type_to_persistence(
    enum record_type record_type)
{
    switch (record_type)
    {
        case RECORD_TYPE_longout:   return PERSISTENT_int;
        case RECORD_TYPE_ulongout:  return PERSISTENT_int;
        case RECORD_TYPE_ao:        return PERSISTENT_double;
        case RECORD_TYPE_bo:        return PERSISTENT_bool;
        case RECORD_TYPE_stringout: return PERSISTENT_string;
        case RECORD_TYPE_mbbo:      return PERSISTENT_int;
        default: ASSERT_FAIL();
    }
}

static enum PERSISTENCE_TYPES waveform_type_to_persistence(
    enum waveform_type waveform_type)
{
    switch (waveform_type)
    {
        case waveform_TYPE_char:    return PERSISTENT_char;
        case waveform_TYPE_short:   return PERSISTENT_short;
        case waveform_TYPE_int:     return PERSISTENT_int;
        case waveform_TYPE_float:   return PERSISTENT_float;
        case waveform_TYPE_double:  return PERSISTENT_double;
        default: ASSERT_FAIL();
    }
}


/* Converts record_type into printable name. */
static const char *get_type_name(enum record_type record_type)
{
    static const char *names[] = {
        "longin",       "longin",       "longout",      "longout",
        "ai",           "ao",           "bi",           "bo",
        "stringin",     "stringout",    "mbbi",         "mbbo",
        "waveform" };
    ASSERT_OK(record_type < ARRAY_SIZE(names));
    return names[record_type];
}


/* Construct key by concatenating record_type and name. */
#define BUILD_KEY(key, name, record_type) \
    char key[strlen(name) + 20]; \
    sprintf(key, "%s:%s", get_type_name(record_type), name)


/* For each of the three record classes (IN, OUT, WAVEFORM) we extract the
 * appropriate fields from the given arguments, which are guaranteed to be of
 * the correct type, and perform any extra initialisation. */

static void initialise_in_fields(
    struct epics_record *base, const struct record_args_in *in_args)
{
    base->in.set_time = in_args->set_time;
    base->in.read = in_args->read;
    base->context = in_args->context;
    if (in_args->io_intr)
        scanIoInit(&base->ioscanpvt);
}

static void initialise_out_fields(
    struct epics_record *base, const struct record_args_out *out_args)
{
    base->out.write = out_args->write;
    base->out.init = out_args->init;
    base->out.save_value = malloc(write_data_size(base->record_type));
    base->context = out_args->context;
    base->persist = out_args->persist;
    if (base->persist)
        create_persistent_waveform(base->key,
            record_type_to_persistence(base->record_type), 1);
}

static void initialise_waveform_fields(
    struct epics_record *base, const struct waveform_args_void *waveform_args)
{
    base->waveform.field_type = waveform_args->field_type;
    base->waveform.max_length = waveform_args->max_length;
    base->waveform.process = waveform_args->process;
    base->waveform.init = waveform_args->init;
    base->context = waveform_args->context;
    base->persist = waveform_args->persist;
    if (base->persist)
        create_persistent_waveform(base->key,
            waveform_type_to_persistence(waveform_args->field_type),
            waveform_args->max_length);
    if (waveform_args->io_intr)
        scanIoInit(&base->ioscanpvt);
}


/* Publishes record of given type with given name as specified by record type
 * specific arguments. */
struct epics_record *publish_epics_record(
    enum record_type record_type, const char *name, const void *args)
{
    struct epics_record *base = malloc(sizeof(struct epics_record));

    /* Construct lookup key of form <record-type>:<name>. */
    BUILD_KEY(key, name, record_type);
    base->record_type = record_type;
    base->key = malloc(strlen(key) + 1);
    strcpy(base->key, key);

    base->bound = false;
    base->ioscanpvt = NULL;
    base->persist = false;
    base->severity = epicsSevNone;

    switch (record_type)
    {
        case RECORD_TYPE_longin:    case RECORD_TYPE_ulongin:
        case RECORD_TYPE_ai:        case RECORD_TYPE_bi:
        case RECORD_TYPE_stringin:  case RECORD_TYPE_mbbi:
            initialise_in_fields(base, args);
            break;
        case RECORD_TYPE_longout:   case RECORD_TYPE_ulongout:
        case RECORD_TYPE_ao:        case RECORD_TYPE_bo:
        case RECORD_TYPE_stringout: case RECORD_TYPE_mbbo:
            initialise_out_fields(base, args);
            break;
        case RECORD_TYPE_waveform:
            initialise_waveform_fields(base, args);
            break;
    }

    if (hash_table == NULL)
        hash_table = hash_table_create(false);
    if (!TEST_OK_(hash_table_insert(hash_table, base->key, base) == NULL,
            "Record \"%s\" already exists!", key))
        ASSERT_FAIL();      // Don't allow caller to carry on
    return base;
}


static bool is_in_record(enum record_type record_type)
{
    switch (record_type)
    {
        case RECORD_TYPE_longin:    case RECORD_TYPE_ulongin:
        case RECORD_TYPE_ai:        case RECORD_TYPE_bi:
        case RECORD_TYPE_stringin:  case RECORD_TYPE_mbbi:
            return true;
        default:
            return false;
    }
}


void trigger_record(
    struct epics_record *record, epicsAlarmSeverity severity,
    struct timespec *timestamp)
{
    bool in_record = is_in_record(record->record_type);
    bool wf_record = record->record_type == RECORD_TYPE_waveform;
    ASSERT_OK(in_record  ||  wf_record);

    record->severity = severity;
    if (timestamp)
    {
        ASSERT_OK(in_record  &&  record->in.set_time);
        record->in.timestamp = *timestamp;
    }

    if (record->ioscanpvt)
        scanIoRequest(record->ioscanpvt);
}



/*****************************************************************************/
/*                                                                           */
/*                   Specialised Record Support Methods                      */
/*                                                                           */
/*****************************************************************************/

/* A number of tricksy functions designed to support very simple and uniform
 * access on top of the rather more general framework developed here. */

#define DEFINE_READ_VAR(record) \
    DECLARE_READ_VAR(record) \
    { \
        const TYPEOF(record) *variable = context; \
        *value = *variable; \
        return true; \
    }

#define DEFINE_WRITE_VAR(record) \
    DECLARE_WRITE_VAR(record) \
    { \
        TYPEOF(record) *variable = context; \
        *variable = *value; \
        return true; \
    }

#define DEFINE_READER(record) \
    DECLARE_READER(record) \
    { \
        TYPEOF(record) (*reader)(void) = context; \
        *value = reader(); \
        return true; \
    }

#define DEFINE_WRITER(record) \
    DECLARE_WRITER(record) \
    { \
        void (*writer)(TYPEOF(record)) = context; \
        writer(*value); \
        return true; \
    }

#define DEFINE_WRITER_B(record) \
    DECLARE_WRITER_B(record) \
    { \
        bool (*writer)(TYPEOF(record)) = context; \
        return writer(*value); \
    }

FOR_IN_RECORDS(DEFINE_READ_VAR,)
FOR_OUT_RECORDS(DEFINE_WRITE_VAR,)
FOR_OUT_RECORDS(DEFINE_READ_VAR,)
FOR_IN_RECORDS(DEFINE_READER,)
FOR_OUT_RECORDS(DEFINE_WRITER,)
FOR_OUT_RECORDS(DEFINE_WRITER_B,)

bool publish_trigger_bi(void *context, bool *value)
{
    *value = true;
    return true;
}

bool publish_action_bo(void *context, const bool *value)
{
    void (*action)(void) = context;
    action();
    return true;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Waveform adapters. */

/* These adapters all implement simplified waveform access, reading or writing a
 * fixed length variable, or calling an action (with uncomitted direction) with
 * a fixed length waveform.  In all cases the true length of the waveform is
 * written back as part of the processing.  This turns out to be sensible, as a
 * caput of a partial waveform leaves the rest of the waveform undisturbed, so
 * can sensibly be treated as an action on the full waveform. */

struct waveform_context {
    size_t size;
    size_t length;
    void *context;
};

void *make_waveform_context(size_t size, size_t length, void *context)
{
    struct waveform_context *info = malloc(sizeof(struct waveform_context));
    info->size = size;
    info->length = length;
    info->context = context;
    return info;
}

void publish_waveform_write_var(void *context, void *array, size_t *length)
{
    struct waveform_context *info = context;
    memcpy(info->context, array, info->length * info->size);
    *length = info->length;
}

void publish_waveform_read_var(void *context, void *array, size_t *length)
{
    struct waveform_context *info = context;
    memcpy(array, info->context, info->length * info->size);
    *length = info->length;
}

void publish_waveform_action(void *context, void *array, size_t *length)
{
    struct waveform_context *info = context;
    void (*action)(void *) = info->context;
    action(array);
    *length = info->length;
}


/*****************************************************************************/
/*                                                                           */
/*                   Record Device Support Implementation                    */
/*                                                                           */
/*****************************************************************************/


/* Looks up the record and records it in dpvt if found.  Also take care to
 * ensure that only one EPICS record binds to any one instance. */
static bool init_record_common(
    dbCommon *pr, const char *name, enum record_type record_type)
{
    BUILD_KEY(key, name, record_type);
    struct epics_record *base = hash_table_lookup(hash_table, key);
    return
        TEST_NULL_(base, "No record found for %s", key)  &&
        TEST_OK_(!base->bound, "Duplicate binding for %s", key)  &&
        DO_(base->bound = true; pr->dpvt = base);
}


/* Common I/O Intr scanning support. */
static long get_ioint_common(int cmd, dbCommon *pr, IOSCANPVT *ioscanpvt)
{
    struct epics_record *base = pr->dpvt;
    if (base == NULL)
        return EPICS_ERROR;
    else
    {
        *ioscanpvt = base->ioscanpvt;
        return EPICS_OK;
    }
}


/* The following two macros are designed to adapt between the external
 * representation of PV data and the internal representation when they're not
 * pointer assignment compatible.  Unfortunately this is a very hacky solution
 * (for instance, call is forced to return a bool), and in fact the only record
 * type we need to adapt is bi/bo to bool. */
#define SIMPLE_ADAPTER(call, type, value, args...) \
    call(args, *(type*[]) { &value })

#define COPY_ADAPTER(call, type, value, args...) \
    ( { \
        type __value = value; \
        bool __ok = call(args, &__value); \
        value = __value; \
        __ok; \
    } )


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Input record processing. */

static bool init_in_record(dbCommon *pr)
{
    struct epics_record *base = pr->dpvt;
    return TEST_OK_(
        base->in.set_time == (pr->tse == epicsTimeEventDeviceTime),
        "Inconsistent timestamping (%d/%d) for %s\n",
            base->in.set_time, pr->tse, base->key);
}

static bool process_in_record(dbCommon *pr, void *result)
{
    struct epics_record *base = pr->dpvt;
    if (base == NULL)
        return false;

    bool ok = base->in.read(base->context, result);

    recGblSetSevr(pr, READ_ALARM, base->severity);
    if (base->in.set_time)
        epicsTimeFromTimespec(&pr->time, &base->in.timestamp);
    pr->udf = !ok;
    return ok;
}

#define DEFINE_PROCESS_IN(record, PROC_OK, VAL, ADAPTER) \
    static long read_##record(record##Record *pr) \
    { \
        bool ok = ADAPTER(process_in_record, \
            TYPEOF(record), pr->VAL, (dbCommon *) pr); \
        return ok ? PROC_OK : EPICS_ERROR; \
    }

#define DEFINE_INIT_IN(record) \
    static long init_record_##record(record##Record *pr) \
    { \
        bool ok = \
            init_record_common((dbCommon *) pr, \
                pr->inp.value.instio.string, RECORD_TYPE_##record)  && \
            init_in_record((dbCommon *) pr); \
        return ok ? EPICS_OK : EPICS_ERROR; \
    }

#define DEFINE_DEFAULT_IN(record, VAL, PROC_OK, ADAPTER) \
    DEFINE_INIT_IN(record) \
    DEFINE_PROCESS_IN(record, PROC_OK, VAL, ADAPTER)



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Output record processing. */

/* Perform a simulation of record processing event, including a synthetic
 * initial timestamp.  This is useful after initialising an output record with
 * driver provided values. */
static void post_init_process(dbCommon *pr)
{
    recGblResetAlarms(pr);
    struct timespec timestamp;
    clock_gettime(CLOCK_REALTIME, &timestamp);
    epicsTimeFromTimespec(&pr->time, &timestamp);
}


/* Common out record initialisation: we look in a couple of places for an
 * initial value.  If there's a persistent value stored used that, otherwise
 * call init if defined.  This is saved so we can restore rejected writes. */
static bool init_out_record(dbCommon *pr, size_t value_size, void *result)
{
    struct epics_record *base = pr->dpvt;
    bool read_ok =
        (base->persist   &&  read_persistent_variable(base->key, result))  ||
        (base->out.init  &&  base->out.init(base->context, result));
    memcpy(base->out.save_value, result, value_size);

    if (read_ok)
        post_init_process(pr);

    return true;
}


/* Common out record processing.  If writing fails then restore saved value,
 * otherwise maintain saved and persistent settings. */
static bool process_out_record(dbCommon *pr, size_t value_size, void *result)
{
    struct epics_record *base = pr->dpvt;
    if (base == NULL)
        return false;

    if (base->out.write(base->context, result))
    {
        /* On successful update take a record (in case we have to revert) and
         * update the persistent record. */
        memcpy(base->out.save_value, result, value_size);
        if (base->persist)
            write_persistent_variable(base->key, result);
        return true;
    }
    else
    {
        /* On a rejected update restore the value from backing store. */
        memcpy(result, base->out.save_value, value_size);
        return false;
    }
}


#define DEFINE_PROCESS_OUT(record, VAL, ADAPTER) \
    static long write_##record(record##Record *pr) \
    { \
        bool ok = ADAPTER(process_out_record, \
            TYPEOF(record), pr->VAL, \
            (dbCommon *) pr, sizeof(TYPEOF(record))); \
        return ok ? EPICS_OK : EPICS_ERROR; \
    }

#define DEFINE_INIT_OUT(record, INIT_OK, VAL, ADAPTER, MLST) \
    static long init_record_##record(record##Record *pr) \
    { \
        bool ok = init_record_common((dbCommon *) pr, \
            pr->out.value.instio.string, RECORD_TYPE_##record); \
        if (ok) \
        { \
            ADAPTER(init_out_record, \
                TYPEOF(record), pr->VAL, \
                (dbCommon *) pr, sizeof(TYPEOF(record))); \
            MLST(pr->mlst = pr->VAL); \
        } \
        return ok ? INIT_OK : EPICS_ERROR; \
    }


#define do_MLST(arg)    arg
#define no_MLST(arg)

#define DEFINE_DEFAULT_OUT(record, VAL, INIT_OK, ADAPTER, MLST) \
    DEFINE_INIT_OUT(record, INIT_OK, VAL, ADAPTER, MLST) \
    DEFINE_PROCESS_OUT(record, VAL, ADAPTER)



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                    IN/OUT Device Driver Implementations                   */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#define DEFINE_IN_OUT(in, out, VAL, CONVERT, ADAPTER, MLST) \
    DEFINE_DEFAULT_IN(in,   VAL, CONVERT,  ADAPTER) \
    DEFINE_DEFAULT_OUT(out, VAL, CONVERT,  ADAPTER, MLST)

/* Mostly we can use simple boilerplate for the process routines. */
DEFINE_IN_OUT(longin,   longout,   val,  EPICS_OK,   SIMPLE_ADAPTER, do_MLST)
DEFINE_IN_OUT(ai,       ao,        val,  NO_CONVERT, SIMPLE_ADAPTER, do_MLST)
DEFINE_IN_OUT(bi,       bo,        rval, EPICS_OK,   COPY_ADAPTER,   do_MLST)
DEFINE_IN_OUT(stringin, stringout, val,  EPICS_OK,   SIMPLE_ADAPTER, no_MLST)
DEFINE_IN_OUT(mbbi,     mbbo,      rval, EPICS_OK,   SIMPLE_ADAPTER, do_MLST)

/* Also need dummy special_linconv routines for ai and ao. */
static long linconv_ai(aiRecord *pr, int cmd) { return EPICS_OK; }
static long linconv_ao(aoRecord *pr, int cmd) { return EPICS_OK; }




/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                       Waveform Record Implementation                      */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* Reading a waveform doesn't fit into the fairly uniform pattern established
 * for the other record types. */


/* Routine to validate record type: ensure that we don't mismatch the record
 * declarations in the code and in the database. */
static bool check_waveform_type(waveformRecord *pr, struct epics_record *base)
{
    epicsEnum16 expected = DBF_NOACCESS;
    switch (base->waveform.field_type)
    {
        case waveform_TYPE_void:    break;
        case waveform_TYPE_char:    expected = DBF_CHAR;    break;
        case waveform_TYPE_short:   expected = DBF_SHORT;   break;
        case waveform_TYPE_int:     expected = DBF_LONG;    break;
        case waveform_TYPE_float:   expected = DBF_FLOAT;   break;
        case waveform_TYPE_double:  expected = DBF_DOUBLE;  break;
    }
    return
        TEST_OK_(pr->ftvl == expected,
            "Array %s.FTVL mismatch %d != %d (%d)",
            base->key, pr->ftvl, expected, base->waveform.field_type)  &&
        TEST_OK_(pr->nelm <= base->waveform.max_length,
            "Array %s too long", base->key);
}


/* After binding to the record base try to load from persistence storage; if
 * that fails, try for an (optional) init method. */
static long init_record_waveform(waveformRecord *pr)
{
    bool ok = init_record_common(
        (dbCommon *) pr, pr->inp.value.instio.string, RECORD_TYPE_waveform)  &&
        check_waveform_type(pr, pr->dpvt);
    if (!ok)
    {
        pr->dpvt = NULL;
        return EPICS_ERROR;
    }

    struct epics_record *base = pr->dpvt;
    size_t nord = 0;
    bool read_ok =
        base->persist  &&
        read_persistent_waveform(base->key, pr->bptr, &nord);
    if (!read_ok  &&  base->waveform.init)
    {
        nord = pr->nelm;
        base->waveform.init(base->context, pr->bptr, &nord);
        read_ok = true;
    }
    pr->nord = nord;
    pr->udf = !read_ok;

    post_init_process((dbCommon *) pr);
    return EPICS_OK;
}


static long process_waveform(waveformRecord *pr)
{
    struct epics_record *base = pr->dpvt;
    if (base == NULL)
        return EPICS_ERROR;

    size_t nord = pr->nord;
    base->waveform.process(base->context, pr->bptr, &nord);
    pr->nord = nord;
    if (base->persist)
        write_persistent_waveform(base->key, pr->bptr, pr->nord);

    recGblSetSevr(pr, READ_ALARM, base->severity);

    /* Note that waveform record support ignores the return code! */
    return EPICS_OK;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                          Device Driver Definitions                        */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#define DEFINE_DEVICE(record, length, args...) \
    static struct record##Device record##Generic = \
    { \
        length, \
        NULL, \
        NULL, \
        init_record_##record, \
        get_ioint_common, \
        args \
    }; \
    epicsExportAddress(dset, record##Generic)

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
