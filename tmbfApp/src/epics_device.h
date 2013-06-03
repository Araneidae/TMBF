/* Common interface to epics records. */

#include <alarm.h>



/*****************************************************************************/
/*                                                                           */
/*                       EPICS_STRING type definition                        */
/*                                                                           */
/*****************************************************************************/


/* Epics strings are rather limited: a massive 39 characters are available! */
typedef char EPICS_STRING[40];

void CopyEpicsString(const EPICS_STRING in, EPICS_STRING *out);


/*****************************************************************************/
/*                                                                           */
/*                        Define EPICS Interfaces                            */
/*                                                                           */
/*****************************************************************************/


/* The following list of enumerations records the possible record types
 * supported by this library. */
enum record_type
{
    RECORD_TYPE_longin,
    RECORD_TYPE_longout,
    RECORD_TYPE_ai,
    RECORD_TYPE_ao,
    RECORD_TYPE_bi,
    RECORD_TYPE_bo,
    RECORD_TYPE_stringin,
    RECORD_TYPE_stringout,
    RECORD_TYPE_mbbi,
    RECORD_TYPE_mbbo,
    RECORD_TYPE_waveform
};


/* Helper macros for determining the underlying type for each supported
 * record type. */
#define TYPEOF(record)   TYPEOF_##record

#define TYPEOF_longin    int
#define TYPEOF_longout   int
#define TYPEOF_ai        double
#define TYPEOF_ao        double
#define TYPEOF_bi        bool
#define TYPEOF_bo        bool
#define TYPEOF_stringin  EPICS_STRING
#define TYPEOF_stringout EPICS_STRING
#define TYPEOF_mbbi      unsigned int
#define TYPEOF_mbbo      unsigned int


/* The i_<record> structures are used to define the complete interface to a
 * record. */


/* This is as close to subclassing as we can come.  Note that name needs to
 * go at the end of the list of fields so that the PUBLISH macros will work
 * properly. */
#define I_COMMON_FIELDS \
    enum record_type record_type; \
    void *context; \
    bool io_intr; \
    bool (*get_timestamp)(void *context, struct timespec *time); \
    epicsAlarmSeverity (*get_alarm_status)(void *context); \
    const char *name

/* The base type common to all records. */
struct i_record {
    I_COMMON_FIELDS;
};


#define DECLARE_I_READER(record) \
    struct i_##record { \
        I_COMMON_FIELDS; \
        bool (*read)(void *context, TYPEOF(record) *result); \
    }

#define DECLARE_I_WRITER(record) \
    struct i_##record { \
        I_COMMON_FIELDS; \
        bool (*write)(void *context, TYPEOF(record) value); \
        bool (*init)(void *context, TYPEOF(record) *initial_value); \
        bool persist; \
    }

DECLARE_I_READER(longin);
DECLARE_I_WRITER(longout);
DECLARE_I_READER(ai);
DECLARE_I_WRITER(ao);
DECLARE_I_READER(bi);
DECLARE_I_WRITER(bo);
DECLARE_I_READER(stringin);
DECLARE_I_WRITER(stringout);
DECLARE_I_READER(mbbi);
DECLARE_I_WRITER(mbbo);


/* The interface to waveform records is a little more involved, as a waveform
 * may be used with a variety of different data types, and the same record
 * type supports both reading and writing.
 *     Although the underlying mechanism works with void pointers, we provide
 * interface declarations for all useful types so that waveform declarations
 * can be type checked. */

enum waveform_type {
    waveform_TYPE_void,
    waveform_TYPE_char,
    waveform_TYPE_short,
    waveform_TYPE_int,
    waveform_TYPE_float,
    waveform_TYPE_double
};

#define DECLARE_I_WAVEFORM(type) \
    struct i_waveform_##type { \
        I_COMMON_FIELDS; \
        enum waveform_type field_type; \
        size_t max_length; \
        bool (*process)( \
            void *context, \
            type *array, size_t max_length, size_t *new_length); \
        bool (*init)( \
            void *context, \
            type *array, size_t max_length, size_t *new_length); \
        bool persist; \
    }

DECLARE_I_WAVEFORM(void);
DECLARE_I_WAVEFORM(char);
DECLARE_I_WAVEFORM(short);
DECLARE_I_WAVEFORM(int);
DECLARE_I_WAVEFORM(float);
DECLARE_I_WAVEFORM(double);



/*****************************************************************************/
/*                                                                           */
/*                        Public EPICS Interfaces                            */
/*                                                                           */
/*****************************************************************************/

/* The private struct record_base is used as an interface to the record. */
struct record_base;

/* Calling this will mark the given record as signalled.  If it is configured
 * for I/O Intr processing then an I/O Intr event will be generated. */
bool SignalRecord(struct record_base *Record);

struct record_base *LookupRecord(const char *name);



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                               Helper Macros                               */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* Macros for generating simple access wrappers: these convert common PV
 * patterns into the format expected by the PUBLISH macros. */

/* This converts function  of the form
 *      void process_waveform(type wf[])
 * into a waveform process method. */
#define WRAP_SIMPLE_WAVEFORM(type, true_length, process_waveform) \
    static bool wrap_##process_waveform( \
        void *context, \
        type *array, size_t max_length, size_t *new_length) \
    { \
        if (max_length == true_length) \
        { \
            process_waveform(array); \
            *new_length = true_length; \
            return true; \
        } \
        else \
        { \
            printf("Invalid call to "#process_waveform": %d != %d\n", \
                max_length, true_length); \
            return false; \
        } \
    }

#define WRAP_VARIABLE_READ_WAVEFORM(type, true_length, waveform) \
    static void read_##waveform(type *array) \
    { \
        COMPILE_ASSERT(sizeof(array[0]) == sizeof(type)); \
        memcpy(array, waveform, sizeof(type) *(true_length)); \
    } \
    WRAP_SIMPLE_WAVEFORM(type, true_length, read_##waveform)

#define WRAP_VARIABLE_WRITE_WAVEFORM(type, true_length, waveform) \
    static void write_##waveform(type *array) \
    { \
        COMPILE_ASSERT(sizeof(array[0]) == sizeof(type)); \
        memcpy(waveform, array, sizeof(type) *(true_length)); \
    } \
    WRAP_SIMPLE_WAVEFORM(type, true_length, write_##waveform)


/* Converts a function for reading a value of the form
 *      type read()
 * into a record processing method. */
#define WRAP_SIMPLE_READ(record, read) \
    static bool wrap_##read(void *context, TYPEOF(record) *result) \
    { \
        *result = read(); \
        return true; \
    }

#define WRAP_SIMPLE_WRITE(record, write) \
    static bool wrap_##write(void *context, TYPEOF(record) value) \
    { \
        write(value); \
        return true; \
    }

#define WRAP_VARIABLE_READ(record, variable) \
    static bool read_##variable(void *context, TYPEOF(record) *result) \
    { \
        *result = variable; \
        return true; \
    }

#define WRAP_VARIABLE_WRITE(record, variable) \
    static bool write_##variable(void *context, TYPEOF(record) value) \
    { \
        variable = value; \
        return true; \
    } \
    static bool init_##variable(void *context, TYPEOF(record) *result) \
    { \
        *result = variable; \
        return true; \
    }

#define WRAP_METHOD(action) \
    static bool wrap_##action(void *context, bool ignored) \
    { \
        action(); \
        return true; \
    }


/* Publication macros to include wrapping and publishing in a single act.  Of
 * course only the wrapped action is supported by the published record -- if
 * more functionality is required then the record should be published as
 * normal. */
#define PUBLISH_SIMPLE_WAVEFORM( \
        type, name, true_length, process_waveform, extra...) \
    WRAP_SIMPLE_WAVEFORM(type, true_length, process_waveform) \
    PUBLISH_WAVEFORM(type, true_length, name, wrap_##process_waveform, ##extra)
#define PUBLISH_SIMPLE_WAVEFORM_INIT( \
        type, name, true_length, write_waveform, read_waveform, extra...) \
    WRAP_SIMPLE_WAVEFORM(type, true_length, write_waveform) \
    WRAP_SIMPLE_WAVEFORM(type, true_length, read_waveform) \
    PUBLISH_WAVEFORM( \
        type, true_length, name, \
        wrap_##write_waveform, wrap_##read_waveform, ##extra)
#define PUBLISH_READ_WAVEFORM(type, name, true_length, waveform) \
    WRAP_VARIABLE_READ_WAVEFORM(type, true_length, waveform) \
    PUBLISH_WAVEFORM(type, true_length, name, wrap_read_##waveform)
#define PUBLISH_WRITE_WAVEFORM(type, name, true_length, waveform, extra...) \
    WRAP_VARIABLE_WRITE_WAVEFORM(type, true_length, waveform) \
    WRAP_VARIABLE_READ_WAVEFORM(type, true_length, waveform) \
    PUBLISH_WAVEFORM( \
        type, true_length, name, \
        wrap_write_##waveform, wrap_read_##waveform, ##extra)
#define PUBLISH_SIMPLE_READ(record, name, read) \
    WRAP_SIMPLE_READ(record, read) \
    PUBLISH(record, name, wrap_##read)
#define PUBLISH_SIMPLE_WRITE(record, name, write, extra...) \
    WRAP_SIMPLE_WRITE(record, write) \
    PUBLISH(record, name, wrap_##write, ##extra)
#define PUBLISH_SIMPLE_WRITE_INIT(record, name, write, initialiser, extra...) \
    WRAP_SIMPLE_WRITE(record, write) \
    WRAP_SIMPLE_READ(record, initialiser) \
    PUBLISH(record, name, wrap_##write, .init = wrap_##initialiser, ##extra)
#define PUBLISH_VARIABLE_READ(record, name, variable) \
    WRAP_VARIABLE_READ(record, variable) \
    PUBLISH(record, name, read_##variable)
#define PUBLISH_VARIABLE_WRITE(record, name, variable, extra...) \
    WRAP_VARIABLE_WRITE(record, variable) \
    PUBLISH(record, name, write_##variable, .init = init_##variable, ##extra)

#define PUBLISH_METHOD(name, action) \
    WRAP_METHOD(action) \
    PUBLISH(bo, name, wrap_##action)


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                            Publication Macros                             */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* The following macros define the public mechanism for declaring EPICS
 * records to the generic device.  Scalar and waveform records are declared
 * differently, as waveform records need special handling. */

#define PUBLISH_WRAPPER(record, record_name, args...) \
    (struct i_##record) { \
        .record_type = RECORD_TYPE_##record, \
        .name = record_name, args }

#define PUBLISH_WAVEFORM_WRAPPER(type, length, record_name, args...) \
    (struct i_waveform_##type) { \
        .record_type = RECORD_TYPE_waveform, \
        .name = record_name, .field_type = waveform_TYPE_##type, \
        .max_length = length, args }


/* During normal compilation the wrapped definitions are simply discarded.  A
 * second pass (with __DEFINE_EPICS__ defined) is used to extract the
 * exported definitions -- these can then be assembled into a structure to be
 * passed to PublishEpicsData(). */
#ifdef __DEFINE_EPICS__
#define PUBLISH_EXPORT_WRAPPER(publish_data) \
    __PUBLISH_DATA__::<<<(struct i_record *) &publish_data,>>>
#else
#define PUBLISH_EXPORT_WRAPPER(publish_data)
#endif



/* Publishing EPICS records through a common interface.  The most general
 * record publishing declaration is of the form:
 *
 *  PUBLISH(type, name, process, etc...)
 *
 * where the values that can be passed to etc depend on the record type. */
#define PUBLISH(args...) \
    PUBLISH_EXPORT_WRAPPER(PUBLISH_WRAPPER(args))

/* Publishing EPICS waveform records.  Theese are declared by the following
 * form:
 *
 *  PUBLISH_WAVEFORM(type, name, process, etc...)
 *
 * where etc can specify record initialisation and other parameters. */
#define PUBLISH_WAVEFORM(args...) \
    PUBLISH_EXPORT_WRAPPER(PUBLISH_WAVEFORM_WRAPPER(args))


#define PUBLISH_DYNAMIC(record, args...) \
    ( { \
        struct i_##record *iRecord = malloc(sizeof(struct i_##record)); \
        *iRecord = PUBLISH_WRAPPER(record, args); \
        PublishDynamic((struct i_record *) iRecord); \
    } )

#define PUBLISH_WAVEFORM_DYNAMIC(type, args...) \
    ( { \
        struct i_waveform_##type *iRecord = \
            malloc(sizeof(struct i_waveform_##type)); \
        *iRecord = PUBLISH_WAVEFORM_WRAPPER(type, args); \
        PublishDynamic((struct i_record *) iRecord); \
    } )



struct record_base *PublishDynamic(const struct i_record *iRecord);
bool PublishEpicsData(const struct i_record *publish_data[]);
#define PUBLISH_EPICS_DATA() \
    PublishEpicsData(EPICS_publish_data)
