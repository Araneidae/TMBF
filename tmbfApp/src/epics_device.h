/* Common interface to epics records. */

#include <alarm.h>



/* Epics strings are rather limited: a massive 39 characters are available! */
typedef char EPICS_STRING[40];

void CopyEpicsString(const EPICS_STRING in, EPICS_STRING out);


/*****************************************************************************/
/* Core EPICS publishing interface. */


/* The following list of enumerations records the possible record types
 * supported by this library. */
enum record_type
{
    RECORD_TYPE_longin,
    RECORD_TYPE_ulongin,    // An alias for longin, but binds to unsigned int
    RECORD_TYPE_longout,
    RECORD_TYPE_ulongout,   // An alias for longout
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
#define TYPEOF_ulongin   unsigned int
#define TYPEOF_longout   int
#define TYPEOF_ulongout  unsigned int
#define TYPEOF_ai        double
#define TYPEOF_ao        double
#define TYPEOF_bi        bool
#define TYPEOF_bo        bool
#define TYPEOF_stringin  EPICS_STRING
#define TYPEOF_stringout EPICS_STRING
#define TYPEOF_mbbi      unsigned int
#define TYPEOF_mbbo      unsigned int


/* Supported waveform types, matches possible values for waveform FTVL field. */
enum waveform_type {
    waveform_TYPE_void,
    waveform_TYPE_char,
    waveform_TYPE_short,
    waveform_TYPE_int,
    waveform_TYPE_float,
    waveform_TYPE_double
};



/* EPICS records are published with the use of a variety of PUBLISH...() macros,
 * defined below.  Three classes of record are supported with slightly different
 * macros and arguments:
 *
 *  IN records
 *  ----------
 *      Record types: [u]longin, ai, bi, stringin, mbbi
 *      PUBLISH(record, name, .read, .context, .io_intr, .set_time)
 *      PUBLISH_READ_VAR(record, name, variable, .io_intr, .set_time)
 *      PUBLISH_READER(record, name, reader)
 *      PUBLISH_TRIGGER(name)
 *
 *  OUT records
 *  -----------
 *      Record types: [u]longout, ao, bo, stringout, mbbo
 *      PUBLISH(record, name, .write, .init, .context, .persist)
 *      PUBLISH_WRITE_VAR(record, name, variable, .persist)
 *      PUBLISH_WRITER[_B](record, name, writer, .persist)
 *      PUBLISH_ACTION(name, action)
 *
 *  WAVEFORM records
 *  ----------------
 *      Record type: waveform
 *      PUBLISH_WAVEFORM(field_type, name, length,
 *          .process, .init, .context, .persist, .io_intr)
 *      PUBLISH_WF_READ_VAR(field_type, name, length, waveform, .io_intr)
 *      PUBLISH_WF_WRITE_VAR(field_type, name, length, waveform, .persist)
 *      PUBLISH_WF_ACTION(field_type, name, length, action, .io_intr, .persist)
 *
 * In all the named macros above the dotted arguments are optional and should be
 * given using named initialiser syntax, eg:
 *
 *      PUBLISH(longin, "RECORD", .read = on_read, .context = some_context).
 *
 * A detailed description of each form of these macros follows below.
 *
 *
 *  PUBLISH(in_record, name, .read, .context, .io_intr, .set_time)
 *
 *      bool read(void *context, TYPEOF(in_record) *value)
 *          This method will be called when the record is processed.  If
 *          possible a valid value should be assigned to *value and true
 *          returned, otherwise false can be returned to indicate no value
 *          available.
 *
 *      void *context
 *          This will be passed to the read method.
 *
 *      bool io_intr
 *          If this is set then SCAN can usefully be set to "I/O Intr" and
 *          record processing can be triggered using trigger_record().
 *
 *      bool set_time
 *          If this is set then the TSE field must be set to -2 and the
 *          timestamp must be explicitly set using trigger_record().  Generally
 *          this is most conveniently used with io_intr.
 *
 *
 *  PUBLISH(out_record, name, .write, .init, .context, .persist)
 *
 *      bool write(void *context, const TYPEOF(out_record) *value)
 *          This will be called on record processing with the value written to
 *          the record passed by reference.  If the value is accepted then true
 *          should be return, otherwise if false is returned then value is
 *          treated as being rejected.
 *
 *      bool init(void *context, TYPEOF(out_record) *value)
 *          This may be called during record initialisation to assign an initial
 *          value to the record.  False can be returned to indicate failure.
 *
 *      void *context
 *          Passed to the write and init methods.
 *
 *      bool persist
 *          If this is set then all successful writes will be mirrored to
 *          persistent storage and the record will be initialised from
 *          persistent storage if possible (in which case init will not be
 *          called).
 *
 *
 *  PUBLISH_WAVEFORM(field_type, name, max_length,
 *          .process, .init, .context, .persist, .io_intr)
 *
 *      size_t max_length
 *          Intrinsic size of the waveform.  This will be checked against the
 *          database when binding the record.
 *
 *      void process(void *context, type *array, size_t *length)
 *          This is called during record processing with *length initialised
 *          with the current waveform length.  The array can be read or written
 *          as required and *length can be updated as appropriate if the data
 *          length changes (though of course max_length must not be exceeded).
 *
 *      void init(void *context, type *array, size_t *length)
 *          This may be called during initialisation to read an initial value.
 *
 *      void *context
 *          Passed to the process and init methods.
 *
 *      bool persist
 *          If set then all updates to the waveform will be mirrored to
 *          persistent storage (after calling process) and initialised from
 *          persistent storage.
 *
 *      bool io_intr
 *          If this is set then SCAN can usefully be set to "I/O Intr" and
 *          record processing can be triggered using trigger_record().
 */

/* Structures defining the possible arguments that can be passed when publishing
 * the corresponding record types.  See detailed comments on PUBLISH() macro for
 * the specific parameters. */

#define DECLARE_IN_ARGS_(record, type) \
    struct record_args_##record { \
        bool (*read)(void *context, type *value); \
        void *context; \
        bool io_intr; \
        bool set_time; \
    }
#define DECLARE_IN_ARGS(record) \
    DECLARE_IN_ARGS_(record, TYPEOF(record))

#define DECLARE_OUT_ARGS_(record, type) \
    struct record_args_##record { \
        bool (*write)(void *context, const type *value); \
        bool (*init)(void *context, type *value); \
        void *context; \
        bool persist; \
    }
#define DECLARE_OUT_ARGS(record) \
    DECLARE_OUT_ARGS_(record, TYPEOF(record))

#define DECLARE_WAVEFORM_ARGS(type) \
    struct waveform_args_##type { \
        enum waveform_type field_type; \
        size_t max_length; \
        void (*process)(void *context, type *array, size_t *length); \
        void (*init)(void *context, type *array, size_t *length); \
        void *context; \
        bool persist; \
        bool io_intr; \
    }


/* These two macros help us to repeat definitions and declarations for groups of
 * records.  Unfortunately string{in,out} is different enough to need separate
 * treatment in general. */
#define FOR_IN_RECORDS(ACTION, sep) \
    ACTION(longin) sep \
    ACTION(ulongin) sep \
    ACTION(ai) sep \
    ACTION(bi) sep \
    ACTION(mbbi) sep
#define FOR_OUT_RECORDS(ACTION, sep) \
    ACTION(longout) sep \
    ACTION(ulongout) sep \
    ACTION(ao) sep \
    ACTION(bo) sep \
    ACTION(mbbo) sep

FOR_IN_RECORDS(DECLARE_IN_ARGS, ;)
DECLARE_IN_ARGS(stringin);

FOR_OUT_RECORDS(DECLARE_OUT_ARGS, ;)
DECLARE_OUT_ARGS(stringout);

DECLARE_WAVEFORM_ARGS(void);
DECLARE_WAVEFORM_ARGS(char);
DECLARE_WAVEFORM_ARGS(short);
DECLARE_WAVEFORM_ARGS(int);
DECLARE_WAVEFORM_ARGS(float);
DECLARE_WAVEFORM_ARGS(double);


/* This is the abstract interface returned by all core publish methods.  Only a
 * handful of operations are supported. */
struct epics_record;

/* Makes the named record of the given type available for binding.  The
 * particular structure passed to args is determined by the record type, this
 * function should only ever be called via the PUBLISH() macro below. */
struct epics_record *publish_epics_record(
    enum record_type record_type, const char *name, const void *args);

#define PUBLISH(record, name, args...) \
    publish_epics_record(RECORD_TYPE_##record, name, \
        &(const struct record_args_##record) { args })

#define PUBLISH_WAVEFORM(type, record_name, length, args...) \
    publish_epics_record(RECORD_TYPE_waveform, record_name, \
        &(const struct waveform_args_##type) { \
            .field_type = waveform_TYPE_##type, .max_length = length, ##args })


/* This updates the recorded record severity and optionally, if .set_time was
 * specified, the record timestamp.  If .io_intr was set then record processing
 * will be triggered.  This can only be called for IN and WAVEFORM records. */
void trigger_record(
    struct epics_record *record, epicsAlarmSeverity severity,
    struct timespec *timestamp);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* The following macros implement derived operations to simplify the publishing
 * of common structures including variables and simple actions without any extra
 * context. */

#define PUBLISH_READ_VAR(record, name, variable, args...) \
    PUBLISH(record, name, \
        .read = publish_var_read_##record, \
        .context = *(TYPEOF(record)*[]) { &(variable) }, ##args)

#define PUBLISH_READER(record, name, reader) \
    PUBLISH(record, name, \
        .read = publish_reader_##record, \
        .context = *(TYPEOF(record) (*[])(void)) { reader })

#define PUBLISH_TRIGGER(name) \
    PUBLISH(bi, name, .read = publish_trigger_bi, .io_intr = true)

#define PUBLISH_WRITE_VAR(record, name, variable, args...) \
    PUBLISH(record, name, \
        .write = publish_var_write_##record, \
        .init = publish_var_read_##record, \
        .context = *(TYPEOF(record)*[]) { &(variable) }, ##args)

#define PUBLISH_WRITER(record, name, writer, args...) \
    PUBLISH(record, name, \
        .write = publish_writer_##record, \
        .context = *(void (*[])(TYPEOF(record))) { writer }, ##args)

#define PUBLISH_WRITER_B(record, name, writer, args...) \
    PUBLISH(record, name, \
        .write = publish_writer_b_##record, \
        .context = *(bool (*[])(TYPEOF(record))) { writer }, ##args)

#define PUBLISH_ACTION(name, action) \
    PUBLISH(bo, name, .write = publish_action_bo, \
        .context = *(void (*[])(void)) { action })

#define PROC_WAVEFORM_T(type) \
    void (*)(void * context, type *array, size_t *length)
#define PUBLISH_WF_READ_VAR(type, name, length, waveform, args...) \
    PUBLISH_WAVEFORM(type, name, length, \
        .process = (PROC_WAVEFORM_T(type)) publish_waveform_read_var, \
        .init    = (PROC_WAVEFORM_T(type)) publish_waveform_read_var, \
        .context = make_waveform_context( \
            sizeof(type), length, *(type *[]) { waveform }), ##args)
#define PUBLISH_WF_WRITE_VAR(type, name, length, waveform, args...) \
    PUBLISH_WAVEFORM(type, name, length, \
        .process = (PROC_WAVEFORM_T(type)) publish_waveform_write_var, \
        .init    = (PROC_WAVEFORM_T(type)) publish_waveform_read_var, \
        .context = make_waveform_context( \
            sizeof(type), length, *(type *[]) { waveform }), ##args)

#define PUBLISH_WF_ACTION(type, name, length, action, args...) \
    PUBLISH_WAVEFORM(type, name, length, \
        .process = (PROC_WAVEFORM_T(type)) publish_waveform_action, \
        .context = make_waveform_context( \
            sizeof(type), length, *(void (*[])(type *)) { action }), ##args)


/* Declarations for the support methods referenced above. */
#define DECLARE_READ_VAR(record) \
    bool publish_var_read_##record(void *context, TYPEOF(record) *value)
#define DECLARE_WRITE_VAR(record) \
    bool publish_var_write_##record(void *context, const TYPEOF(record) *value)
#define DECLARE_READER(record) \
    bool publish_reader_##record(void *context, TYPEOF(record) *value)
#define DECLARE_WRITER(record) \
    bool publish_writer_##record(void *context, const TYPEOF(record) *value)
#define DECLARE_WRITER_B(record) \
    bool publish_writer_b_##record(void *context, const TYPEOF(record) *value)

FOR_IN_RECORDS(DECLARE_READ_VAR, ;)
FOR_OUT_RECORDS(DECLARE_WRITE_VAR, ;)
FOR_OUT_RECORDS(DECLARE_READ_VAR, ;)
FOR_IN_RECORDS(DECLARE_READER, ;)
FOR_OUT_RECORDS(DECLARE_WRITER, ;)
FOR_OUT_RECORDS(DECLARE_WRITER_B, ;)

bool publish_trigger_bi(void *context, bool *value);
bool publish_action_bo(void *context, const bool *value);

void publish_waveform_action(void *context, void *array, size_t *length);
void publish_waveform_write_var(void *context, void *array, size_t *length);
void publish_waveform_read_var(void *context, void *array, size_t *length);
void *make_waveform_context(size_t size, size_t length, void *context);
