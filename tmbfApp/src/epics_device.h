/* Common interface to EPICS records.
 *
 * EPICS records are published with the use of a variety of PUBLISH...() macros,
 * defined below.  Three classes of record are supported with slightly different
 * macros and arguments:
 *
 *  IN records
 *  ----------
 *      Record types: [u]longin, ai, bi, stringin, mbbi
 *      PUBLISH(record, name, read, .context, .io_intr, .set_time)
 *      PUBLISH_READ_VAR[_I](record, name, variable)
 *      PUBLISH_READER[_I](record, name, reader)
 *      PUBLISH_TRIGGER[_T](name)
 *
 *  OUT records
 *  -----------
 *      Record types: [u]longout, ao, bo, stringout, mbbo
 *      PUBLISH(record, name, write, .init, .context, .persist)
 *      PUBLISH_WRITE_VAR[_P](record, name, variable)
 *      PUBLISH_WRITER[_B][_P](record, name, writer)
 *      PUBLISH_ACTION(name, action)
 *
 *  WAVEFORM records
 *  ----------------
 *      Record type: waveform
 *      PUBLISH_WAVEFORM(field_type, name, length, process,
 *          .init, .context, .persist, .io_intr)
 *      PUBLISH_WF_READ_VAR[_I](field_type, name, length, waveform)
 *      PUBLISH_WF_WRITE_VAR[_P](field_type, name, length, waveform)
 *      PUBLISH_WF_ACTION{,_I,_P}(field_type, name, length, action)
 *
 * Suffixes:
 *      _I  Sets .io_intr to enable "I/O Intr" scanning
 *      _P  Sets .persist to enable persistent storage
 *      _T  Sets .set_time to enable timestamp override
 *      _B  Enables writer to return bool result
 *
 *
 * In the examples above the dotted arguments are optional and should be
 * specified using C99 named initialiser syntax, eg:
 *
 *      PUBLISH(longin, "RECORD", on_read, .context = read_context).
 *
 *
 * The following two macros define the core interface.
 *
 *  PUBLISH(in_record, name, read, .context, .io_intr, .set_time)
 *  PUBLISH(out_record, name, write, .init, .context, .persist)
 *
 *      The PUBLISH macro is used to create a software binding for the
 *      appropriate record type to the given name.  The corresponding read or
 *      write method will be called when the record processes, and the macro
 *      ensures proper type checking.  Note that IN records and OUT records
 *      support different arguments.
 *
 *      The following macros provide support for more specialised variants of
 *      these records with hard-wired implementations of the read and write
 *      methods.
 *
 *      For IN records the returned value is a struct epics_record* which can be
 *      passed to trigger_record().
 *
 *      bool read(void *context, TYPEOF(in_record) *value)
 *          For IN records this method will be called when the record is
 *          processed.  If possible a valid value should be assigned to *value
 *          and true returned, otherwise false can be returned to indicate no
 *          value available.
 *
 *      bool write(void *context, const TYPEOF(out_record) *value)
 *          For OUT records this will be called on record processing with the
 *          value written to the record passed by reference.  If the value is
 *          accepted then true should be return, otherwise if false is returned
 *          then value is treated as being rejected.
 *
 *      bool init(void *context, TYPEOF(out_record) *value)
 *          For OUT records this may be called during record initialisation to
 *          assign an initial value to the record.  False can be returned to
 *          indicate failure.  If .persist is set and a value is successfully
 *          read from storage then this method will be ignored.
 *
 *  PUBLISH_WAVEFORM(field_type, name, max_length, process,
 *      .init, .context, .persist, .io_intr)
 *
 *      The PUBLISH_WAVEFORM creates the software binding for waveform records
 *      with data of the specified type.  The process method will be called each
 *      time the record processes -- the process method can choose whether to
 *      implement reading or writing as the primitive operation.
 *
 *      void process(void *context, field_type *array, size_t *length)
 *          This is called during record processing with *length initialised
 *          with the current waveform length.  The array can be read or written
 *          as required and *length can be updated as appropriate if the data
 *          length changes (though of course max_length must not be exceeded).
 *
 *      void init(void *context, field_type *array, size_t *length)
 *          This may be called during initialisation to read an initial value.
 *
 *
 *  The following arguments are common between both of these macros:
 *
 *      void *context
 *          For the top level publish macros PUBLISH and PUBLISH_WAVEFORM the
 *          context argument is passed through to each of the given callbacks to
 *          create a closure binding.
 *
 *      bool io_intr
 *          For IN and WAVEFORM records it is possible to set SCAN="I/O Intr"
 *          and trigger processing internally (by calling trigger_record()).
 *          For this to work this flag must also be set.  The _I macro variants
 *          automatically set this flag.
 *
 *      bool set_time
 *          For IN records it is also possible to specify the timestamp when
 *          triggering the record.  In this case the TSE field must be set to -2
 *          and trigger_record() must be used to explicitly set the timestamp.
 *          This is most conveniently combined with io_intr.  The _T macro
 *          variant automatically sets this flag.
 *
 *      bool persist
 *          For OUT and WAVEFORM records this flag can be set to ensure that all
 *          successful writes are mirrored to persistent storage, and the record
 *          will be initialised from persistent storage if possible.
 *
 *
 * The following macros provide specialisation for specific types of record.
 *
 *  PUBLISH_READ_VAR(record, name, variable)
 *  PUBLISH_READ_VAR_I(record, name, variable)
 *
 *      The given variable will be read each time the record is processed.
 *
 *  PUBLISH_READER(record, name, reader)
 *  PUBLISH_READER_I(record, name, reader)
 *
 *      TYPEOF(record) reader(void)
 *          This will be called each time the record processes and should return
 *          the value to be reported.
 *
 *  PUBLISH_TRIGGER(name)
 *  PUBLISH_TRIGGER_T(name)
 *
 *      This record is useful for generating triggers into the database.  The
 *      record type is set to bi and the io_intr flag is set.  Call
 *      trigger_record() to make this record process, use FLNK in the database
 *      to build a useful processing chain.
 *
 *  PUBLISH_WRITE_VAR(record, name, variable)
 *  PUBLISH_WRITE_VAR_P(record, name, variable)
 *
 *      The variable is written each time the record is processed and is read on
 *      startup to initialise the associated EPICS record.
 *
 *  PUBLISH_WRITER(record, name, writer)
 *  PUBLISH_WRITER_P(record, name, writer)
 *  PUBLISH_WRITER_B(record, name, writer)
 *  PUBLISH_WRITER_B_P(record, name, writer)
 *
 *      void writer(TYPEOF(record) value)
 *      bool writer(TYPEOF(record) value)
 *          This method will be called each time the record processes.  For the
 *          _B variants the writer can return a boolean to optionally reject the
 *          write, otherwise void is returned and the write is unconditional.
 *
 *  PUBLISH_ACTION(name, action)
 *
 *      void action(void)
 *          This method is called when the record processes.
 *
 *  PUBLISH_WF_READ_VAR(field_type, name, max_length, waveform)
 *  PUBLISH_WF_READ_VAR_I(field_type, name, max_length, waveform)
 *
 *      The waveform will be read each time the record processes.
 *
 *  PUBLISH_WF_WRITE_VAR(field_type, name, max_length, waveform)
 *  PUBLISH_WF_WRITE_VAR_P(field_type, name, max_length, waveform)
 *
 *      The waveform will be written each time the record processes.
 *
 *  PUBLISH_WF_ACTION(field_type, name, max_length, action)
 *  PUBLISH_WF_ACTION_I(field_type, name, max_length, action)
 *  PUBLISH_WF_ACTION_P(field_type, name, max_length, action)
 *
 *      void action(field_type value[max_length])
 *          This is called each time the record processes.  It is up to the
 *          implementation of action() to determine whether this is a read or a
 *          write action.
 *
 * The following arguments are common to most or several of the macros above:
 *
 *      record
 *          The first argument to most of the publishing macros defines the
 *          associated record type, and the possible record types for each macro
 *          are listed above.  Possible values for record are:
 *
 *              longin, ulongin, ai, bi, stringin, mbbi (all IN records)
 *              longout, ulongout, ao, bo, stringout, mbbo (all OUT records)
 *
 *         Note that the record types ulongin and ulongout are aliases for
 *         longin and longout respectively with unsigned types.  The underlying
 *         EPICS type remains signed, this is just a convenience for interfacing
 *         to unsigned data.
 *
 *      const char *name
 *          The name field present in all publication macros and determines the
 *          name that will be used to bind the published record to the record
 *          definition in the EPICS database.  The corresponding name should
 *          also occur in the INP or OUT field of the appropriate database
 *          entry.
 *
 *      TYPEOF(record) variable
 *          A variable is passed to the _VAR macros.  Its address is stored and
 *          the variable is read or written as appropriate when the record is
 *          processed.
 *
 *      field_type waveform[max_length]
 *          A waveform is passed to the _WF_VAR macros, and is read or written
 *          as approprate when the record is processed.
 *
 * The following arguments are common to all WAVEFORM macros:
 *
 *      field_type
 *          Specifies the data type for the waveform, can be one of: char,
 *          short, int, float, double.
 *
 *      size_t max_length
 *          Intrinsic size of the waveform.  This will be checked against the
 *          database when binding the record.
 */

#include <alarm.h>


/* This must be called once before publishing any PVs. */
bool initialise_epics_device(void);


/*****************************************************************************/
/* Basic types. */

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

#define TYPEOF_longin    int32_t
#define TYPEOF_ulongin   uint32_t
#define TYPEOF_longout   int32_t
#define TYPEOF_ulongout  uint32_t
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


/* Epics strings are rather limited: a massive 39 characters are available! */
typedef struct epics_string { char s[40]; } EPICS_STRING;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Core record publishing interface. */

/* This is the abstract interface returned by all core publish methods.  At
 * present only trigger_record() is supported. */
struct epics_record;

/* Makes the named record of the given type available for binding.  The
 * particular structure passed to args is determined by the record type, this
 * function should only ever be called via the PUBLISH() or PUBLISH_WAVEFORM()
 * macros below. */
struct epics_record *publish_epics_record(
    enum record_type record_type, const char *name, const void *args);

/* This updates the recorded record severity and optionally, if .set_time was
 * specified, the record timestamp.  If .io_intr was set then record processing
 * will be triggered.  This can only be called for IN and WAVEFORM records. */
void trigger_record(
    struct epics_record *record, epicsAlarmSeverity severity,
    const struct timespec *timestamp);

/* Simple helper for EPICS_STRING type. */
void copy_epics_string(EPICS_STRING *out, const char *in);

/* Returns published epics_record structure with the given type and name.
 * Generates ASSERT fail if record not present. */
struct epics_record *lookup_epics_record(
    enum record_type record_type, const char *name);
#define LOOKUP_RECORD(record, name) \
    lookup_epics_record(RECORD_TYPE_##record, name)


/* Core PUBLISH and PUBLISH_WAVEFORM macros.  Wrappers for publish_epics_record
 * above, possible argument structures defined below. */

#define PUBLISH(record, name, args...) \
    publish_epics_record(RECORD_TYPE_##record, name, \
        &(const struct record_args_##record) { args })

#define PUBLISH_WAVEFORM(type, record_name, length, args...) \
    publish_epics_record(RECORD_TYPE_waveform, record_name, \
        &(const struct waveform_args_##type) { \
            .field_type = waveform_TYPE_##type, .max_length = length, ##args })


/* This function (wrapped by a type dispatch macro) allows the value of an out
 * record to be updated from within the device.  If process is False then the
 * generated process callback is suppressed (as far as possible).  This method
 * is only available for out records. */
void _write_out_record_value(
    enum record_type record_type, struct epics_record *record,
    const void *value, bool process);
void _write_out_record_waveform(
    enum waveform_type waveform_type, struct epics_record *record,
    const void *value, size_t length, bool process);
#define WRITE_OUT_RECORD(type, record, value, process) \
    _write_out_record_value( \
        RECORD_TYPE_##type, record, (const TYPEOF(type)[]) { value }, process)
#define WRITE_OUT_RECORD_WF(type, record, value, length, process) \
    _write_out_record_waveform( \
        waveform_TYPE_##type, record, *(const type*[]) { value }, \
        length, process)
/* Helper macro for writing a value to a named record. */
#define WRITE_NAMED_RECORD(record, name, value) \
    WRITE_OUT_RECORD( \
        record, LOOKUP_RECORD(record, name), (value), true)
#define WRITE_NAMED_RECORD_WF(type, name, value, length) \
    WRITE_OUT_RECORD_WF( \
        type, LOOKUP_RECORD(waveform, name), (value), length, true)


/* The value of any managed record can be read. */
void _read_record_value(
    enum record_type record_type, struct epics_record *record, void *value);
void _read_record_waveform(
    enum waveform_type waveform_type, struct epics_record *record,
    void *value, size_t length);
#define READ_RECORD_VALUE(type, record) \
    ( { \
        TYPEOF(type) value__; \
        _read_record_value(RECORD_TYPE_##type, record, &value__); \
        value__; \
    } )
#define READ_RECORD_VALUE_WF(type, record, value, length) \
    _read_record_waveform( \
        waveform_TYPE_##type, record, *(type *[]) { value }, length)

#define READ_NAMED_RECORD(record, name) \
    READ_RECORD_VALUE(record, LOOKUP_RECORD(record, name))
#define READ_NAMED_RECORD_WF(type, name, value, length) \
    READ_RECORD_VALUE_WF( \
        type, LOOKUP_RECORD(waveform, name), value, length)


/******************************************************************************/
/* Detailed macro based definitions.
 *
 * Because of the uniform but polymorphic character of the EPICS record
 * interface defined here it is necessary to make extensive use of macros to
 * ensure that we can get as much strict type checking as possible.
 *
 * All identifiers starting with _ should be treated as internal and should not
 * be used outside the implementation of this library. */


/* Structures defining the possible arguments that can be passed when publishing
 * the corresponding record types.  See detailed comments above for the specific
 * parameters. */

#define _DECLARE_IN_ARGS_(record, type) \
    struct record_args_##record { \
        bool (*read)(void *context, type *value); \
        void *context; \
        bool io_intr; \
        bool set_time; \
    }
#define _DECLARE_IN_ARGS(record) \
    _DECLARE_IN_ARGS_(record, TYPEOF(record))

#define _DECLARE_OUT_ARGS_(record, type) \
    struct record_args_##record { \
        bool (*write)(void *context, const type *value); \
        bool (*init)(void *context, type *value); \
        void *context; \
        bool persist; \
    }
#define _DECLARE_OUT_ARGS(record) \
    _DECLARE_OUT_ARGS_(record, TYPEOF(record))

#define _DECLARE_WAVEFORM_ARGS(type) \
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
 * records. */
#define _FOR_IN_RECORDS(ACTION, sep) \
    ACTION(longin) sep \
    ACTION(ulongin) sep \
    ACTION(ai) sep \
    ACTION(bi) sep \
    ACTION(mbbi) sep \
    ACTION(stringin) sep
#define _FOR_OUT_RECORDS(ACTION, sep) \
    ACTION(longout) sep \
    ACTION(ulongout) sep \
    ACTION(ao) sep \
    ACTION(bo) sep \
    ACTION(mbbo) sep \
    ACTION(stringout) sep

_FOR_IN_RECORDS(_DECLARE_IN_ARGS, ;)
_FOR_OUT_RECORDS(_DECLARE_OUT_ARGS, ;)

_DECLARE_WAVEFORM_ARGS(void);
_DECLARE_WAVEFORM_ARGS(char);
_DECLARE_WAVEFORM_ARGS(short);
_DECLARE_WAVEFORM_ARGS(int);
_DECLARE_WAVEFORM_ARGS(float);
_DECLARE_WAVEFORM_ARGS(double);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* The following macros implement derived operations to simplify the publishing
 * of common structures including variables and simple actions without any extra
 * context. */

#define PUBLISH_READ_VAR_(record, name, variable, args...) \
    PUBLISH(record, name, \
        .read = _publish_var_read_##record, \
        .context = *(TYPEOF(record)*[]) { &(variable) }, ##args)
#define PUBLISH_READ_VAR(record, name, variable) \
    PUBLISH_READ_VAR_(record, name, variable)
#define PUBLISH_READ_VAR_I(record, name, variable) \
    PUBLISH_READ_VAR_(record, name, variable, .io_intr = true)

#define PUBLISH_READER_(record, name, reader, args...) \
    PUBLISH(record, name, \
        .read = _publish_reader_##record, \
        .context = *(TYPEOF(record) (*[])(void)) { reader }, ##args)
#define PUBLISH_READER(record, name, reader) \
    PUBLISH_READER_(record, name, reader)
#define PUBLISH_READER_I(record, name, reader) \
    PUBLISH_READER_(record, name, reader, .io_intr = true)

#define PUBLISH_TRIGGER_(name, args...) \
    PUBLISH(bi, name, .read = _publish_trigger_bi, .io_intr = true, ##args)
#define PUBLISH_TRIGGER(name) \
    PUBLISH_TRIGGER_(name)
#define PUBLISH_TRIGGER_T(name) \
    PUBLISH_TRIGGER_(name, .set_time = true)

#define PUBLISH_WRITE_VAR_(record, name, variable, args...) \
    PUBLISH(record, name, \
        .write = _publish_var_write_##record, \
        .init = _publish_var_read_##record, \
        .context = *(TYPEOF(record)*[]) { &(variable) }, ##args)
#define PUBLISH_WRITE_VAR(record, name, variable) \
    PUBLISH_WRITE_VAR_(record, name, variable)
#define PUBLISH_WRITE_VAR_P(record, name, variable) \
    PUBLISH_WRITE_VAR_(record, name, variable, .persist = true)

#define PUBLISH_WRITER(record, name, writer, args...) \
    PUBLISH(record, name, \
        .write = _publish_writer_##record, \
        .context = *(void (*[])(TYPEOF(record))) { writer }, ##args)
#define PUBLISH_WRITER_P(record, name, writer, args...) \
    PUBLISH_WRITER(record, name, writer, .persist = true, ##args)

#define PUBLISH_WRITER_B(record, name, writer, args...) \
    PUBLISH(record, name, \
        .write = _publish_writer_b_##record, \
        .context = *(bool (*[])(TYPEOF(record))) { writer }, ##args)
#define PUBLISH_WRITER_B_P(record, name, writer, args...) \
    PUBLISH_WRITER_B_(record, name, writer, .persist = true, ##args)

#define PUBLISH_ACTION(name, action) \
    PUBLISH(bo, name, .write = _publish_action_bo, \
        .context = *(void (*[])(void)) { action })


#define PROC_WAVEFORM_T(type) \
    void (*)(void *context, type *array, size_t *length)

#define PUBLISH_WF_READ_VAR_(type, name, length, waveform, args...) \
    PUBLISH_WAVEFORM(type, name, length, \
        .process = (PROC_WAVEFORM_T(type)) _publish_waveform_read_var, \
        .init    = (PROC_WAVEFORM_T(type)) _publish_waveform_read_var, \
        .context = _make_waveform_context( \
            sizeof(type), length, *(type *[]) { waveform }), ##args)
#define PUBLISH_WF_READ_VAR(type, name, length, waveform) \
    PUBLISH_WF_READ_VAR_(type, name, length, waveform)
#define PUBLISH_WF_READ_VAR_I(type, name, length, waveform) \
    PUBLISH_WF_READ_VAR_(type, name, length, waveform, .io_intr = true)

#define PUBLISH_WF_WRITE_VAR_(type, name, length, waveform, args...) \
    PUBLISH_WAVEFORM(type, name, length, \
        .process = (PROC_WAVEFORM_T(type)) _publish_waveform_write_var, \
        .init    = (PROC_WAVEFORM_T(type)) _publish_waveform_read_var, \
        .context = _make_waveform_context( \
            sizeof(type), length, *(type *[]) { waveform }), ##args)
#define PUBLISH_WF_WRITE_VAR(type, name, length, waveform) \
    PUBLISH_WF_WRITE_VAR_(type, name, length, waveform)
#define PUBLISH_WF_WRITE_VAR_P(type, name, length, waveform) \
    PUBLISH_WF_WRITE_VAR_(type, name, length, waveform, .persist = true)

#define PUBLISH_WF_ACTION(type, name, length, action, args...) \
    PUBLISH_WAVEFORM(type, name, length, \
        .process = (PROC_WAVEFORM_T(type)) _publish_waveform_action, \
        .context = _make_waveform_context( \
            sizeof(type), length, *(void (*[])(type *)) { action }), ##args)
#define PUBLISH_WF_ACTION_I(type, name, length, action) \
    PUBLISH_WF_ACTION(type, name, length, action, .io_intr = true)
#define PUBLISH_WF_ACTION_P(type, name, length, action) \
    PUBLISH_WF_ACTION(type, name, length, action, .persist = true)


/* Declarations for the support methods referenced above. */
#define _DECLARE_READ_VAR(record) \
    bool _publish_var_read_##record(void *context, TYPEOF(record) *value)
#define _DECLARE_WRITE_VAR(record) \
    bool _publish_var_write_##record(void *context, const TYPEOF(record) *value)
#define _DECLARE_READER(record) \
    bool _publish_reader_##record(void *context, TYPEOF(record) *value)
#define _DECLARE_WRITER(record) \
    bool _publish_writer_##record(void *context, const TYPEOF(record) *value)
#define _DECLARE_WRITER_B(record) \
    bool _publish_writer_b_##record(void *context, const TYPEOF(record) *value)

_FOR_IN_RECORDS(_DECLARE_READ_VAR, ;)
_FOR_OUT_RECORDS(_DECLARE_WRITE_VAR, ;)
_FOR_OUT_RECORDS(_DECLARE_READ_VAR, ;)
_FOR_IN_RECORDS(_DECLARE_READER, ;)
_FOR_OUT_RECORDS(_DECLARE_WRITER, ;)
_FOR_OUT_RECORDS(_DECLARE_WRITER_B, ;)

bool _publish_trigger_bi(void *context, bool *value);
bool _publish_action_bo(void *context, const bool *value);

void _publish_waveform_action(void *context, void *array, size_t *length);
void _publish_waveform_write_var(void *context, void *array, size_t *length);
void _publish_waveform_read_var(void *context, void *array, size_t *length);
void *_make_waveform_context(size_t size, size_t length, void *context);
