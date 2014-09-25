/* Extra support built on top of epics_device. */

/* Trigger and interlock support.
 *
 * Implements synchronisation with epics by publishing two records, named "TRIG"
 * and "DONE".  The database should be configured to use these thus:
 *
 *      record(bi, "TRIG")
 *      {
 *          field(SCAN, "I/O Intr")
 *          field(FLNK, "FANOUT")
 *      }
 *      record(fanout, "FANOUT")
 *      {
 *          field(LNK1, "first-record")     # Process all associated
 *          ...                             # records here
 *          field(LNKn, "DONE")
 *      }
 *      record(bo, "DONE") { }
 *
 * In other words, TRIG should initiate processing on all records in its group
 * and then DONE should be processed to indicate that all processing is
 * complete.  The EPICS driver will then block between signalling TRIG and
 * receiving receipt of DONE to ensure that the record processing block
 * retrieves a consistent set of data.
 *
 * The underling driver code should be of the form
 *
 *      while(running)
 *      {
 *          wait for event;
 *          interlock_wait(interlock);
 *          process data for epics;
 *          interlock_signal(interlock, NULL);
 *      }
 *
 * Note that wait()ing is the first action: this is quite important. */

struct epics_interlock;

/* Creates an interlock with the given names.  If set_time is set then a
 * timestamp should be passed when signalling the interlock. */
struct epics_interlock *create_interlock(const char *base_name, bool set_time);

/* Blocks until EPICS reports back by processing the DONE record.  The first
 * call must be made before calling interlock_signal() and will wait for EPICS
 * to finish initialising. */
void interlock_wait(struct epics_interlock *interlock);

/* Signals the interlock.  If set_time was specified then a timestamp should be
 * passed through, otherwise NULL will serve. */
void interlock_signal(struct epics_interlock *interlock, struct timespec *time);


/* Needs to be called early, before EPICS initialisation completes. */
bool initialise_epics_extra(void);


/* This routine will block until EPICS had finished startup.  This can be called
 * first to ensure that interlock_wait(...) will not block. */
void wait_for_epics_start(void);

/* Returns true if EPICS initialisation has finished, false if
 * wait_for_epics_start() will block. */
bool check_epics_ready(void);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Support for records with data stored as part of the record.
 *
 * The API here consists of the following calls:
 *
 *  record = PUBLISH_IN_VALUE[_I][_T](type, name)
 *      Publishes EPICS PV with writeable value stored as part of the record.
 *
 *  WRITE_IN_RECORD(type, record, value, .severity, .timestamp, .force_update)
 *      Updates record with new value.  Optionally a .severity and a .timestamp
 *      can be specified, though the .timestamp value is ignored if _T was not
 *      used when publishing the record.
 *
 *  value = READ_IN_RECORD(type, record)
 *      Returns value written to record.
 */

struct in_epics_record_longin;
struct in_epics_record_ulongin;
struct in_epics_record_ai;
struct in_epics_record_bi;
struct in_epics_record_string;
struct in_epics_record_mbbi;
struct in_epics_record_;

#define CONVERT_TYPE(from, to, value) \
    ( { \
        union { from f; to t; } _convert = { .f = (value) }; \
        _convert.t; \
    } )
#define _CONVERT_TO_IN_RECORD(type, record) \
    CONVERT_TYPE( \
        struct in_epics_record_##type *, struct in_epics_record_ *, record)


struct publish_in_epics_record_args {
    bool io_intr;
    bool set_time;
    bool force_update;
};
struct in_epics_record_ *_publish_write_epics_record(
    enum record_type record_type, const char *name,
    const struct publish_in_epics_record_args *args);
#define PUBLISH_IN_VALUE(type, name, args...) \
    CONVERT_TYPE( \
        struct in_epics_record_ *, struct in_epics_record_##type *, \
        _publish_write_epics_record(RECORD_TYPE_##type, name, \
            &(const struct publish_in_epics_record_args) { args }))
#define PUBLISH_IN_VALUE_I(type, name) \
    PUBLISH_IN_VALUE(type, name, .io_intr = true)
#define PUBLISH_IN_VALUE_T(type, name) \
    PUBLISH_IN_VALUE(type, name, .set_time = true)
#define PUBLISH_IN_VALUE_I_T(type, name) \
    PUBLISH_IN_VALUE(type, name, .io_intr = true, .set_time = true)


struct write_in_epics_record_args {
    epicsAlarmSeverity severity;
    const struct timespec *timestamp;
    bool force_update;
};

void _write_in_record(
    enum record_type record_type, struct in_epics_record_ *record,
    const void *value, const struct write_in_epics_record_args *args);
#define WRITE_IN_RECORD(type, record, value, args...) \
    _write_in_record( \
        RECORD_TYPE_##type, _CONVERT_TO_IN_RECORD(type, record), \
        (const TYPEOF(type)[]) { value }, \
        &(const struct write_in_epics_record_args) { args })
#define WRITE_IN_RECORD_SEV(type, record, severity, args...) \
    _write_in_record( \
        RECORD_TYPE_##type, _CONVERT_TO_IN_RECORD(type, record), NULL, \
        &(const struct write_in_epics_record_args) { \
            .severity = severity, ##args })


void *_read_in_record(
    enum record_type record_type, struct in_epics_record_ *record);
#define READ_IN_RECORD(type, record) \
    (* (const TYPEOF(type) *) _read_in_record( \
        RECORD_TYPE_##type, _CONVERT_TO_IN_RECORD(type, record))
