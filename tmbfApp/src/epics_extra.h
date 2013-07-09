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
struct epics_interlock *create_interlock(
    const char *trigger_name, const char *done_name, bool set_time);

/* Blocks until EPICS reports back by processing the DONE record.  The first
 * call must be made before calling interlock_signal() and will wait for EPICS
 * to finish initialising. */
void interlock_wait(struct epics_interlock *interlock);

/* Signals the interlock.  If set_time was specified then a timestamp should be
 * passed through, otherwise NULL will serve. */
void interlock_signal(struct epics_interlock *interlock, struct timespec *time);


/* Needs to be called early, before EPICS initialisation completes. */
bool initialise_epics_extra(void);
