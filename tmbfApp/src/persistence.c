/* Implemention of persistent state. */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h>

#include "error.h"
#include "hashtable.h"

#include "persistence.h"



/* Used to store information about individual persistent variables. */
struct persistent_variable {
    const struct persistent_action *action;
    bool value_present;
    char variable[0];
};


/* Used to implement core persistent actions, essentially converting values to
 * and from external strings. */
struct persistent_action {
    size_t size;
    /* Write value to output buffer (which must be long enough!), returns number
     * of characters written. */
    size_t (*write)(char *out, const void *variable);
    /* Reads value from given character buffer, advancing the character pointer
     * past the characters read, returns false and generates an error message if
     * there is a parsing error. */
    bool (*read)(const char **in, void *variable);
};



/******************************************************************************/
/* Reading and writing basic values. */

#define EPICS_STRING_LENGTH     40

#define DEFINE_WRITE(type, name, format) \
    static size_t write_##name(char *out, const void *variable) \
    { \
        type value; \
        memcpy(&value, variable, sizeof(type)); \
        return sprintf(out, format, value); \
    }

static bool check_number(const char *start, const char *end)
{
    return TEST_OK_(end > start  &&  errno == 0, "Error converting number");
}

#define DEFINE_READ_NUM(type, name, convert, extra...) \
    static bool read_##name(const char **string, void *variable) \
    { \
        errno = 0; \
        const char *start = *string; \
        char *end; \
        type result = (type) convert(start, &end, ##extra); \
        memcpy(variable, &result, sizeof(type)); \
        *string = end; \
        return check_number(start, *string); \
    }

DEFINE_WRITE(int32_t, int, "%d")
DEFINE_WRITE(double, double, "%.16g")
DEFINE_READ_NUM(int32_t, int, strtol, 10)
DEFINE_READ_NUM(double, double, strtod)

/* We go for the simplest possible escaping: octal escape for everything.  Alas,
 * this can quadruple the size of the buffer to 160 chars. */
static size_t write_string(char *out, const void *variable)
{
    const char *start = out;
    const char *string = variable;
    *out++ = '"';
    for (int i = 0; i < EPICS_STRING_LENGTH; i ++)
    {
        char ch = string[i];
        if (ch == '\0')
            break;
        else if (' ' <= ch  &&  ch <= '~'  &&  ch != '"'  &&  ch != '\\')
            *out++ = ch;
        else
            out += sprintf(out, "\\%03o", (unsigned char) ch);
    }
    *out++ = '"';
    *out = '\0';
    return out - start;
}

/* Parses three octal digits as part of an escape sequence. */
static bool parse_octal(const char **in, char *result)
{
    unsigned int value = 0;
    bool ok = true;
    for (int i = 0; ok  &&  i < 3; i ++)
    {
        char ch = *(*in)++;
        ok = TEST_OK_('0' <= ch  &&  ch <= '7', "Expected octal digit");
        value = (value << 3) + (ch - '0');
    }
    *result = (char) value;
    return ok;
}

/* We go for the most witless string parsing possible, must be double quoted,
 * and we only recognise octal character escapes. */
static bool read_string(const char **in, void *variable)
{
    char *string = variable;
    memset(variable, 0, EPICS_STRING_LENGTH);
    bool ok = TEST_OK_(*(*in)++ == '"', "Expected quoted string");
    for (int i = 0; ok  &&  i < EPICS_STRING_LENGTH; i ++)
    {
        char ch = *(*in)++;
        if (ch == '"')
            return true;
        else if (ch == '\\')
            ok = parse_octal(in, &string[i]);
        else
            string[i] = ch;
    }
    return ok  &&  TEST_OK_(*(*in)++ == '"', "Missing closing quote");
}


/* Note that this table is indexed by PERSISTENCE_TYPES, so any changes in one
 * must be reflected in the other. */
static const struct persistent_action persistent_actions[] = {
    { sizeof(int32_t),      write_int,      read_int },     // longout
    { sizeof(double),       write_double,   read_double },  // ao
    { EPICS_STRING_LENGTH,  write_string,   read_string },  // stringout
};


/******************************************************************************/

/* Lookup table of persistent variables. */
static struct hash_table *variable_table;
/* Flag set if persistent state needs to be written to disk. */
static bool persistence_dirty = false;
/* Persistence loaded from and written to this file. */
static const char *state_filename = NULL;
/* How long to wait between persistence wakeups. */
static int persistence_interval;

/* To ensure state is updated in a timely way we have a background thread
 * responsible for this. */
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t psignal = PTHREAD_COND_INITIALIZER;
/* Used to signal thread termination. */
static bool thread_running = true;
/* Thread handle used for shutdown. */
static pthread_t persistence_thread_id;

#define LOCK()      pthread_mutex_lock(&mutex);
#define UNLOCK()    pthread_mutex_unlock(&mutex);
#define SIGNAL()    pthread_cond_broadcast(&psignal);



/* Creates new persistent variable. */
void create_persistent_variable(const char *name, enum PERSISTENCE_TYPES type)
{
    LOCK();
    const struct persistent_action *action = &persistent_actions[type];
    struct persistent_variable *persistence =
        malloc(sizeof(struct persistent_variable) + action->size);
    persistence->action = action;
    persistence->value_present = false;

    hash_table_insert(variable_table, name, persistence);
    UNLOCK();
}


static struct persistent_variable *lookup_persistence(const char *name)
{
    struct persistent_variable *persistence =
        hash_table_lookup(variable_table, name);
    TEST_NULL_(persistence, "Persistent variable %s not found", name);
    return persistence;
}


/* Updates variable from value stored on disk. */
bool read_persistent_variable(const char *name, void *variable)
{
    LOCK();
    struct persistent_variable *persistence = lookup_persistence(name);
    bool ok = persistence != NULL  &&  persistence->value_present;
    if (ok)
        memcpy(variable, persistence->variable, persistence->action->size);
    UNLOCK();
    return ok;
}


/* Writes value to persistent variable. */
void write_persistent_variable(const char *name, const void *value)
{
    LOCK();
    struct persistent_variable *persistence = lookup_persistence(name);
    if (persistence != NULL)
    {
        /* Don't force a write of the persistence file if nothing has actually
         * changed. */
        persistence_dirty =
            persistence_dirty  ||  !persistence->value_present  ||
            memcmp(persistence->variable, value, persistence->action->size);

        persistence->value_present = true;
        memcpy(persistence->variable, value, persistence->action->size);
    }
    UNLOCK();
}


/* Parse a line of the form <key>=<value> using the parsing method associated
 * with <key>, or fail if <key> not known. */
static bool parse_line(char *line, int line_number)
{
    char *equal;
    const char *cequal;
    struct persistent_variable *persistence;
    bool ok =
        TEST_NULL_(equal = strchr(line, '='), "Missing =")  &&
        DO_(*equal++ = '\0'; cequal = equal)  &&
        TEST_NULL_(persistence = hash_table_lookup(variable_table, line),
            "Persistence key \"%s\" not found", line)  &&
        persistence->action->read(&cequal, persistence->variable)  &&
        TEST_OK_(*cequal == '\0', "Unexpected extra characters")  &&
        DO_(persistence->value_present = true);
    TEST_OK_(ok, "Parse error on line %d of state file %s\n",
        line_number, state_filename);
    return ok;
}

static bool parse_persistence_file(const char *filename)
{
    FILE *in;
    if (!TEST_NULL_(in = fopen(filename, "r"),
            "Unable to open state file %s", filename))
        /* If persistence file isn't found we report open failure but don't
         * actually fail -- this isn't an error. */
        return true;

    char line[1024];
    int line_number = 0;

    bool ok = true;
    while (fgets(line, sizeof(line), in))
    {
        line_number += 1;

        /* Skip lines beginning with # and ignore blank lines.  All other
         * lines are processed. */
        if (line[0] != '#')
        {
            int len = strlen(line);
            if (line[len - 1] == '\n')
                line[len - 1] = '\0';

            if (*line != '\0')
                /* Allow parsing of individual lines to fail, but
                 * accumulate overall error code.  This shouldn't actually
                 * ever happen, of course. */
                ok = parse_line(line, line_number)  &&  ok;
        }
    }
    fclose(in);
    return ok;
}


bool load_persistent_state(void)
{
    bool ok = true;
    if (state_filename != NULL)
    {
        LOCK();
        ok = parse_persistence_file(state_filename);
        UNLOCK();
    }
    return ok;
}


/* Writes persistent state to given file. */
static bool write_persistent_state(const char *filename)
{
    FILE *out;
    if (!TEST_NULL(out = fopen(filename, "w")))
        return false;

    /* Start with a timestamp log. */
    char out_buffer[256];
    time_t now = time(NULL);
    bool ok =
        TEST_OK(fprintf(out, "# Written: %s", ctime_r(&now, out_buffer)) > 0);

    int ix = 0;
    const void *key;
    void *value;
    while (ok  &&  hash_table_walk(variable_table, &ix, &key, &value))
    {
        const char *name = key;
        struct persistent_variable *persistence = value;
        if (persistence->value_present)
        {
            persistence->action->write(out_buffer, persistence->variable);
            ok = TEST_OK(fprintf(out, "%s=%s\n", name, out_buffer) > 0);
        }
    }
    fclose(out);
    return ok;
}

/* Updates persistent state via a backup file to avoid data loss (assuming
 * rename is implemented as an OS atomic action). */
static bool update_persistent_state(void)
{
    size_t name_len = strlen(state_filename);
    char backup_file[name_len + strlen(".backup") + 1];
    sprintf(backup_file, "%s.backup", state_filename);
    return
        write_persistent_state(backup_file)  &&
        TEST_IO(rename(backup_file, state_filename));
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Top level control. */


/* Wrapper to wait for a pthread condition with a timeout. */
#define NSECS   1000000000
static bool pwait_timeout(int secs, long nsecs)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    struct timespec timeout = {
        .tv_sec = now.tv_sec + secs,
        .tv_nsec = 1000 * now.tv_usec + nsecs
    };
    if (timeout.tv_nsec >= NSECS)
    {
        timeout.tv_nsec -= NSECS;
        timeout.tv_sec += 1;
    }
    int rc = pthread_cond_timedwait(&psignal, &mutex, &timeout);
    return rc != ETIMEDOUT;
}


/* This thread is responsible for ensuring the persistent state file is up to
 * date.  It wakes up periodically to check if anything has changed, and if it
 * has, ensures the state file is updated.  The file is also written on shutdown
 * if necessary. */
static void *persistence_thread(void *context)
{
    LOCK();
    while (thread_running)
    {
        pwait_timeout(persistence_interval, 0);
        if (persistence_dirty  &&  state_filename != NULL)
            update_persistent_state();
        persistence_dirty = false;
    }
    UNLOCK();
    return NULL;
}


/* Must be called before marking any variables as persistent. */
bool initialise_persistent_state(const char *file_name, int save_interval)
{
    state_filename = file_name;
    persistence_interval = save_interval;
    variable_table = hash_table_create(true);   // Let table look after names
    return TEST_0(pthread_create(
        &persistence_thread_id, NULL, persistence_thread, NULL));
}


/* Writes out persistent state file if necessary.  All we have to do in fact is
 * wake up the responsible thread and then wait for it to complete. */
void terminate_persistent_state(void)
{
    LOCK();
    thread_running = false;
    pthread_cond_signal(&psignal);
    UNLOCK();
    pthread_join(persistence_thread_id, NULL);
}
