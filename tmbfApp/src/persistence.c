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
    size_t max_length;
    size_t length;
    char variable[0];
};


/* Used to implement core persistent actions, essentially converting values to
 * and from external strings. */
struct persistent_action {
    size_t size;
    /* Write value to output buffer (which must be long enough!), returns number
     * of characters written. */
    void (*write)(FILE *out, const void *variable);
    /* Reads value from given character buffer, advancing the character pointer
     * past the characters read, returns false and generates an error message if
     * there is a parsing error. */
    bool (*read)(const char **in, void *variable);
};



/******************************************************************************/
/* Reading and writing basic values. */

#define EPICS_STRING_LENGTH     40

#define DEFINE_WRITE(type, format) \
    static void write_##type(FILE *out, const void *variable) \
    { \
        type value; \
        memcpy(&value, variable, sizeof(type)); \
        fprintf(out, format, value); \
    }

DEFINE_WRITE(int8_t, "%d")
DEFINE_WRITE(int16_t, "%d")
DEFINE_WRITE(int32_t, "%d")
DEFINE_WRITE(float, "%.8g")
DEFINE_WRITE(double, "%.17g")


static bool check_number(const char *start, const char *end)
{
    return TEST_OK_(end > start  &&  errno == 0, "Error converting number");
}



#define DEFINE_READ_NUM(type, convert, extra...) \
    static bool read_##type(const char **string, void *variable) \
    { \
        errno = 0; \
        const char *start = *string; \
        char *end; \
        type result = (type) convert(start, &end, ##extra); \
        memcpy(variable, &result, sizeof(type)); \
        *string = end; \
        return check_number(start, *string); \
    }

DEFINE_READ_NUM(int8_t, strtol, 10)
DEFINE_READ_NUM(int16_t, strtol, 10)
DEFINE_READ_NUM(int32_t, strtol, 10)
DEFINE_READ_NUM(float, strtof)
DEFINE_READ_NUM(double, strtod)


static void write_bool(FILE *out, const void *variable)
{
    bool value = *(const bool *) variable;
    fputc(value ? 'Y' : 'N', out);
}

static bool read_bool(const char **in, void *variable)
{
    char ch = *(*in)++;
    *(bool *) variable = ch == 'Y';
    return TEST_OK_(ch == 'Y'  ||  ch == 'N', "Invalid boolean value");
}


/* We go for the simplest possible escaping: octal escape for everything.  Alas,
 * this can quadruple the size of the buffer to 160 chars. */
static void write_string(FILE *out, const void *variable)
{
    const char *string = variable;
    fputc('"', out);
    for (int i = 0; i < EPICS_STRING_LENGTH; i ++)
    {
        char ch = string[i];
        if (ch == '\0')
            break;
        else if (' ' <= ch  &&  ch <= '~'  &&  ch != '"'  &&  ch != '\\')
            fputc(ch, out);
        else
            fprintf(out, "\\%03o", (unsigned char) ch);
    }
    fputc('"', out);
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
    { sizeof(bool),         write_bool,     read_bool },
    { sizeof(int8_t),       write_int8_t,   read_int8_t },
    { sizeof(int16_t),      write_int16_t,  read_int16_t },
    { sizeof(int32_t),      write_int32_t,  read_int32_t },
    { sizeof(float),        write_float,    read_float },
    { sizeof(double),       write_double,   read_double },
    { EPICS_STRING_LENGTH,  write_string,   read_string },
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
void create_persistent_waveform(
    const char *name, enum PERSISTENCE_TYPES type, size_t max_length)
{
    const struct persistent_action *action = &persistent_actions[type];
    struct persistent_variable *persistence =
        malloc(sizeof(struct persistent_variable) + max_length * action->size);
    persistence->action = action;
    persistence->max_length = max_length;
    persistence->length = 0;

    LOCK();
    hash_table_insert(variable_table, name, persistence);
    UNLOCK();
}


void create_persistent_variable(const char *name, enum PERSISTENCE_TYPES type)
{
    create_persistent_waveform(name, type, 1);
}


static struct persistent_variable *lookup_persistence(const char *name)
{
    struct persistent_variable *persistence =
        hash_table_lookup(variable_table, name);
    TEST_NULL_(persistence, "Persistent variable %s not found", name);
    return persistence;
}


/* Updates variable from value stored on disk. */
bool read_persistent_waveform(const char *name, void *variable, size_t *length)
{
    LOCK();
    struct persistent_variable *persistence = lookup_persistence(name);
    bool ok = persistence != NULL  &&  persistence->length > 0;
    if (ok)
    {
        memcpy(variable, persistence->variable,
            persistence->length * persistence->action->size);
        *length = persistence->length;
    }
    UNLOCK();
    return ok;
}

bool read_persistent_variable(const char *name, void *variable)
{
    size_t length;
    return read_persistent_waveform(name, variable, &length);
}


/* Writes value to persistent variable. */
void write_persistent_waveform(
    const char *name, const void *value, size_t length)
{
    LOCK();
    struct persistent_variable *persistence = lookup_persistence(name);
    if (persistence != NULL)
    {
        /* Don't force a write of the persistence file if nothing has actually
         * changed. */
        size_t size = length * persistence->action->size;
        persistence_dirty =
            persistence_dirty  ||
            persistence->length != length  ||
            memcmp(persistence->variable, value, size);

        persistence->length = length;
        memcpy(persistence->variable, value, size);
    }
    UNLOCK();
}

void write_persistent_variable(const char *name, const void *value)
{
    write_persistent_waveform(name, value, 1);
}


static bool parse_value(
    const char **line, struct persistent_variable *persistence)
{
    void *variable = persistence->variable;
    size_t size = persistence->action->size;
    size_t length = 0;
    bool ok = true;
    for (; ok  &&  **line != '\0'  &&  length < persistence->max_length;
         length ++)
    {
        ok = persistence->action->read(line, variable);
        variable += size;
    }
    persistence->length = ok ? length : 0;
    return ok  &&  TEST_OK_(**line == '\0', "Unexpected extra characters");
}


/* Parse a line of the form <key>=<value> using the parsing method associated
 * with <key>, or fail if <key> not known. */
static bool parse_line(char *line, int line_number)
{
    char *equal;
    const char *value;
    struct persistent_variable *persistence;
    bool ok =
        TEST_NULL_(equal = strchr(line, '='), "Missing =")  &&
        DO_(*equal++ = '\0'; value = equal)  &&
        TEST_NULL_(persistence = hash_table_lookup(variable_table, line),
            "Persistence key \"%s\" not found", line)  &&
        parse_value(&value, persistence);
    /* Report location of error. */
    if (!ok)
        print_error("Parse error on line %d of state file %s",
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

    char line[10240];
    int line_number = 0;

    bool ok = true;
    while (fgets(line, sizeof(line), in))
    {
        line_number += 1;

        /* Skip lines beginning with # and ignore blank lines.  All other lines
         * are processed. */
        if (line[0] != '#')
        {
            int len = strlen(line);
            if (TEST_OK_(line[len - 1] == '\n',
                    "Line %d truncated?", line_number))
                line[len - 1] = '\0';

            if (*line != '\0')
                /* Allow parsing of individual lines to fail, but accumulate
                 * overall error code.  This shouldn't actually ever happen, of
                 * course. */
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


static void write_line(
    FILE *out, const char *name, const struct persistent_variable *persistence)
{
    const void *variable = persistence->variable;
    size_t size = persistence->action->size;
    fprintf(out, "%s=", name);
    for (size_t i = 0; i < persistence->length; i ++)
    {
        if (i != 0)
            fputc(' ', out);
        persistence->action->write(out, variable);
        variable += size;
    }
    fputc('\n', out);
}


/* Writes persistent state to given file. */
static bool write_persistent_state(const char *filename)
{
    FILE *out;
    if (!TEST_NULL(out = fopen(filename, "w")))
        return false;

    /* Start with a timestamp log. */
    char out_buffer[40];
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
        if (persistence->length > 0)
            write_line(out, name, persistence);
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
