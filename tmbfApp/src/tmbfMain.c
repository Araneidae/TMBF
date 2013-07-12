#include <stdbool.h>
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <semaphore.h>
#include <signal.h>

#include <epicsThread.h>
#include <iocsh.h>
#include <caerr.h>
#include <envDefs.h>
#include <dbAccess.h>
#include <iocInit.h>

#include "error.h"
#include "hardware.h"
#include "epics_device.h"
#include "epics_extra.h"
#include "adc_dac.h"
#include "tune.h"
#include "device.h"
#include "ddr_epics.h"
#include "fir.h"
#include "bunch_select.h"
#include "sequencer.h"
#include "triggers.h"
#include "sensors.h"
#include "detector.h"
#include "pvlogging.h"
#include "persistence.h"


/* External declaration of caRepeater thread.  This should really be published
 * by a standard EPICS header file. */
extern void caRepeaterThread(void *);
/* External declaration of DBD binding. */
extern int tmbf_registerRecordDeviceDriver(struct dbBase *pdbbase);


/* If the IOC shell is not running then this semaphore is used to request IOC
 * shutdown. */
static sem_t ShutdownSemaphore;

/* Persistence state management. */
static const char *persistence_state_file = NULL;
static int persistence_interval = 1200;             // 20 minute update interval

/* Device name configured on startup. */
const char *device_name = "(unknown)";


#define TEST_EPICS(command) \
    ( { \
        int __status__ = (command); \
        if (__status__ != 0) \
            printf(#command " (%s, %d): %s (%d)\n", \
                __FILE__, __LINE__, ca_message(__status__), __status__); \
        __status__ == 0; \
    } )




static bool Interactive = true;



/* Routine for printing an error message complete with associated file name
 * and line number. */
void print_error(const char * Message, ...)
{
    /* Large enough not to really worry about overflow.  If we do generate a
     * silly message that's too big, then that's just too bad. */
    const int MESSAGE_LENGTH = 512;
    int Error = errno;
    char ErrorMessage[MESSAGE_LENGTH];

    va_list args;
    va_start(args, Message);
    int Count = vsnprintf(ErrorMessage, MESSAGE_LENGTH, Message, args);

    if (Error != 0)
    {
        /* This is very annoying: strerror() is not not necessarily thread
         * safe ... but not for any compelling reason, see:
         *  http://sources.redhat.com/ml/glibc-bugs/2005-11/msg00101.html
         * and the rather unhelpful reply:
         *  http://sources.redhat.com/ml/glibc-bugs/2005-11/msg00108.html
         *
         * On the other hand, the recommended routine strerror_r() is
         * inconsistently defined -- depending on the precise library and its
         * configuration, it returns either an int or a char*.  Oh dear.
         *
         * Ah well.  We go with the GNU definition, so here is a buffer to
         * maybe use for the message. */
        char StrError[256];
        snprintf(ErrorMessage + Count, MESSAGE_LENGTH - Count,
            ": (%d) %s", Error, strerror_r(Error, StrError, sizeof(StrError)));
    }
    fprintf(stderr, "%s\n", ErrorMessage);
}


void panic_error(const char *filename, int line)
{
    print_error("Unrecoverable error at %s, line %d", filename, line);
    fflush(stderr);
    fflush(stdout);
    _exit(255);
}



/* This routine spawns a caRepeater thread, as recommended by Andrew Johnson
 * (private communication, 2006/12/04).  This means that this IOC has no
 * external EPICS dependencies (otherwise the caRepeater application needs to be
 * run). */
static bool StartCaRepeater(void)
{
    return TEST_NULL(epicsThreadCreate(
        "CAC-repeater", epicsThreadPriorityLow,
        epicsThreadGetStackSize(epicsThreadStackMedium),
        caRepeaterThread, 0));
}


static void at_exit(int sig)
{
    sem_post(&ShutdownSemaphore);
    if (Interactive)
        /* If the IOC shell is running we have a problem.  Closing stdin
         * *sometimes* works... */
        close(0);
}

/* Set up basic signal handling environment.  We configure four shutdown
 * signals (HUP, INT, QUIT and TERM) to call AtExit(). */
static bool InitialiseSignals(void)
{
    struct sigaction action = {
        .sa_handler = at_exit, .sa_flags = 0 };
    return
        /* Block all signals during AtExit() signal processing. */
        TEST_IO(sigfillset(&action.sa_mask))  &&
        /* Catch all the usual culprits: HUP, INT, QUIT and TERM. */
        TEST_IO(sigaction(SIGHUP,  &action, NULL))  &&
        TEST_IO(sigaction(SIGINT,  &action, NULL))  &&
        TEST_IO(sigaction(SIGQUIT, &action, NULL))  &&
        TEST_IO(sigaction(SIGTERM, &action, NULL));
}


/* Write the PID of this process to the given file. */

static bool WritePid(const char * FileName)
{
    FILE * output = fopen(FileName, "w");
    if (output == NULL)
    {
        perror("Can't open PID file");
        return false;
    }
    else
    {
        /* Lazy error checking here.  Really should check that there aren't
         * any errors in any of the following. */
        fprintf(output, "%d", getpid());
        fclose(output);
//         /* Remember PID filename so we can remove it on exit. */
//         PidFileName = FileName;
        return true;
    }
}


static bool ProcessOptions(int *argc, char ** *argv)
{
    bool Ok = true;
    while (Ok)
    {
        switch (getopt(*argc, *argv, "+np:s:i:d:"))
        {
            case 'n':   Interactive = false;                    break;
            case 'p':   Ok = WritePid(optarg);                  break;
            case 's':   persistence_state_file = optarg;        break;
            case 'i':   persistence_interval = atoi(optarg);    break;
            case 'd':   device_name = optarg;                   break;
            default:
                printf("Sorry, didn't understand\n");
                return false;
            case -1:
                /* All options successfully read.  Consume them and return
                 * success. */
                *argc -= optind;
                *argv += optind;
                return true;
        }
    }
    return false;
}


/* Prints interactive startup message as recommended by GPL. */

static void StartupMessage(void)
{
    printf("EPICS TMBF Driver, Version %s.  Built: %s.\n",
        TMBF_VERSION, BUILD_DATE_TIME);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Core EPICS startup (st.cmd equivalent processing). */

/* Configures the IOC prompt to show the EPICS device name. */
static void set_prompt(void)
{
    char prompt[256];
    snprintf(prompt, sizeof(prompt), "%s> ", device_name);
    epicsEnvSet("IOCSH_PS1", prompt);
}

static bool load_database(const char *database)
{
    char macros[1024];
    snprintf(macros, sizeof(macros), "DEVICE=%s,FIR_LENGTH=%d",
        device_name, hw_read_fir_length());
    return TEST_EPICS(dbLoadRecords(database, macros));
}

static bool initialise_epics(void)
{
    set_prompt();
    return
        TEST_EPICS(dbLoadDatabase("dbd/tmbf.dbd", NULL, NULL))  &&
        TEST_EPICS(tmbf_registerRecordDeviceDriver(pdbbase))  &&
        load_database("db/tmbf.db")  &&
        TEST_EPICS(iocInit());
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

int main(int argc,char *argv[])
{
    bool Ok =
        ProcessOptions(&argc, &argv) &&
        TEST_OK_(argc == 0, "Unexpected extra arguments")  &&
        initialise_persistent_state(
            persistence_state_file, persistence_interval)  &&
        initialise_hardware()  &&
        StartCaRepeater()  &&
        HookLogging()  &&
        InitialiseSignals()  &&
        initialise_epics_device()  &&
        initialise_epics_extra()  &&
//         InitialiseTune()  &&
//         GenericInit()  &&
        initialise_ddr_epics()  &&
        initialise_adc_dac()  &&
        initialise_fir()  &&
        initialise_bunch_select()  &&
        initialise_sequencer()  &&
        initialise_triggers()  &&
        initialise_sensors()  &&
        initialise_detector()  &&
        DO_(load_persistent_state())  &&
        initialise_epics();

    if (Ok)
    {
        StartupMessage();
        if (Interactive)
            Ok = TEST_EPICS(iocsh(NULL));
        else
        {
            printf("Running in non-interactive mode.  "
                "Kill process %d to close.\n", getpid());
            fflush(stdout);
            /* Now just block until we are killed. */
            sem_wait(&ShutdownSemaphore);
        }

        terminate_persistent_state();
    }
    return Ok ? 0 : 1;
}
