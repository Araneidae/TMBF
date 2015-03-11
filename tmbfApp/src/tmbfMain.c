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
#include <fcntl.h>
#include <execinfo.h>

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
#include "ddr_epics.h"
#include "fir.h"
#include "bunch_select.h"
#include "sequencer.h"
#include "triggers.h"
#include "sensors.h"
#include "detector.h"
#include "tune.h"
#include "tune_peaks.h"
#include "tune_follow.h"
#include "pvlogging.h"
#include "persistence.h"


/* External declaration of DBD binding. */
extern int tmbf_registerRecordDeviceDriver(struct dbBase *pdbbase);


/* Name of PID file so that we can remove it on successful termination. */
static const char *PidFileName = NULL;

/* If the IOC shell is not running then this semaphore is used to request IOC
 * shutdown. */
static sem_t ShutdownSemaphore;

/* Persistence state management. */
static const char *persistence_state_file = NULL;
static int persistence_interval = 1200;             // 20 minute update interval

/* Device name configured on startup. */
static const char *device_name = "(unknown)";

/* File to read hardware configuration settings. */
static const char *hardware_config_file = NULL;

/* Limits length of waveforms logged on output. */
static int max_log_array_length = 10000;


#define TEST_EPICS(command) \
    ( { \
        int __status__ = (command); \
        if (__status__ != 0) \
            printf(#command " (%s, %d): %s (%d)\n", \
                __FILE__, __LINE__, ca_message(__status__), __status__); \
        __status__ == 0; \
    } )




static bool Interactive = true;



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
        /* Remember PID filename so we can remove it on exit. */
        PidFileName = FileName;
        return true;
    }
}


static bool ProcessOptions(int *argc, char ** *argv)
{
    bool Ok = true;
    while (Ok)
    {
        switch (getopt(*argc, *argv, "+np:s:i:l:d:H:"))
        {
            case 'n':   Interactive = false;                    break;
            case 'p':   Ok = WritePid(optarg);                  break;
            case 's':   persistence_state_file = optarg;        break;
            case 'i':   persistence_interval = atoi(optarg);    break;
            case 'l':   max_log_array_length = atoi(optarg);    break;
            case 'd':   device_name = optarg;                   break;
            case 'H':   hardware_config_file = optarg;          break;
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
    database_add_macro("DEVICE", "%s", device_name);
    database_add_macro("FIR_LENGTH", "%d", hw_read_fir_length());
    return database_load_file(database);
}

static bool initialise_epics(void)
{
    set_prompt();
    return
        start_caRepeater()  &&
        hook_pv_logging("db/access.acf", max_log_array_length)  &&
        TEST_EPICS(dbLoadDatabase("dbd/tmbf.dbd", NULL, NULL))  &&
        TEST_EPICS(tmbf_registerRecordDeviceDriver(pdbbase))  &&
        load_database("db/tmbf.db")  &&
        TEST_EPICS(iocInit());
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Reboot and Restart Support */

/* This is a fairly dirty system for ensuring that we can restart either EPICS
 * or the entire Libera even when fairly low on memory. */

static void detach_process(const char *process, const char *const argv[])
{
    /* Annoyingly we need to fork a new process because the first thing that
     * `/etc/init.d/epics restart` does is to kill the old PID, so we need a new
     * one.  We need to use vfork() here because if we are low on memory then
     * fork() will fail. */
    pid_t pid;
    if (TEST_IO(pid = vfork()))
    {
        if (pid == 0)
        {
            /* We're not obeying the rules for forkv(), but we should get away
             * with it.  All the calls we're making are system calls which
             * should only affect the new process, and the old one is going to
             * be gone soon anyway. */

            /* Ensure that none of our open files will be inherited.  It's safer
             * to do this than to close them. */
            for (int i = 3; i < sysconf(_SC_OPEN_MAX); i ++)
                fcntl(i, F_SETFD, FD_CLOEXEC);

            /* Enable all signals. */
            sigset_t all_signals;
            sigfillset(&all_signals);
            sigprocmask(SIG_UNBLOCK, &all_signals, NULL);

            /* Finally we can actually exec the new process... */
            char *envp[] = { NULL };
            execve(process, REINTERPRET_CAST(char **, argv), envp);
        }
    }
}


static void do_reboot(void)
{
    printf("TMBF reboot requested\n");
    const char *args[] = { "/sbin/reboot", NULL };
    detach_process(args[0], args);
}


static void do_restart(void)
{
    printf("EPICS IOC restart requested\n");
    const char *args[] = { "/etc/init.d/epics", "restart", NULL };
    detach_process(args[0], args);
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/* Driver version. */
static EPICS_STRING version_string = { TMBF_VERSION };


/* Initialises the various TMBF subsystems. */
static bool initialise_subsystems(void)
{
    PUBLISH_READ_VAR(stringin, "VERSION", version_string);
    PUBLISH_READER(ulongin, "FPGAVER", hw_read_version);
    PUBLISH_WRITER(bo, "LOOPBACK", hw_write_loopback_enable);
    PUBLISH_WRITER(bo, "COMPENSATE", hw_write_compensate_disable);
    PUBLISH_ACTION("RESTART", do_restart);
    PUBLISH_ACTION("REBOOT", do_reboot);

    return
        initialise_ddr_epics()  &&
        initialise_adc_dac()  &&
        initialise_fir()  &&
        initialise_bunch_select()  &&
        initialise_sequencer()  &&
        initialise_triggers()  &&
        initialise_sensors()  &&
        initialise_detector()  &&
        initialise_tune()  &&
        initialise_tune_peaks()  &&
        initialise_tune_follow();
}


int main(int argc,char *argv[])
{
    bool Ok =
        ProcessOptions(&argc, &argv) &&
        TEST_OK_(argc == 0, "Unexpected extra arguments")  &&

        initialise_persistent_state(
            persistence_state_file, persistence_interval)  &&
        initialise_hardware(hardware_config_file, FPGA_VERSION)  &&
        InitialiseSignals()  &&

        initialise_epics_device()  &&
        initialise_epics_extra()  &&

        initialise_subsystems()  &&

        DO(load_persistent_state())  &&
        initialise_epics();

    if (Ok)
    {
        printf("EPICS TMBF Driver, Version %s.  Built: %s.\n",
            TMBF_VERSION, BUILD_DATE_TIME);
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

    if (PidFileName != NULL)
        IGNORE(TEST_IO(unlink(PidFileName)));

    return Ok ? 0 : 1;
}
