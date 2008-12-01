#include <stdbool.h>
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <epicsThread.h>
#include <iocsh.h>
#include <caerr.h>

#include "test_error.h"
#include "hardware.h"
#include "device.h"


#define TEST_EPICS(command, args...) \
    ( { \
        int __status__ = (command)(args); \
        if (__status__ != 0) \
            printf(#command "(" #args ") (%s, %d): %s (%d)\n", \
                __FILE__, __LINE__, ca_message(__status__), __status__); \
        __status__ == 0; \
    } )




static bool Interactive = true;



/* Routine for printing an error message complete with associated file name
 * and line number. */

void print_error(const char * Message, const char * FileName, int LineNumber)
{
    /* Large enough not to really worry about overflow.  If we do generate a
     * silly message that's too big, then that's just too bad. */
    const int MESSAGE_LENGTH = 512;
    int Error = errno;
    char ErrorMessage[MESSAGE_LENGTH];
    
    int Count = snprintf(ErrorMessage, MESSAGE_LENGTH,
        "%s (%s, %d)", Message, FileName, LineNumber);
    if (errno != 0)
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
    printf("%s\n", ErrorMessage);
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
        switch (getopt(*argc, *argv, "+np:"))
        {
            case 'n':   Interactive = false;                break;
            case 'p':   Ok = WritePid(optarg);              break;
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

static void StartupMessage()
{
    printf(
"\n"
"EPICS TMBF Driver, Version %s.  Built: %s.\n"
"Copyright (C) 2007-2008\n"
"Isa Uzun, James Rowland, Michael Abbott, Diamond Light Source.\n"
"This program comes with ABSOLUTELY NO WARRANTY.  This is free software,\n"
"and you are welcome to redistribute it under certain conditions.\n"
"For details see the GPL or the attached file COPYING.\n",
        TMBF_VERSION, BUILD_DATE_TIME);
}


static const iocshFuncDef iocsh_dump_registers_def = { "d", 0, NULL };
static void iocsh_dump_registers(const iocshArgBuf *args)
{
    dump_registers();
}

int main(int argc,char *argv[])
{
    bool Ok = 
        ProcessOptions(&argc, &argv) &&
        InitialiseHardware()  &&
        GenericInit();
    for (int i = 0; Ok && i < argc; i ++)
        Ok = TEST_EPICS(iocsh, argv[i]);
    if (Ok)
    {
        iocshRegister(&iocsh_dump_registers_def, iocsh_dump_registers);
        StartupMessage();
        if (Interactive)
            Ok = TEST_EPICS(iocsh, NULL);
        else
        {
            printf("Running in non-interactive mode.  "
                "Kill process %d to close.\n", getpid());
            fflush(stdout);
            /* Now just block until we are killed. */
            while (true)
                sleep((unsigned int)-1);
        }
    }
    return Ok ? 0 : 1;
}
