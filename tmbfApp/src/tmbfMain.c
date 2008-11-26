#include <stddef.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "epicsThread.h"
#include "iocsh.h"

#include <stdbool.h>

#include "device.h"


static bool Interactive = true;



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



int main(int argc,char *argv[])
{
    if (!ProcessOptions(&argc, &argv))
        return 1;
    
    GenericInit();
    for (int i = 0; i < argc; i ++)
    {
        iocsh(argv[i]);
        epicsThreadSleep(.2);
    }

    StartupMessage();
    if (Interactive)
        iocsh(NULL);
    else
    {
        printf("Running in non-interactive mode.  "
            "Kill process %d to close.\n", getpid());
        fflush(stdout);
        /* Now just block until we are killed. */
        while (true)
            sleep((unsigned int)-1);
    }
    return 0;
}
