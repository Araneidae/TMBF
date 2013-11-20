/* Simple interface to IOC caput logging. */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define db_accessHFORdb_accessC     // Needed to get correct DBF_ values
#include <dbAccess.h>
#include <dbFldTypes.h>
#include <db_access.h>
#include <asTrapWrite.h>
#include <asDbLib.h>

#include "pvlogging.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                            IOC PV put logging                             */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* Used to trim arrays of excessive length when logging. */
static int max_array_length;


/* Alas dbGetField is rather rubbish at formatting floating point numbers, so we
 * do that ourselves, but the rest formats ok. */
static void FormatField(dbAddr *dbaddr, dbr_string_t *value)
{
#define FORMAT(type, format) \
    do { \
        type *raw = (type *) dbaddr->pfield; \
        for (int i = 0; i < length; i ++) \
            snprintf(value[i], sizeof(dbr_string_t), format, raw[i]); \
    } while (0)

    long length = dbaddr->no_elements;
    switch (dbaddr->field_type)
    {
        case DBF_FLOAT:
            FORMAT(dbr_float_t, "%.7g");
            break;
        case DBF_DOUBLE:
            FORMAT(dbr_double_t, "%.15lg");
            break;
        default:
            dbGetField(dbaddr, DBR_STRING, value, NULL, &length, NULL);
            break;
    }
#undef FORMAT
}

static void PrintValue(dbr_string_t *value, int length)
{
    if (length == 1)
        printf("%s", value[0]);
    else
    {
        printf("[");
        int i = 0;
        for (; i < length  &&  i < max_array_length; i ++)
        {
            if (i > 0)  printf(", ");
            printf("%s", value[i]);
        }
        if (length > max_array_length + 1)
            printf(", ...");
        if (length > max_array_length)
            printf(", %s", value[length-1]);
        printf("]");
    }
}

static void EpicsPvPutHook(asTrapWriteMessage *pmessage, int after)
{
    dbAddr *dbaddr = (dbAddr *) pmessage->serverSpecific;
    long length = dbaddr->no_elements;
    dbr_string_t *value = (dbr_string_t *) calloc(length, sizeof(dbr_string_t));
    FormatField(dbaddr, value);

    if (after)
    {
        /* Log the message after the event. */
        dbr_string_t *old_value = (dbr_string_t *) pmessage->userPvt;
        printf("%s@%s %s.%s ",
            pmessage->userid, pmessage->hostid,
            dbaddr->precord->name, dbaddr->pfldDes->name);
        PrintValue(old_value, length);
        printf(" -> ");
        PrintValue(value, length);
        printf("\n");

        free(old_value);
        free(value);
    }
    else
        /* Just save the old value for logging after. */
        pmessage->userPvt = value;
}


bool HookLogging(int max_length)
{
    max_array_length = max_length;
    asSetFilename("db/access.acf");
    asTrapWriteRegisterListener(EpicsPvPutHook);
    return true;
}
