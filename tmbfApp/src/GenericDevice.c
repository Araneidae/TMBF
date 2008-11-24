#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

/* EPICS */
#include "dbDefs.h"
#include "dbAccess.h"
#include "dbScan.h"
#include "recGbl.h"
#include "recSup.h"
#include "devSup.h"
#include "link.h"
#include "aiRecord.h"
#include "aoRecord.h"
#include "waveformRecord.h"
#include "mbboRecord.h"
#include "epicsExport.h"
#include "gpHash.h"
#include "errlog.h"

#include "GenericDevice.h"


/* == Generic Functions == */

static const int menuFtypeENUM = 9;
static const int menuFtypeDOUBLE = 8;
static const int TABLESIZE = 1024;
static const int NO_CONVERSION = 2;

static void * hash = NULL;
static GenericHook HookFunction = NULL;
static pthread_mutex_t global_lock = PTHREAD_MUTEX_INITIALIZER;


typedef struct {
    unsigned index;
    Variant * v;
} Closure;

static Closure * closure_new()
{
    return (Closure *)calloc(1, sizeof(Closure));
}

static Variant * variant_new()
{
    return (Variant *)calloc(1, sizeof(Variant));
}

static char * vx_strdup(char * src)
{
    char * dest = malloc(1 + strlen(src));
    strcpy(dest, src);
    return dest;
}

static Closure * store_lookup(char * name)
{
    GPHENTRY * entry = 0;
    char * key = vx_strdup(name);
    char * paren = strstr(key, "(");
    int index = 0;
    Closure * c = 0;
    if(paren != NULL)
    {
        index = atoi(paren + 1);
        *(paren) = '\0';
    }
    
    if(hash != 0 && (entry = gphFind(hash, key, 0)))
    {
        /* need one closure for each record */
        /* one prototype variant for each registered function */
        Variant * v = (Variant *)(entry->userPvt);
        c = closure_new();
        v->name = key;
        c->v = v;
        c->index = index;
    }
    else 
        printf("No handler found for generic record \"%s\"\n", name);
    return c;
}


void GenericEvent(int event)
{
    post_event(event);
}

void GenericRegister(char * name, GenericFunction f, void * udata)
{
    GPHENTRY * entry = 0;
    if(hash == 0) 
        gphInitPvt(&hash, TABLESIZE);
    entry = gphAdd(hash, name, 0);
    if(entry == 0) 
        printf("GenericRegister: Failed to add %s - duplicate name?\n", name);
    else
    {
        Variant * v = variant_new();
        v->usr = udata;
        v->f = f;
        entry->userPvt = (void *)v;
    }
}

void RegisterGenericHook(GenericHook Hook)
{
    if (HookFunction)
        printf("Warning: previously registered generic hook overwritten\n");
    HookFunction = Hook;
}

void GenericGlobalLock(int OnEntry, Variant *v)
{
    if (OnEntry)
        pthread_mutex_lock(&global_lock);
    else
        pthread_mutex_unlock(&global_lock);
}

void GenericCall(Closure * c, unsigned length, double * buffer, int type)
{
    if (c == 0)
        /* Silently fail: otherwise we flood the log with useless messages,
         * and we've already said our bit at startup when the handler
         * couldn't be found. */
        return;
    
    Variant v = * c->v;
    v.index = c->index;
    v.type = type;
    v.length = length;
    v.buffer = buffer;
    
    if (HookFunction)  HookFunction(1, &v);
    v.f(&v);
    if (HookFunction)  HookFunction(0, &v);
}



/* == Device Support == */

static Closure * map_address(DBLINK link, void * record)
{
    if(link.type != INST_IO)
    {
        recGblRecordError(S_db_badField, record, "not INST_IO");
        /* address is the wrong type - must be of the form @string */
        return 0;
    }
    /* need to split into name[index] */
    return store_lookup(((struct instio *)&(link.value))->string);
}

#define INIT_DPVT(record, in_out) \
    record->dpvt = map_address(record->in_out, record); \
    if(record->dpvt == NULL) \
        return S_db_badField

// static long init(void * pAfter)
// {
//     return 0;
// }

static long init_ai(aiRecord * record)
{
    INIT_DPVT(record, inp);
    return NO_CONVERSION;
}

static long init_ao(aoRecord * record)
{
    INIT_DPVT(record, out);
    record->udf = FALSE;
    return NO_CONVERSION;
}

static long init_waveform(waveformRecord * record)
{
    INIT_DPVT(record, inp);
    return 0;
}

static long init_mbbo(mbboRecord * record)
{
    INIT_DPVT(record, out);
    record->udf = FALSE;
    return NO_CONVERSION;
}


static long read_ai(aiRecord * record)
{
    GenericCall(record->dpvt, 1, &record->val, menuFtypeDOUBLE);
    record->udf = FALSE;
    return NO_CONVERSION;
}

static long write_ao(aoRecord * record)
{
    GenericCall(record->dpvt, 1, &record->val, menuFtypeDOUBLE);
    record->udf = FALSE;
    return 0;
}

static long read_waveform(waveformRecord * record)
{
    /* cast to read function -> nice! */
    GenericCall(record->dpvt, record->nelm, record->bptr, record->ftvl);
    record->udf = FALSE;
    if(record->bptr == NULL) 
        record->nord = 0;
    else
        record->nord = record->nelm;
    return 0;
}

static long write_mbbo(mbboRecord * record)
{
    GenericCall(record->dpvt, 1, (double *)&record->val, menuFtypeENUM);
    record->udf = FALSE;
    return 0;
}



/* Device Support Interface */

typedef struct {
    long      number;
    DEVSUPFUN report;
    DEVSUPFUN init;
    DEVSUPFUN init_record;
    DEVSUPFUN get_ioint_info;
    DEVSUPFUN read;
    DEVSUPFUN linconv;
} devSupport;

#define EXPORT_DEVICE(name, dev_init, dev_rw) \
    devSupport dev##name##Generic = { \
        6, NULL, NULL, dev_init, NULL, dev_rw, NULL \
    }; \
    epicsExportAddress(dset, dev##name##Generic)

//        6, NULL, init, dev_init, NULL, dev_rw, NULL 

EXPORT_DEVICE(Ai,       init_ai,        read_ai);
EXPORT_DEVICE(Ao,       init_ao,        write_ao);
EXPORT_DEVICE(Waveform, init_waveform,  read_waveform);
EXPORT_DEVICE(Mbbo,     init_mbbo,      write_mbbo);
