/* generic functions take variant type */

struct Variant;

typedef void (*GenericFunction)(struct Variant *v);
typedef void (*GenericHook)(int OnEntry, struct Variant *v);

typedef struct Variant {
    int type;
    unsigned length;
    char * name;
    double * buffer;
    unsigned index;
    void * usr;         // User data, passed back to caller
    GenericFunction f;  // Function to be called to process record
} Variant;

/* Registers a process method for the generic device record with the given
 * name.  Callback is called with v.usr set to Context each time the
 * associated record is processed. */
void GenericRegister(char * Name, GenericFunction Callback, void * Context);

/* Can be used to post an EPICS event to trigger record processing. */
void GenericEvent(int);

/* Registers a hook function which is called before and after every
 * registered generic callback routine.  This is designed to be used to
 * implement global locking by passing GenericGlobalLock as the hook
 * function. */
void RegisterGenericHook(GenericHook hook);
void GenericGlobalLock(int pass, Variant *value);
