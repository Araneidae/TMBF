#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

#include "epics_device.h"

#define MAKE_TEST(type, format) \
    static struct epics_record *record_##type; \
    static void test_##type(TYPEOF(type) value) \
    { \
        printf("test_"#type" "format"\n", value); \
    }

#define MAKE_WRITE(type, format) \
    static void write_##type(TYPEOF(type) value) \
    { \
        printf("write_"#type" "format"\n", value); \
        WRITE_OUT_RECORD_NP(type, record_##type, value); \
    }

#define MAKE(type, format) \
    MAKE_TEST(type, format) \
    MAKE_WRITE(type, format)

MAKE(longout, "%d")
MAKE(ulongout, "%u")
MAKE(ao, "%g")
MAKE(bo, "%d")
// MAKE(stringout, "%s")
MAKE(mbbo, "%d")

static struct epics_record *record_stringout;
static void test_stringout(EPICS_STRING value)
{
    printf("test_stringout %s\n", value.s);
}

static void write_stringout(EPICS_STRING value)
{
    printf("write_stringout %s\n", value.s);
    WRITE_OUT_RECORD_NP(stringout, record_stringout, value);
}


static struct epics_record *loop1, *loop2;
static void write_loop1(int value)
{
    printf("write_loop1 %d\n", value);
    WRITE_OUT_RECORD_PP(longout, loop2, value + 1);
}
static void write_loop2(int value)
{
    printf("write_loop2 %d\n", value);
    WRITE_OUT_RECORD_PP(longout, loop1, value + 1);
}



#define DECLARE_TEST(type, name) \
    record_##type = PUBLISH_WRITER(type, "TEST:"name, test_##type); \
    PUBLISH_WRITER(type, "WRITE:"name, write_##type)

bool initialise_test(void);
bool initialise_test(void)
{
    DECLARE_TEST(longout, "LONGOUT");
    DECLARE_TEST(ulongout, "ULONGOUT");
    DECLARE_TEST(ao, "AO");
    DECLARE_TEST(bo, "BO");
    DECLARE_TEST(stringout, "STRINGOUT");
    DECLARE_TEST(mbbo, "MBBO");
    loop1 = PUBLISH_WRITER(longout, "TEST:LOOP1", write_loop1);
    loop2 = PUBLISH_WRITER(longout, "TEST:LOOP2", write_loop2);
    return true;
}
