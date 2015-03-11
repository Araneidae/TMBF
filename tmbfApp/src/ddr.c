/* Interface to fast DDR RAM used for capturing full data rate waveforms.  The
 * buffer contains 64M samples, half pre-trigger, half post-trigger.  Capture
 * is initiated by arming (either internally or externally) and completed by
 * triggering -- after triggering capture continues for 32M samples (64ms) and
 * then the data can be read out. */

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <pthread.h>

#include "error.h"
#include "hardware.h"

#include "ddr.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Register interface to FPGA. */

#define PAGE_SIZE               0x00001000      // 4K

#define HISTORY_IOBASE          0x14018000
#define HISTORY_FIFO_IOBASE     0x14019000


/* To be mapped at 0x14018000.  The following registers provide access to and
 * control of the DDR history buffer. */
struct history_buffer_interface {
    uint32_t post_filtering;            // 00: Enable data filtering on readout
    uint32_t start_address;             // 04:
    uint32_t address_step;              // 08: Interval between samples
    uint32_t transfer_size;             // 0C: Size of single transfer
    uint32_t transfer_count;            // 10: Triggers data fetch
    uint32_t transfer_status;           // 14: Status and fifo fill level
    uint32_t sdram_operation_request;   // 18:
    uint32_t sdram_control;             // 1C: Enable/disable
};

/* To be mapped at 0x14019000. */
struct history_buffer_fifo {
    uint32_t fifo;                      // 00: Read to empty fifo
};


/* Pointers into FPGA control registers. */
static volatile struct history_buffer_interface *history_buffer;
static volatile struct history_buffer_fifo *fifo;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* DDR buffer readout. */

/* Need to interlock access to the DDR hardware. */
static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;

#define LOCK()      pthread_mutex_lock(&lock);
#define UNLOCK()    pthread_mutex_unlock(&lock);


#define PURGE_FIFO_LIMIT    65536

/* Ensures data transfer FIFO is empty on startup. */
static void purge_read_buffer(void)
{
    /* Ensure DDR isn't enabled when we start as this will mess up our ability
     * to read the FIFO. */
    hw_write_ddr_disable();

    unsigned int fifo_size = history_buffer->transfer_status & 0x7FF;
    if (fifo_size > 0)
    {
        printf("Purging read FIFO: %u\n", fifo_size);
        unsigned int total_read = 0;
        do {
            for (unsigned int i = 0; i < fifo_size; i ++)
            {
                fifo->fifo;
                fifo->fifo;
            }
            total_read += fifo_size;
            fifo_size = history_buffer->transfer_status & 0x7FF;
        } while (fifo_size > 0  &&  total_read < PURGE_FIFO_LIMIT);
        printf("%u atoms discarded\n", total_read);
    }
}


#define MAX_FIFO_SIZE       512

#define MAX_FIFO_WAITS      20      // How long to wait for FIFO ready

static bool start_buffer_transfer(ssize_t offset, size_t interval, size_t count)
{
    bool armed, busy, iq_select;
    hw_read_ddr_status(&armed, &busy, &iq_select);
    if (!TEST_OK_(!armed && !busy,
            "Cannot read from DDR: busy capturing data"))
        return false;

    /* In IQ mode we read from the start of the buffer, otherwise we can read
     * the DDR offset. */
    uint32_t ddr_trigger_offset = iq_select ? 0 : hw_read_ddr_offset();

    int ddr_delay = hw_read_ddr_delay();
    history_buffer->post_filtering = 0;
    history_buffer->start_address = (uint32_t)
        (offset + ddr_delay + (int) ddr_trigger_offset) & 0xFFFFFF;
    history_buffer->address_step = interval;
    history_buffer->transfer_size = 1;
    /* Writing to this register initiates transfer.  count is in "atoms". */
    history_buffer->transfer_count = count;

    /* Wait for FIFO to become ready. */
    int waits = 0;
    while (waits < MAX_FIFO_WAITS  &&
           (history_buffer->transfer_status & 0x7FF) == 0)
        waits += 1;
    return TEST_OK_(waits < MAX_FIFO_WAITS, "Gave up waiting for DDR FIFO");
}


static inline int16_t extract_sample(uint32_t atom)
{
    return (int16_t) atom;
}


/* Fills buffer (which should be MAX_FIFO_SIZE atoms long) from the DDR fifo,
 * returns the number of atoms read. */
static size_t get_buffer_fifo(int16_t *buffer, size_t count)
{
    unsigned int fifo_size = history_buffer->transfer_status & 0x7FF;
    if (fifo_size > MAX_FIFO_SIZE)
        fifo_size = MAX_FIFO_SIZE;
    if (!TEST_OK_(fifo_size <= count,
            "DDR FIFO overrun: %u/%zu", fifo_size, count))
        fifo_size = count;
    for (unsigned int i = 0; i < fifo_size; i ++)
    {
        /* Take two words at a time from the fifo (one "atom") and convert
         * into four 16-bit samples.  Also remove the sign displacement at
         * this point. */
        uint32_t half_atom = fifo->fifo;
        *buffer++ = extract_sample(half_atom);
        *buffer++ = extract_sample(half_atom >> 16);
        half_atom = fifo->fifo;
        *buffer++ = extract_sample(half_atom);
        *buffer++ = extract_sample(half_atom >> 16);
    }
    return fifo_size;
}


/* For loop helper to capture some of the repeated pattern below.  Reads block
 * of atoms into the given buffer and tracks count to ensure that we don't
 * overrun the buffer in error. */
#define FOR_FIFO_ATOMS(atoms_read, count, buffer) \
    for (size_t atoms_read; \
        atoms_read = get_buffer_fifo(buffer, count), \
            atoms_read > 0  &&  count >= atoms_read; \
        count -= atoms_read) \


/* Reads the given number of complete turns from the trigger point. */
bool read_ddr_turns(ssize_t start, size_t turns, int16_t *result)
{
    size_t count = turns * ATOMS_PER_TURN;
    LOCK();
    bool ok =
        start_buffer_transfer(start * ATOMS_PER_TURN, 1, count)  &&

        DO(FOR_FIFO_ATOMS(atoms_read, count, result)
            result += atoms_read * SAMPLES_PER_ATOM)  &&
        TEST_OK_(count == 0, "Incomplete DDR buffer read: %zu", count);
    UNLOCK();
    return ok;
}


/* Reads the given number of a single bunch. */
bool read_ddr_bunch(ssize_t start, size_t bunch, size_t turns, int16_t *result)
{
    size_t offset = bunch % SAMPLES_PER_ATOM;
    int16_t buffer[MAX_FIFO_SIZE * SAMPLES_PER_ATOM];

    LOCK();
    bool ok =
        start_buffer_transfer(
            start * ATOMS_PER_TURN + (ssize_t) bunch / SAMPLES_PER_ATOM,
            ATOMS_PER_TURN, turns)  &&

        DO(FOR_FIFO_ATOMS(atoms_read, turns, buffer)
            for (size_t i = 0; i < atoms_read; i ++)
                *result++ = buffer[i * SAMPLES_PER_ATOM + offset])  &&
        TEST_OK_(turns == 0, "Incomplete DDR buffer read: %zu", turns);
    UNLOCK();
    return ok;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* DDR buffer initialisation. */

bool initialise_ddr(void)
{
    int mem;
    return
        TEST_IO(mem = open("/dev/mem", O_RDWR | O_SYNC))  &&
        TEST_IO(history_buffer = mmap(
            0, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
            mem, HISTORY_IOBASE))  &&
        TEST_IO(fifo = mmap(
            0, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
            mem, HISTORY_FIFO_IOBASE))  &&
        DO(purge_read_buffer());
}
