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

#define CLK_IOBASE              0x14004000
#define HISTORY_IOBASE          0x14018000
#define HISTORY_FIFO_IOBASE     0x14019000


/* To be mapped at 0x14004000.  The following registers allow access to the
 * internally generated FPGA events. */
struct clk_interface {
    uint32_t lst_event_capture_lsb0;    // 00: LST event id and fifo
    uint32_t lst_event_capture_lsb1;    // 04: LST event timestamp
    uint32_t lst_event_capture_msb;     // 08: LST event id, advance fifo
    uint32_t lmt_event_capture_lsb0;    // 0C: LMT event id and fifo
    uint32_t lmt_event_capture_lsb1;    // 10: LMT timestamp
    uint32_t lmt_event_capture_msb;     // 14: LMT advance fifo
    uint32_t lst_event_control_lsb;     // 18: LST event control
    uint32_t lmt_event_control_lsb;     // 1C: LMT event control
    uint32_t lst_event_control_msb;     // 20: LST event control
    uint32_t lmt_event_control_msb;     // 24: LMT event control
    uint32_t interrupt_status;          // 28:
};

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
static volatile struct clk_interface *clk_interface;
static volatile struct history_buffer_interface *history_buffer;
static volatile struct history_buffer_fifo *fifo;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* DDR buffer readout. */


/* Callback function to report successful triggering. */
static void (*ddr_trigger_callback)(void);

/* Record of current trigger state. */
static uint32_t arm_timestamp;
static uint32_t trigger_timestamp;


/* Ensures data transfer FIFO is empty on startup. */
static void purge_read_buffer(void)
{
    unsigned int fifo_size = history_buffer->transfer_status & 0x7FF;
    if (fifo_size > 0)
    {
        printf("Purging read FIFO: ");
        unsigned int total_read = 0;
        do {
            for (unsigned int i = 0; i < fifo_size; i ++)
            {
                fifo->fifo;
                fifo->fifo;
            }
            total_read += fifo_size;
            fifo_size = history_buffer->transfer_status & 0x7FF;
        } while (fifo_size > 0);
        printf("%u atoms discarded\n", total_read);
    }
}


#define SAMPLES_PER_ATOM    4
#define ATOMS_PER_TURN      (SAMPLES_PER_TURN / SAMPLES_PER_ATOM)   // 234

#define MAX_FIFO_SIZE       512

#define MAX_FIFO_WAITS      20      // How long to wait for FIFO ready

static void start_buffer_transfer(ssize_t offset, size_t interval, size_t count)
{
    history_buffer->post_filtering = 0;
    history_buffer->start_address =
        (offset + trigger_timestamp - arm_timestamp) & 0xFFFFFF;
    history_buffer->address_step = interval;
    history_buffer->transfer_size = 1;
    /* Writing to this register initiates transfer.  count is in "atoms". */
    history_buffer->transfer_count = count;

    /* Wait for FIFO to become ready. */
    int waits = 0;
    while (waits < MAX_FIFO_WAITS  &&
           (history_buffer->transfer_status & 0x7FF) == 0)
        waits += 1;
    TEST_OK_(waits < MAX_FIFO_WAITS, "Gave up waiting for DDR FIFO");
}


static inline int16_t extract_sample(uint32_t atom)
{
    return (int16_t) atom - 8192;
}


/* Fills buffer (which should be MAX_FIFO_SIZE atoms long) from the DDR fifo,
 * returns the number of atoms read. */
static size_t get_buffer_fifo(int16_t *buffer, size_t count)
{
    unsigned int fifo_size = history_buffer->transfer_status & 0x7FF;
    if (fifo_size > MAX_FIFO_SIZE)
        fifo_size = MAX_FIFO_SIZE;
    if (!TEST_OK_(fifo_size <= count,
            "DDR FIFO overrun: %u/%u", fifo_size, count))
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
void read_ddr_turns(ssize_t start, size_t turns, int16_t *result)
{
    size_t count = turns * ATOMS_PER_TURN;
    start_buffer_transfer(start * ATOMS_PER_TURN, 1, count);

    FOR_FIFO_ATOMS(atoms_read, count, result)
        result += atoms_read * SAMPLES_PER_ATOM;
    TEST_OK_(count == 0, "Incomplete DDR buffer read: %u", count);
}


/* Reads the given number of a single bunch. */
void read_ddr_bunch(ssize_t start, size_t bunch, size_t turns, int16_t *result)
{
    start_buffer_transfer(
        start * ATOMS_PER_TURN + bunch / SAMPLES_PER_ATOM,
        ATOMS_PER_TURN, turns);

    size_t offset = bunch % SAMPLES_PER_ATOM;
    int16_t buffer[MAX_FIFO_SIZE * SAMPLES_PER_ATOM];
    FOR_FIFO_ATOMS(atoms_read, turns, buffer)
        for (size_t i = 0; i < atoms_read; i ++)
            *result++ = buffer[i * SAMPLES_PER_ATOM + offset];
    TEST_OK_(turns == 0, "Incomplete DDR buffer read: %u", turns);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Event handling and dispatch. */

/* We're only interested in the trigger received immediately after arming the
 * FIFO. */
static bool fifo_armed = false;


/* Ensures any residual FIFO events are purged on startup. */
static void purge_fifo(void)
{
    while ((clk_interface->lmt_event_capture_lsb0 & 0x1FF) > 0)
        clk_interface->lmt_event_capture_msb;
}


/* Consumes all events in the FIFO and updates timestamps as appropriate.
 * Returns true if a trigger has been seen. */
static bool consume_events(void)
{
    bool triggered = false;
    uint32_t lsb0;
    while (
        lsb0 = clk_interface->lmt_event_capture_lsb0,
        (lsb0 & 0x1FF) > 0)
    {
        unsigned int event = lsb0 >> 16;
        uint32_t timestamp = clk_interface->lmt_event_capture_lsb1;
        clk_interface->lmt_event_capture_msb;

        if (event & 0x2000)
        {
            /* ARM event tells us when data capture begins.  This timestamp
             * corresponds to zero offset into DDR buffer. */
            arm_timestamp = timestamp;
            fifo_armed = true;
        }
        if (fifo_armed  &&  (event & 0x4000))
        {
            /* We're only interested in the first trigger after arming; this
             * tells us from where to read data. */
            trigger_timestamp = timestamp;
            fifo_armed = false;
            triggered = true;
        }
    }
    return triggered;
}


static void *event_thread(void *context)
{
    /* Enable the history buffer enable and trigger events, ensure all other
     * events are disabled. */
    clk_interface->lmt_event_control_lsb = 0x60000000;
    clk_interface->lmt_event_control_msb = 0;

    /* Consume all events in buffer. */
    purge_fifo();

    /* Periodically wake up and dispatch events. */
    while (true)
    {
        /* Consume FIFO and report a trigger if seen. */
        if (consume_events())
        {
            /* After seeing a trigger we need to wait for capture to complete
             * before trying to read the buffer (there doesn't seem to be an
             * event for this, surprisingly enough).  We have to wait for 32M
             * samples at 500MHz, or around 64ms. */
            usleep(64000);
            ddr_trigger_callback();
        }

        /* Now sleep for a bit; 50ms seems responsive enough (20Hz). */
        usleep(50000);
    }
    return NULL;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* DDR buffer initialisation. */

bool initialise_ddr(void (*ddr_trigger)(void))
{
    ddr_trigger_callback = ddr_trigger;

    int mem;
    pthread_t thread_id;
    return
        TEST_IO(mem = open("/dev/mem", O_RDWR | O_SYNC))  &&
        TEST_IO(clk_interface = mmap(
            0, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
            mem, CLK_IOBASE))  &&
        TEST_IO(history_buffer = mmap(
            0, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
            mem, HISTORY_IOBASE))  &&
        TEST_IO(fifo = mmap(
            0, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
            mem, HISTORY_FIFO_IOBASE))  &&
        DO_(purge_read_buffer())  &&
        TEST_0(pthread_create(&thread_id, NULL, event_thread, NULL));
}
