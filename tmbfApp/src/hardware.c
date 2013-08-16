#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <pthread.h>

#include "error.h"
#include "hardware.h"


#define TMBF_DATA_ADDRESS       0x14028000
#define TMBF_CONFIG_ADDRESS     0x1402C000

#define DATA_AREA_SIZE          (1 << 14)
#define CONTROL_AREA_SIZE       (1 << 12)

#define ADC_MINMAX_OFFSET       (4 * 256)
#define DAC_MINMAX_OFFSET       (4 * 512)



struct tmbf_config_space
{
    uint32_t fpga_version;          //  0  Version and FIR count
    //  15:0    Version code
    //  19:16   Number of taps in FIR

    uint32_t system_status;         //  1  Status register
    //  2:0     Current sequencer state ("program counter")
    //  3       Set if buffer busy
    //  4       Set if sequencer busy
    //  8:5     Overflow bits (IQ/ACC/DAC/FIR)

    uint32_t control;               //  2  System control register
    //  0       Global DAC output enable (1 => enabled)
    //  1       DDR trigger source select (0 => soft, 1 => external)
    //  2       Buffer & seq trigger source select (0 => soft, 1 => external)
    //  3       (unused)
    //  4       Arm DDR (must be pulsed, works on rising edge)
    //  5       Soft trigger DDR (pulsed)
    //  6       Arm buffer and sequencer
    //  7       Soft trigger buffer and sequencer (pulsed)
    //  8       Arm bunch counter sync (pulsed)
    //  9       (unused)
    //  11:10   Buffer data select (FIR+ADC/IQ/FIR+DAC/ADC+DAC)
    //  13:12   Buffer readback channel select
    //  14      Detector bunch mode enable
    //  15      Detector input select
    //  19      (unused)
    //  18:16   HOM gain select (in 6dB steps)
    //  23:20   FIR gain select (in 6dB steps)
    //  27:24   Detector gain select (in 6dB steps)
    //  29:28   DDR input select (0 => ADC, 1 => DAC, 2 => FIR, 3 => 0)
    //  30      (unused)
    //  31      Enable internal loopback

    uint32_t nco_frequency;         //  3  Fixed NCO generator frequency
    uint32_t dac_delay;             //  4  DAC output delay (2ns steps)
    uint32_t bunch_select;          //  5  Detector bunch selections
    uint32_t adc_offset_ab;         //  6  ADC channel offsets (channels A/B)
    uint32_t adc_offset_cd;         //  7  ADC channel offsets (channels C/D)
    uint32_t fir_bank;              //  8  Select FIR bank
    uint32_t fir_write;             //  9  Write FIR coefficients
    uint32_t bunch_bank;            // 10  Select bunch config bank
    uint32_t bunch_write;           // 11  Write bunch configuration
    uint32_t sequencer_abort;       // 12  Abort sequencer operation
    uint32_t sequencer_arm;         // 13  Arm sequencer
    uint32_t sequencer_start_write; // 14  Starts write sequence
    uint32_t sequencer_write;       // 15  Write sequencer data
    uint32_t adc_minmax_channel;    // 16  Select ADC min/max readout channel
    uint32_t dac_minmax_channel;    // 17  Select DAC min/max readout channel
    uint32_t latch_overflow;        // 18  Latch new overflow status
};

/* These three pointers directly overlay the FF memory. */
static volatile struct tmbf_config_space *config_space;
static volatile uint32_t *adc_minmax;       // ADC min/max readout area
static volatile uint32_t *dac_minmax;       // DAC min/max readout area
static volatile uint32_t *buffer_data;      // Fast buffer readout area



/******************************************************************************/
/* Helper routines and definitions. */

static pthread_mutex_t global_lock = PTHREAD_MUTEX_INITIALIZER;

#define LOCK()      pthread_mutex_lock(&global_lock);
#define UNLOCK()    pthread_mutex_unlock(&global_lock);


/* Reads packed array of min/max values using readout selector.  Used for ADC
 * and DAC readouts. */
static void read_minmax(
    volatile uint32_t *minmax, volatile uint32_t *channel,
    short *min_out, short *max_out)
{
    LOCK();
    for (int n = 0; n < 4; n++)
    {
        /* Channel select and read enable for buffer. */
        *channel = 2*n + 1;
        for (int i = 0; i < MAX_BUNCH_COUNT/4; i++)
        {
            uint32_t data = minmax[i];
            min_out[4*i + n] = (short) (data & 0xFFFF);
            max_out[4*i + n] = (short) (data >> 16);
        }
    }
    *channel = 0;
    UNLOCK();
}


/* Writes to a sub field of a register by reading the register and writing
 * the appropriately masked value.  The register *must* be locked. */
static void write_control_bit_field(
    unsigned int start, unsigned int bits, uint32_t value)
{
    uint32_t mask = ((1 << bits) - 1) << start;
    config_space->control =
        (config_space->control & ~mask) | ((value << start) & mask);
}

/* Sets and the clears all the bits in the given mask. */
static void pulse_mask(uint32_t mask)
{
    LOCK();
    uint32_t state = config_space->control;
    config_space->control = state | mask;
    config_space->control = state & ~mask;
    UNLOCK();
}

/* Sets the selected bit and resets it back to zero. */
static void pulse_control_bit(unsigned int bit)
{
    pulse_mask(1 << bit);
}

#define WRITE_CONTROL_BITS(start, length, value) \
    LOCK(); \
    write_control_bit_field(start, length, value); \
    UNLOCK()


/******************************************************************************/



int hw_read_version(void)
{
    return config_space->fpga_version & 0xFFFF;
}


void hw_read_overflows(bool *fir, bool *dac, bool *acc, bool *iq)
{
    config_space->latch_overflow = 0xF;
    uint32_t status = config_space->system_status;
    *fir = (status >> 5) & 1;
    *dac = (status >> 6) & 1;
    *acc = (status >> 7) & 1;
    *iq  = (status >> 8) & 1;
}


void hw_write_loopback_enable(bool loopback)
{
    WRITE_CONTROL_BITS(31, 1, loopback);
}


/* * * * * * * * * * * * */
/* ADC: Data Input Stage */

static uint32_t combine_offsets(short offset_a, short offset_b)
{
    return ((uint32_t) offset_a & 0xFFFF) + ((uint32_t) offset_b << 16);
}

void hw_write_adc_offsets(short offsets[4])
{
    config_space->adc_offset_ab = combine_offsets(offsets[0], offsets[1]);
    config_space->adc_offset_cd = combine_offsets(offsets[2], offsets[3]);
}


void hw_read_adc_minmax(short min[MAX_BUNCH_COUNT], short max[MAX_BUNCH_COUNT])
{
    read_minmax(adc_minmax, &config_space->adc_minmax_channel, min, max);
}


/* * * * * * * * * * * * * * * */
/* FIR: Filtering for Feedback */

static int fir_filter_length;      // Initialised at startup

void hw_write_fir_gain(unsigned int gain)
{
    WRITE_CONTROL_BITS(20, 4, gain);
}

void hw_write_fir_taps(int bank, int taps[])
{
    LOCK();
    config_space->fir_bank = (uint32_t) bank;
    for (int i = 0; i < fir_filter_length; i++)
        config_space->fir_write = taps[i];
    UNLOCK();
}

int hw_read_fir_length(void)
{
    return fir_filter_length;
}


/* * * * * * * * * * * * * */
/* DAC: Data Output Stage */

void hw_read_dac_minmax(short min[MAX_BUNCH_COUNT], short max[MAX_BUNCH_COUNT])
{
    read_minmax(dac_minmax, &config_space->dac_minmax_channel, min, max);
}

void hw_write_dac_enable(unsigned int enable)
{
    WRITE_CONTROL_BITS(0, 1, enable);
}

void hw_write_dac_delay(unsigned int delay)
{
    config_space->dac_delay = delay;
}


/* * * * * * * * * * * * * * * * * * * * * * */
/* DDR: High Speed High Volume Data Capture */

void hw_write_ddr_select(unsigned int selection)
{
    WRITE_CONTROL_BITS(28, 2, selection);
}

void hw_write_ddr_trigger_select(bool external)
{
    WRITE_CONTROL_BITS(1, 1, external);
}


/* * * * * * * * * * * * * * * * * * * * */
/* BUN: Bunch by Bunch Banked Selection */

void hw_write_bun_entry(int bank, struct bunch_entry entries[MAX_BUNCH_COUNT])
{
    LOCK();
    config_space->bunch_bank = (uint32_t) bank;
    for (int i = 0; i < MAX_BUNCH_COUNT; i ++)
    {
        struct bunch_entry *entry = &entries[i];
        config_space->bunch_write =
            (entry->bunch_gain & 0x7FF) |
            ((entry->output_select & 0x7) << 11) |
            ((entry->fir_select & 0x3) << 14);
    }
    UNLOCK();
}

void hw_write_bun_sync(void)
{
    pulse_control_bit(8);
}


/* * * * * * * * * * * * * * * * * */
/* BUF: High Speed Internal Buffer */

void hw_write_buf_select(unsigned int selection)
{
    WRITE_CONTROL_BITS(10, 2, selection);
}

void hw_write_buf_trigger_select(bool external)
{
    WRITE_CONTROL_BITS(2, 1, external);
}


bool hw_read_buf_status(void)
{
    return (config_space->system_status >> 3) & 1;
}

/* Annoyingly close to identical to min/max readout, but different channel
 * control. */
void hw_read_buf_data(short low[BUF_DATA_LENGTH], short high[BUF_DATA_LENGTH])
{
    LOCK();
    for (int n = 0; n < 4; n++)
    {
        /* Channel select and read enable for buffer. */
        write_control_bit_field(12, 2, n);
        for (int i = 0; i < BUF_DATA_LENGTH/4; i++)
        {
            uint32_t data = buffer_data[i];
            low [4*i + n] = (short) (data & 0xFFFF);
            high[4*i + n] = (short) (data >> 16);
        }
    }
    UNLOCK();
}


/* * * * * * * * * * * * * * * * * * * * * * */
/* NCO: Fixed Frequency Numerical Oscillator */

void hw_write_nco_freq(uint32_t freq)
{
    config_space->nco_frequency = freq;
}

void hw_write_nco_gain(unsigned int gain)
{
    WRITE_CONTROL_BITS(16, 3, gain);
}


/* * * * * * * * * * * * * */
/* DET: Frequency Detector */

void hw_write_det_mode(bool bunch_mode)
{
    WRITE_CONTROL_BITS(14, 1, bunch_mode);
}

void hw_write_det_input_select(unsigned int input)
{
    WRITE_CONTROL_BITS(15, 1, input);
}

void hw_write_det_bunches(unsigned int bunch[4])
{
    config_space->bunch_select =
        (bunch[0] & 0xFF) |
        ((bunch[1] & 0xFF) << 8) |
        ((bunch[2] & 0xFF) << 16) |
        ((bunch[3] & 0xFF) << 24);
}

void hw_write_det_gain(unsigned int gain)
{
    WRITE_CONTROL_BITS(24, 4, gain);
}

void hw_write_det_window(uint16_t window[DET_WINDOW_LENGTH])
{
    LOCK();
    config_space->sequencer_start_write = 1;    // Select sequencer window
    for (int i = 0; i < DET_WINDOW_LENGTH; i ++)
        config_space->sequencer_write = window[i];
    UNLOCK();
}


/* * * * * * * * * * * * * * * * * * * * * */
/* SEQ: Programmed Bunch and Sweep Control */

void hw_write_seq_entries(struct seq_entry entries[MAX_SEQUENCER_COUNT])
{
    LOCK();
    config_space->sequencer_start_write = 0;    // Select seq state file
    for (int i = 0; i < MAX_SEQUENCER_COUNT; i ++)
    {
        struct seq_entry *entry = &entries[i];
        config_space->sequencer_write = entry->start_freq;
        config_space->sequencer_write = entry->delta_freq;
        config_space->sequencer_write = entry->dwell_time;
        config_space->sequencer_write =
            (entry->capture_count & 0xFFF) |        // bits 11:0
            ((entry->bunch_bank & 0x3) << 12) |     //      13:12
            ((entry->hom_gain & 0x7) << 14) |       //      16:14
            (entry->hom_enable << 17) |             //      17
            entry->enable_window << 18;             //      18
        config_space->sequencer_write = entry->window_rate;
        config_space->sequencer_write = 0;
        config_space->sequencer_write = 0;
        config_space->sequencer_write = 0;
    }
    UNLOCK();
}

void hw_write_seq_count(unsigned int sequencer_pc)
{
    config_space->sequencer_arm = sequencer_pc;
}

unsigned int hw_read_seq_state(void)
{
    return config_space->system_status & 0x7;
}

bool hw_read_seq_status(void)
{
    return (config_space->system_status >> 4) & 1;
}

void hw_write_seq_reset(void)
{
    config_space->sequencer_abort = 1;
}


/* * * * * * * * * * * * */
/* TRG: Trigger control. */

static uint32_t make_mask2(int ddr_bit, int buf_bit, bool ddr, bool buf)
{
    uint32_t mask = 0;
    if (ddr) mask |= 1 << ddr_bit;
    if (buf) mask |= 1 << buf_bit;
    return mask;
}

void hw_write_trg_arm(bool ddr, bool buf)
{
    pulse_mask(make_mask2(4, 6, ddr, buf));
}

void hw_write_trg_soft_trigger(bool ddr, bool buf)
{
    pulse_mask(make_mask2(5, 7, ddr, buf));
}


/******************************************************************************/

bool initialise_hardware(void)
{
    int mem;
    bool ok =
        TEST_IO(mem = open("/dev/mem", O_RDWR | O_SYNC))  &&
        TEST_IO(config_space = mmap(
            0, CONTROL_AREA_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
            mem, TMBF_CONFIG_ADDRESS))  &&
        TEST_IO(buffer_data = mmap(
            0, DATA_AREA_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
            mem, TMBF_DATA_ADDRESS));
    if (ok)
    {
        adc_minmax = (volatile void *) config_space + ADC_MINMAX_OFFSET;
        dac_minmax = (volatile void *) config_space + DAC_MINMAX_OFFSET;
        fir_filter_length = (config_space->fpga_version >> 16) & 0xF;
    }
    return ok;
}
