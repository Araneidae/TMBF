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

#define ADC_MINMAX_OFFSET       (4 * 512)
#define DAC_MINMAX_OFFSET       (4 * 768)



struct tmbf_config_space
{
    uint32_t fpga_version;          //  0  Version and FIR count
    //  15:0    Version code
    //  19:16   Number of taps in FIR

    uint32_t control;               //  1  System control register
    //  0       Global DAC output enable (1 => enabled)
    //  1       DDR trigger source select (0 => soft, 1 => external)
    //  2       Buffer & seq trigger source select (0 => soft, 1 => external)
    //  3       (unused)
    //  4       Arm DDR (must be pulsed, works on rising edge)
    //  5       Soft trigger DDR (pulsed)
    //  6       Arm buffer and sequencer
    //  7       Soft trigger buffer and sequencer (pulsed)
    //  8       Arm bunch counter sync (pulsed)
    //  9       DDR input select (0 => ADC, 1 => DAC)
    //  11:10   Buffer data select (FIR+ADC/IQ/FIR+DAC/ADC+DAC)
    //  13:12   Buffer readback channel select
    //  14      Detector bunch mode enable
    //  15      Detector input select
    //  19:16   HOM gain select (in 3dB steps)
    //  23:20   FIR gain select (in 3dB steps)
    //  26:24   Detector gain select (in 6dB steps)
    //  31:27   (unused)

    uint32_t nco_frequency;         //  2  Fixed NCO generator frequency
    uint32_t dac_delay;             //  3  DAC output delay (2ns steps)
    uint32_t bunch_select;          //  4  Detector bunch selections
    uint32_t adc_offset_ab;         //  5  ADC channel offsets (channels A/B)
    uint32_t adc_offset_cd;         //  6  ADC channel offsets (channels C/D)
    uint32_t system_status;         //  7  Status register
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
static void write_bit_field(
    volatile uint32_t *reg, unsigned int start,
    unsigned int bits, uint32_t value)
{
    uint32_t mask = ((1 << bits) - 1) << start;
    *reg = (*reg & ~mask) | ((value << start) & mask);
}

/* Sets the selected bit and resets it back to zero. */
static void pulse_bit_field(volatile uint32_t *reg, unsigned int bit)
{
    write_bit_field(reg, bit, 1, 1);
    write_bit_field(reg, bit, 1, 0);
}


#define WRITE_CONTROL_BITS(start, length, value) \
    LOCK(); \
    write_bit_field(&config_space->control, start, length, value); \
    UNLOCK()

#define PULSE_CONTROL_BIT(bit) \
    LOCK(); \
    pulse_bit_field(&config_space->control, bit); \
    UNLOCK()


bool initialise_hardware(void)
{
    int mem;
    return
        TEST_IO(mem = open("/dev/mem", O_RDWR | O_SYNC))  &&
        TEST_IO(config_space = mmap(
            0, CONTROL_AREA_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
            mem, TMBF_CONFIG_ADDRESS))  &&
        TEST_IO(buffer_data = mmap(
            0, DATA_AREA_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
            mem, TMBF_DATA_ADDRESS))  &&
        DO_(
            adc_minmax = (volatile void *) config_space + ADC_MINMAX_OFFSET;
            dac_minmax = (volatile void *) config_space + DAC_MINMAX_OFFSET);
}


/******************************************************************************/



int hw_read_version(void)
{
    return config_space->fpga_version & 0xFFFF;
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

void hw_write_fir_gain(unsigned int gain)
{
    WRITE_CONTROL_BITS(20, 4, gain);
}

void hw_write_fir_taps(int bank, int taps[MAX_FIR_COEFFS])
{
    LOCK();
    config_space->fir_bank = (uint32_t) bank;
    for (int i = 0; i < MAX_FIR_COEFFS; i++)
        config_space->fir_write = taps[i];
    UNLOCK();
}

int hw_read_fir_length(void)
{
    return MAX_FIR_COEFFS;      // Will interrogate FPGA for this
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
    WRITE_CONTROL_BITS(9, 1, selection);
}

void hw_write_ddr_arm(void)
{
    PULSE_CONTROL_BIT(4);
}

void hw_write_ddr_trigger_select(unsigned int selection)
{
    WRITE_CONTROL_BITS(1, 1, selection);
}

void hw_write_ddr_soft_trigger(void)
{
    PULSE_CONTROL_BIT(5);
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
    PULSE_CONTROL_BIT(8);
}


/* * * * * * * * * * * * * * * * * */
/* BUF: High Speed Internal Buffer */

void hw_write_buf_select(unsigned int selection)
{
    WRITE_CONTROL_BITS(10, 2, selection);
}

void hw_write_buf_arm(void)
{
    PULSE_CONTROL_BIT(6);
}

void hw_write_buf_trigger_select(unsigned int selection)
{
    WRITE_CONTROL_BITS(2, 1, selection);
}

void hw_write_buf_soft_trigger(void)
{
    PULSE_CONTROL_BIT(7);
}

bool hw_read_buf_status(void)
{
    config_space->system_status = 0;    // Temporary FPGA workaround!!
    bool result = (config_space->system_status >> 3) & 1;
    return result;
}

/* Annoyingly close to identical to min/max readout, but different channel
 * control. */
void hw_read_buf_data(short low[BUF_DATA_LENGTH], short high[BUF_DATA_LENGTH])
{
    LOCK();
    for (int n = 0; n < 4; n++)
    {
        /* Channel select and read enable for buffer. */
        write_bit_field(&config_space->control, 12, 2, n);
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
    WRITE_CONTROL_BITS(16, 4, gain);
}


/* * * * * * * * * * * * * */
/* DET: Frequency Detector */

void hw_write_det_mode(bool bunch_mode)
{
    WRITE_CONTROL_BITS(14, 1, bunch_mode);
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
    WRITE_CONTROL_BITS(24, 3, gain);
}


/* * * * * * * * * * * * * * * * * * * * * */
/* SEQ: Programmed Bunch and Sweep Control */

void hw_write_seq_entries(struct seq_entry entries[MAX_SEQUENCER_COUNT])
{
    LOCK();
    config_space->sequencer_start_write = 1;
    for (int i = 0; i < MAX_SEQUENCER_COUNT; i ++)
    {
        struct seq_entry *entry = &entries[i];
        config_space->sequencer_write = entry->start_freq;
        config_space->sequencer_write = entry->delta_freq;
        config_space->sequencer_write = entry->dwell_time;
        config_space->sequencer_write =
            (entry->capture_count & 0xFFF) |        // bits 11:0
            ((entry->bunch_bank & 0x3) << 12) |     //      13:12
            ((entry->hom_gain & 0xF) << 14) |       //      17:14
            ((entry->wait_time & 0x3FFF) << 18);    //      31:18
    }
    UNLOCK();
}

unsigned int hw_read_seq_state(void)
{
    return config_space->system_status & 0x7;
}

void hw_write_seq_reset(void)
{
    config_space->sequencer_abort = 1;
}
