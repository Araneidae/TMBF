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
#include "config_file.h"

#include "hardware.h"


#define TMBF_DATA_ADDRESS       0x14028000
#define TMBF_CONFIG_ADDRESS     0x1402C000

#define DATA_AREA_SIZE          (1 << 14)
#define CONTROL_AREA_SIZE       (1 << 12)

#define ADC_MINMAX_OFFSET       (4 * 256)
#define DAC_MINMAX_OFFSET       (4 * 512)


/* Definition of hardware offsets.  These all define internal delays in the FPGA
 * which we compensate for when reading and writing data.  In all cases the
 * reference bunch "zero" is that selected by the output multiplexor.  Also, all
 * offsets assume bunches are aligned in the closed loop. */
static int DDR_ADC_DELAY;       // Offset into DDR of ADC bunch zero
static int DDR_FIR_DELAY;       //                    FIR
static int DDR_RAW_DAC_DELAY;   //                    Raw DAC
static int DDR_DAC_DELAY;       //                    DAC

static int BUF_ADC_DELAY;       // Offset into BUF of ADC bunch zero
static int BUF_FIR_DELAY;       //                    FIR
static int BUF_DAC_DELAY;       //                    DAC

static int MINMAX_ADC_DELAY;    // Offset into minmax buffer of ADC bunch zero
static int MINMAX_DAC_DELAY;    //                              DAC

static int BUNCH_FIR_OFFSET;    // Offset of FIR selection
static int BUNCH_GAIN_OFFSET;   // Offset of DAC gain selection
static int DET_ADC_OFFSET;      // Offset of detector ADC bunch zero
static int DET_FIR_OFFSET;      // Offset of detector FIR bunch zero

static const struct config_entry hardware_config_defs[] = {
    CONFIG(DDR_ADC_DELAY),
    CONFIG(DDR_FIR_DELAY),
    CONFIG(DDR_RAW_DAC_DELAY),
    CONFIG(DDR_DAC_DELAY),

    CONFIG(BUF_ADC_DELAY),
    CONFIG(BUF_FIR_DELAY),
    CONFIG(BUF_DAC_DELAY),

    CONFIG(MINMAX_ADC_DELAY),
    CONFIG(MINMAX_DAC_DELAY),

    CONFIG(BUNCH_FIR_OFFSET),
    CONFIG(BUNCH_GAIN_OFFSET),
    CONFIG(DET_ADC_OFFSET),
    CONFIG(DET_FIR_OFFSET),
};

static const char *hardware_config_file;



struct tmbf_config_space
{
    uint32_t fpga_version;          //  0  Version and FIR count
    //  15:0    Version code
    //  19:16   Number of taps in FIR

    uint32_t system_status;         //  1  Status register
    //  2:0     Current sequencer state ("program counter")
    //  3       Set if buffer busy
    //  4       Set if sequencer busy
    //  9:5     Overflow bits (IQ/ACC/DAC/FIR)
    //  13:10   Trigger phase bits
    //  14      Trigger armed
    //  15      (unused)
    //  19:16   Bunch trigger phase bits
    //  15:31   (unused)

    uint32_t control;               //  2  System control register
    //  0       Global DAC output enable (1 => enabled)
    //  1       DDR trigger source select (0 => soft, 1 => external)
    //  2       Buffer & seq trigger source select (0 => soft, 1 => external)
    //  3       Enable internal loopback (testing only!)
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
    //  19      (unused)
    //  22:20   FIR gain select (in 6dB steps)
    //  23      (unused)
    //  26:24   Detector gain select (in 6dB steps)
    //  27      (unused)
    //  29:28   DDR input select (0 => ADC, 1 => DAC, 2 => FIR, 3 => 0)
    //  31:30   ACD input fine delay (2ns steps)

    uint32_t nco_frequency;         //  3  Fixed NCO generator frequency
    uint32_t dac_delay;             //  4  DAC output delay (2ns steps)
    uint32_t bunch_select;          //  5  Detector bunch selections
    uint32_t adc_offset_ab;         //  6  ADC channel offsets (channels A/B)
    uint32_t adc_offset_cd;         //  7  ADC channel offsets (channels C/D)
    uint32_t dac_precomp_taps[3];   // 8-10     DAC pre-compensation filter
    uint32_t bunch_zero_offset;     // 11  Bunch zero offset
    uint32_t padding[4];            // 11-15    (unused)

    uint32_t fir_bank;              // 16  Select FIR bank
    uint32_t fir_write;             // 17  Write FIR coefficients
    uint32_t bunch_bank;            // 18  Select bunch config bank
    uint32_t bunch_write;           // 19  Write bunch configuration
    uint32_t sequencer_abort;       // 20  Abort sequencer operation
    uint32_t sequencer_pc;          // 21  Sequencer starting state
    uint32_t sequencer_start_write; // 22  Starts write sequence
    uint32_t sequencer_write;       // 23  Write sequencer data
    uint32_t adc_minmax_channel;    // 24  Select ADC min/max readout channel
    uint32_t dac_minmax_channel;    // 25  Select DAC min/max readout channel
    uint32_t latch_overflow;        // 26  Latch new overflow status
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


/* Used to compensate a value by subtracting a bunch count offset. */
static int subtract_offset(int value, int offset, int max_count)
{
    value -= offset;
    if (value < 0)
        value += max_count;
    return value;
}


/* Reads packed array of min/max values using readout selector.  Used for ADC
 * and DAC readouts. */
static void read_minmax(
    volatile uint32_t *minmax, volatile uint32_t *channel,
    int delay, short *min_out, short *max_out)
{
    LOCK();
    for (int n = 0; n < 4; n++)
    {
        /* Channel select and read enable for buffer. */
        *channel = 2*n + 1;
        for (int i = 0; i < MAX_BUNCH_COUNT/4; i++)
        {
            uint32_t data = minmax[i];
            int out_ix = subtract_offset(i, delay, MAX_BUNCH_COUNT/4);
            min_out[4*out_ix + n] = (short) (data & 0xFFFF);
            max_out[4*out_ix + n] = (short) (data >> 16);
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

#define READ_STATUS_BITS(start, length) \
    ((config_space->system_status >> (start)) & ((1 << (length)) - 1))


/******************************************************************************/



int hw_read_version(void)
{
    return config_space->fpga_version & 0xFFFF;
}


void hw_read_overflows(
    const bool read_bits[OVERFLOW_BIT_COUNT],
    bool overflow_bits[OVERFLOW_BIT_COUNT])
{
    uint32_t read_mask = 0;
    for (int i = 0; i < OVERFLOW_BIT_COUNT; i ++)
        read_mask |= read_bits[i] << i;

    /* Latch the selected overflow bits and read back the associated status. */
    config_space->latch_overflow = read_mask;
    uint32_t status = READ_STATUS_BITS(5, OVERFLOW_BIT_COUNT);

    for (int i = 0; i < OVERFLOW_BIT_COUNT; i ++)
        overflow_bits[i] = (status >> i) & 1;
}


void hw_write_loopback_enable(bool loopback)
{
    WRITE_CONTROL_BITS(3, 1, loopback);
}


void hw_write_compensate_disable(bool disable)
{
    int config_defs_count = ARRAY_SIZE(hardware_config_defs);
    if (disable)
        for (int i = 0; i < config_defs_count; i ++)
            *hardware_config_defs[i].result = 0;
    else
        config_parse_file(
            hardware_config_file, hardware_config_defs, config_defs_count);
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
    read_minmax(
        adc_minmax, &config_space->adc_minmax_channel,
        MINMAX_ADC_DELAY, min, max);
}

void hw_write_adc_skew(unsigned int skew)
{
    WRITE_CONTROL_BITS(30, 2, skew);
}


/* * * * * * * * * * * * * * * */
/* FIR: Filtering for Feedback */

static int fir_filter_length;      // Initialised at startup

void hw_write_fir_gain(unsigned int gain)
{
    WRITE_CONTROL_BITS(20, 3, gain);
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
    read_minmax(
        dac_minmax, &config_space->dac_minmax_channel,
        MINMAX_DAC_DELAY, min, max);
}

void hw_write_dac_enable(unsigned int enable)
{
    WRITE_CONTROL_BITS(0, 1, enable);
}

void hw_write_dac_delay(unsigned int delay)
{
    config_space->dac_delay = delay + 4;
}

void hw_write_dac_precomp(short taps[3])
{
    for (int i = 0; i < 3; i ++)
        config_space->dac_precomp_taps[i] = taps[i];
}


/* * * * * * * * * * * * * * * * * * * * * * */
/* DDR: High Speed High Volume Data Capture */

static int ddr_selection;

void hw_write_ddr_select(unsigned int selection)
{
    ddr_selection = selection;
    WRITE_CONTROL_BITS(28, 2, selection);
}

void hw_write_ddr_trigger_select(bool external)
{
    WRITE_CONTROL_BITS(1, 1, external);
}

int hw_read_ddr_delay(void)
{
    switch (ddr_selection)
    {
        case 0: return DDR_ADC_DELAY;
        case 1: return DDR_FIR_DELAY;
        case 2: return DDR_RAW_DAC_DELAY;
        case 3: return DDR_DAC_DELAY;
        default: ASSERT_FAIL();
    }
}


/* * * * * * * * * * * * * * * * * * * * */
/* BUN: Bunch by Bunch Banked Selection */

void hw_write_bun_entry(int bank, struct bunch_entry entries[MAX_BUNCH_COUNT])
{
    LOCK();
    config_space->bunch_bank = (uint32_t) bank;
    for (int i = 0; i < MAX_BUNCH_COUNT; i ++)
    {
        /* Take bunch offsets into account when writing the bunch entry. */
        int gain_ix = subtract_offset(i, 4*BUNCH_GAIN_OFFSET, MAX_BUNCH_COUNT);
        int output_ix = i;  // Reference bunch, no offset required
        int fir_ix  = subtract_offset(i, 4*BUNCH_FIR_OFFSET, MAX_BUNCH_COUNT);
        unsigned int bunch_gain    = entries[gain_ix].bunch_gain;
        unsigned int output_select = entries[output_ix].output_select;
        unsigned int fir_select    = entries[fir_ix].fir_select;
        config_space->bunch_write =
            (bunch_gain & 0x7FF) |
            ((output_select & 0x7) << 11) |
            ((fir_select & 0x3) << 14);
    }
    UNLOCK();
}

void hw_write_bun_sync(void)
{
    pulse_control_bit(8);
}

void hw_write_bun_zero_bunch(int bunch)
{
    config_space->bunch_zero_offset = bunch;
}

int hw_read_bun_trigger_phase(void)
{
    return READ_STATUS_BITS(16, 4);
}


/* * * * * * * * * * * * * * * * * */
/* BUF: High Speed Internal Buffer */

static int buf_selection;

void hw_write_buf_select(unsigned int selection)
{
    buf_selection = selection;
    WRITE_CONTROL_BITS(10, 2, selection);
}

void hw_write_buf_trigger_select(bool external)
{
    WRITE_CONTROL_BITS(2, 1, external);
}


bool hw_read_buf_status(void)
{
    return READ_STATUS_BITS(3, 1) || READ_STATUS_BITS(14, 1);   // Separate out
}

/* To make things complicated, the fast buffer has separate delays for each of
 * its two channels! */
static void get_buf_delays(int *low_delay, int *high_delay)
{
    switch (buf_selection)
    {
        case 0: *low_delay = BUF_FIR_DELAY; *high_delay = BUF_ADC_DELAY; break;
        case 1: *low_delay = 0;             *high_delay = 0;             break;
        case 2: *low_delay = BUF_FIR_DELAY; *high_delay = BUF_DAC_DELAY; break;
        case 3: *low_delay = BUF_ADC_DELAY; *high_delay = BUF_DAC_DELAY; break;
        default: ASSERT_FAIL();
    }
}

/* Annoyingly close to identical to min/max readout, but different channel
 * control. */
void hw_read_buf_data(short low[BUF_DATA_LENGTH], short high[BUF_DATA_LENGTH])
{
    LOCK();
    int low_delay, high_delay;
    get_buf_delays(&low_delay, &high_delay);

    for (int n = 0; n < 4; n++)
    {
        /* Channel select and read enable for buffer. */
        write_control_bit_field(12, 2, n);
        for (int i = 0; i < BUF_DATA_LENGTH/4; i++)
        {
            uint32_t data = buffer_data[i];
            low [4*((i - low_delay)  & (BUF_DATA_LENGTH/4 - 1)) + n] =
                (short) (data & 0xFFFF);
            high[4*((i - high_delay) & (BUF_DATA_LENGTH/4 - 1)) + n] =
                (short) (data >> 16);
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

static unsigned int det_input;
static unsigned int det_bunches[4];

void hw_write_det_mode(bool bunch_mode)
{
    WRITE_CONTROL_BITS(14, 1, bunch_mode);
}

/* Applies the appropriate bunch selection offset to the detector bunch
 * selection before writing the result to hardware. */
static void update_det_bunch_select(void)
{
    int offset = 0;
    switch (det_input)
    {
        case 0: offset = DET_FIR_OFFSET;    break;
        case 1: offset = DET_ADC_OFFSET;    break;
    }
    unsigned int bunch[4];
    for (int i = 0; i < 4; i ++)
        bunch[i] = subtract_offset(det_bunches[i], offset, MAX_BUNCH_COUNT/4);
    config_space->bunch_select =
        (bunch[0] & 0xFF) |
        ((bunch[1] & 0xFF) << 8) |
        ((bunch[2] & 0xFF) << 16) |
        ((bunch[3] & 0xFF) << 24);
}

void hw_write_det_input_select(unsigned int input)
{
    det_input = input;
    WRITE_CONTROL_BITS(15, 1, input);
    /* As bunch offset compensation depends on which source we have to rewrite
     * the bunches when the input changes. */
    update_det_bunch_select();
}

void hw_write_det_bunches(unsigned int bunch[4])
{
    memcpy(det_bunches, bunch, sizeof(det_bunches));
    update_det_bunch_select();
}

void hw_write_det_gain(unsigned int gain)
{
    WRITE_CONTROL_BITS(24, 3, gain);
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
            (entry->bunch_bank & 0x3) << 12 |       //      13:12
            (entry->hom_gain & 0x7) << 14 |         //      16:14
            entry->hom_enable << 17 |               //      17
            entry->enable_window << 18 |            //      18
            entry->write_enable << 19 |             //      19
            (entry->holdoff & 0xFF) << 20;          //      27:20
        config_space->sequencer_write = entry->window_rate;
        config_space->sequencer_write = 0;
        config_space->sequencer_write = 0;
        config_space->sequencer_write = 0;
    }
    UNLOCK();
}

void hw_write_seq_count(unsigned int sequencer_pc)
{
    config_space->sequencer_pc = sequencer_pc;
}

unsigned int hw_read_seq_state(void)
{
    return READ_STATUS_BITS(0, 3);
}

bool hw_read_seq_status(void)
{
    return READ_STATUS_BITS(4, 1) || READ_STATUS_BITS(14, 1);   // Separate out
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

int hw_read_trg_raw_phase(void)
{
    return READ_STATUS_BITS(10, 4);
}


/******************************************************************************/

bool initialise_hardware(const char *config_file)
{
    hardware_config_file = config_file;
    int mem;
    bool ok =
        config_parse_file(
            config_file, hardware_config_defs,
            ARRAY_SIZE(hardware_config_defs))  &&
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
