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
static int DET_ADC_DELAY;       // Group delay to detector for ADC
static int DET_FIR_DELAY;       // Group delay to detector for FIR

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
    CONFIG(DET_ADC_DELAY),
    CONFIG(DET_FIR_DELAY),
};

static const char *hardware_config_file;



struct tmbf_config_space
{
    /* The first two registers are completely different for reading and writing,
     * so we overlay two completely different register definitions. */
    union {
        // Read only registers
        struct {
            uint32_t fpga_version;  //  0  Version and FIR count
            //  15:0    Version code
            //  19:16   Number of taps in FIR

            uint32_t system_status; //  1  Status register
            //  3:0     Trigger phase bits
            //  7:4     Bunch trigger phase bits
            //  15:8    Overflow bits
            //          0   FIR gain overflow
            //          1   DAC mux output overflow
            //          2   DAC pre-emphasis filter overflow
            //          4   IQ FIR input overflow
            //          5   IQ accumulator overflow
            //          6   IQ readout overflow
            //  18:16   Current sequencer state ("program counter")
            //  19      (unused)
            //  20      Buffer trigger armed
            //  21      Set if buffer busy
            //  22      Set if sequencer busy
            //  23      DDR trigger armed
            //  24      ADC clock dropout detect.
            //  31:25   (unused)
        };

        // Write only registers
        struct {
            uint32_t pulse;         //  0  Pulse event register
            //  0       Arm DDR (must be pulsed, works on rising edge)
            //  1       Soft trigger DDR (pulsed)
            //  2       Arm buffer and sequencer
            //  3       Soft trigger buffer and sequencer (pulsed)
            //  4       Arm bunch counter sync (pulsed)
            //  5       Disarm DDR, pulsed
            //  7       Abort sequencer operation
            //  8       Enable DDR capture (must be done before triggering)
            //  9       Initiate ADC min/max readout
            //  10      Initiate DAC min/max readout

            uint32_t write_select;  //  1  Initiate write register
        };
    };

    uint32_t control;               //  2  System control register
    //  0       Global DAC output enable (1 => enabled)
    //  1       (unused)
    //  2       Detector input select
    //  5:3     Sequencer starting state
    //  9:6     (unused)
    //  11:10   Buffer data select (FIR+ADC/IQ/FIR+DAC/ADC+DAC)
    //  13:12   Buffer readback channel select
    //  14      Detector bunch mode enable
    //  15      Select debug data for IQ buffer input
    //  19:16   HOM gain select (in 6dB steps)
    //  22:20   FIR gain select (in 6dB steps)
    //  23      Enable internal loopback (testing only!)
    //  26:24   Detector gain select (in 6dB steps)
    //  27      Front panel LED
    //  29:28   DDR input select (0 => ADC, 1 => DAC, 2 => FIR, 3 => 0)
    //  31:30   ACD input fine delay (2ns steps)

    uint32_t nco_frequency;         //  3  Fixed NCO generator frequency
    uint32_t dac_delay;             //  4  DAC output delay (2ns steps)
    //  9:0     DAC output delay in 2ns steps
    //  11:10   DAC pre-emphasis filter group delay in 2ns steps
    uint32_t bunch_select;          //  5  Detector bunch selections
    uint32_t adc_offset_ab;         //  6  ADC channel offsets (channels A/B)
    uint32_t adc_offset_cd;         //  7  ADC channel offsets (channels C/D)
    uint32_t dac_preemph_taps[3];   // 8-10     DAC pre-emphasis filter
    uint32_t bunch_zero_offset;     // 11  Bunch zero offset
    uint32_t ddr_trigger_delay;     // 12  DDR Trigger delay control
    uint32_t buf_trigger_delay;     // 13  BUF Trigger delay control
    uint32_t trigger_blanking;      // 14  Trigger blanking length in turns
    uint32_t ftune_control;         // 15  Tune following control
    //  15:0    Dwell time in turns
    //  16      Tune following enable
    //  17      Multibunch enable
    //  19:18   Channel selection
    //  27:20   Single bunch selection
    //  28      Input selection
    //  31:29   (unused)
    uint32_t ftune_control2;        // 16  Extra tune following control
    //  15:0    Cordic rotation angle
    uint32_t fir_write;             // 17  Write FIR coefficients
    uint32_t unused_2;              // 16    (unused)
    uint32_t bunch_write;           // 19  Write bunch configuration
    uint32_t adc_minmax_read;       // 20  Read ADC min/max data
    uint32_t dac_minmax_read;       // 21  Read DAC min/max data
    uint32_t unused_3;              // 22    (unused)
    uint32_t sequencer_write;       // 23  Write sequencer data
    uint32_t unused_4[2];           // 24-25 (unused)
    uint32_t latch_overflow;        // 26  Latch new overflow status
};

/* These two pointers directly overlay the FF memory. */
static volatile struct tmbf_config_space *config_space;
static volatile uint32_t *buffer_data;      // Fast buffer readout area



/******************************************************************************/
/* Helper routines and definitions. */

static pthread_mutex_t global_lock = PTHREAD_MUTEX_INITIALIZER;

#define LOCK()      pthread_mutex_lock(&global_lock);
#define UNLOCK()    pthread_mutex_unlock(&global_lock);


/* Writes to a sub field of a register by reading the register and writing
 * the appropriately masked value.  The register *must* be locked. */
static void write_control_bit_field(
    unsigned int start, unsigned int bits, uint32_t value)
{
    uint32_t mask = ((1 << bits) - 1) << start;
    config_space->control =
        (config_space->control & ~mask) | ((value << start) & mask);
}

/* Writes mask to the pulse register.  This generates simultaneous pulse events
 * for all selected bits. */
static void pulse_mask(uint32_t mask)
{
    config_space->pulse = mask;
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
    int pulse_bit, volatile uint32_t *read_register,
    int delay, short *min_out, short *max_out)
{
    LOCK();
    pulse_control_bit(pulse_bit);
    int out_ix = subtract_offset(0, 4*delay, BUNCHES_PER_TURN);
    for (int i = 0; i < BUNCHES_PER_TURN; i++)
    {
        uint32_t data = *read_register;
        min_out[out_ix] = (short) (data & 0xFFFF);
        max_out[out_ix] = (short) (data >> 16);

        out_ix += 1;
        if (out_ix >= BUNCHES_PER_TURN)
            out_ix = 0;
    }
    UNLOCK();
}


/******************************************************************************/



int hw_read_version(void)
{
    return config_space->fpga_version & 0xFFFF;
}


/* This routine is a bit more involved that most because we want to read a
 * programmable set of bits: all the bits we set are reset after being read. */
void hw_read_overflows(
    const bool read_bits[OVERFLOW_BIT_COUNT],
    bool overflow_bits[OVERFLOW_BIT_COUNT])
{
    uint32_t read_mask = 0;
    for (int i = 0; i < OVERFLOW_BIT_COUNT; i ++)
        read_mask |= read_bits[i] << i;

    /* Latch the selected overflow bits and read back the associated status. */
    config_space->latch_overflow = read_mask;
    uint32_t status = READ_STATUS_BITS(8, OVERFLOW_BIT_COUNT);

    for (int i = 0; i < OVERFLOW_BIT_COUNT; i ++)
        overflow_bits[i] = (status >> i) & 1;
}


void hw_write_loopback_enable(bool loopback)
{
    WRITE_CONTROL_BITS(23, 1, loopback);
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


void hw_write_front_panel_led(bool enable)
{
    WRITE_CONTROL_BITS(27, 1, enable);
}


bool hw_read_clock_dropout(void)
{
    return READ_STATUS_BITS(24, 1);
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

void hw_read_adc_minmax(
    short min[BUNCHES_PER_TURN], short max[BUNCHES_PER_TURN])
{
    read_minmax(9, &config_space->adc_minmax_read, MINMAX_ADC_DELAY, min, max);
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
    config_space->write_select = (uint32_t) bank;
    for (int i = 0; i < fir_filter_length; i++)
        config_space->fir_write = taps[fir_filter_length - i - 1];
    UNLOCK();
}

int hw_read_fir_length(void)
{
    return fir_filter_length;
}


/* * * * * * * * * * * * * */
/* DAC: Data Output Stage */

void hw_read_dac_minmax(
    short min[BUNCHES_PER_TURN], short max[BUNCHES_PER_TURN])
{
    read_minmax(10, &config_space->dac_minmax_read, MINMAX_DAC_DELAY, min, max);
}

void hw_write_dac_enable(unsigned int enable)
{
    WRITE_CONTROL_BITS(0, 1, enable);
}

void hw_write_dac_preemph(short taps[3])
{
    for (int i = 0; i < 3; i ++)
        config_space->dac_preemph_taps[i] = taps[i];
}

void hw_write_dac_delay(unsigned int dac_delay, unsigned int preemph_delay)
{
    config_space->dac_delay =
        ((dac_delay + 4) & 0x3ff) |
        (preemph_delay & 0x3) << 10;
}


/* * * * * * * * * * * * * * * * * * * * * * */
/* DDR: High Speed High Volume Data Capture */

static int ddr_selection;

void hw_write_ddr_select(unsigned int selection)
{
    ddr_selection = selection;
    WRITE_CONTROL_BITS(28, 2, selection);
}

void hw_write_ddr_enable(void)
{
    pulse_control_bit(8);
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

void hw_write_bun_entry(int bank, struct bunch_entry entries[BUNCHES_PER_TURN])
{
    LOCK();
    config_space->write_select = (uint32_t) bank;
    for (int i = 0; i < BUNCHES_PER_TURN; i ++)
    {
        /* Take bunch offsets into account when writing the bunch entry. */
        int gain_ix = subtract_offset(i, 4*BUNCH_GAIN_OFFSET, BUNCHES_PER_TURN);
        int output_ix = i;  // Reference bunch, no offset required
        int fir_ix  = subtract_offset(i, 4*BUNCH_FIR_OFFSET, BUNCHES_PER_TURN);
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
    pulse_control_bit(4);
}

void hw_write_bun_zero_bunch(int bunch)
{
    config_space->bunch_zero_offset = bunch;
}

int hw_read_bun_trigger_phase(void)
{
    return READ_STATUS_BITS(4, 4);
}


/* * * * * * * * * * * * * * * * * */
/* BUF: High Speed Internal Buffer */

static int buf_selection;

void hw_write_buf_select(unsigned int selection)
{
    LOCK();
    buf_selection = selection;
    /* The buffer selection is a bit weird.  The bottom bit selects whether
     * debug data is routed to IQ, the top two bits are the actual selection,
     * and debug is only available on IQ. */
    bool enable_debug = selection == BUF_SELECT_DEBUG;
    write_control_bit_field(10, 2, enable_debug ? BUF_SELECT_IQ : selection);
    write_control_bit_field(15, 1, enable_debug);
    UNLOCK();
}


bool hw_read_buf_status(void)
{
    return READ_STATUS_BITS(20, 1) || READ_STATUS_BITS(21, 1); // Separate out?
}

bool hw_read_buf_busy(void)
{
    if (buf_selection == BUF_SELECT_IQ)
        return hw_read_buf_status()  &&  hw_read_seq_status();
    else
        return hw_read_buf_status();
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
        case 4: *low_delay = 0;             *high_delay = 0;             break;
        default: ASSERT_FAIL();
    }
}

/* Annoyingly close to identical to min/max readout, but different channel
 * control. */
void hw_read_buf_data(
    int raw[RAW_BUF_DATA_LENGTH],
    short low[RAW_BUF_DATA_LENGTH], short high[RAW_BUF_DATA_LENGTH])
{
    LOCK();
    int low_delay, high_delay;
    get_buf_delays(&low_delay, &high_delay);

    for (int n = 0; n < 4; n++)
    {
        /* Channel select and read enable for buffer. */
        write_control_bit_field(12, 2, n);
        for (int i = 0; i < RAW_BUF_DATA_LENGTH/4; i++)
        {
            uint32_t data = buffer_data[i];
            raw [4*i + n] = data;
            low [4*((i - low_delay)  & (RAW_BUF_DATA_LENGTH/4 - 1)) + n] =
                (short) (data & 0xFFFF);
            high[4*((i - high_delay) & (RAW_BUF_DATA_LENGTH/4 - 1)) + n] =
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
    WRITE_CONTROL_BITS(16, 4, gain);
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
        case DET_IN_ADC: offset = DET_ADC_OFFSET; break;
        case DET_IN_FIR: offset = DET_FIR_OFFSET; break;
    }
    unsigned int bunch[4];
    for (int i = 0; i < 4; i ++)
        bunch[i] = subtract_offset(det_bunches[i], offset, BUNCHES_PER_TURN/4);
    config_space->bunch_select =
        (bunch[0] & 0xFF) |
        ((bunch[1] & 0xFF) << 8) |
        ((bunch[2] & 0xFF) << 16) |
        ((bunch[3] & 0xFF) << 24);
}

void hw_write_det_input_select(unsigned int input)
{
    det_input = input;
    WRITE_CONTROL_BITS(2, 1, input);
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
    config_space->write_select = 1;    // Select sequencer window
    for (int i = 0; i < DET_WINDOW_LENGTH; i ++)
        config_space->sequencer_write = window[i];
    UNLOCK();
}

void hw_read_det_delays(int *adc_delay, int *fir_delay)
{
    *adc_delay = DET_ADC_DELAY;
    *fir_delay = DET_FIR_DELAY;
}


/* * * * * * * * * * * * * * * * * * * * * */
/* FTUN: Fast Tune Following and Detector  */

static int ftun_dwell;
static int ftun_bunch;
static bool ftun_multibunch;
static unsigned int ftun_input_select;
static bool ftun_enable;

static void update_ftune_state(void)
{
    config_space->ftune_control =
        ((ftun_dwell - 1) & 0xFFFF) |       // bits 15:0
        ftun_enable << 16 |                 //      16
        ftun_multibunch << 17 |             //      17
        (ftun_bunch & 0x3FF) << 18 |        //      27:18
        (ftun_input_select & 0x1) << 28;    //      28
}

#define DEFINE_FTUN_WRITE(name) \
    void hw_write_ftun_##name(typeof(ftun_##name) name) \
    { \
        LOCK(); \
        ftun_##name = name; \
        update_ftune_state(); \
        UNLOCK(); \
    }

DEFINE_FTUN_WRITE(dwell)
DEFINE_FTUN_WRITE(bunch)
DEFINE_FTUN_WRITE(multibunch)
DEFINE_FTUN_WRITE(input_select)
DEFINE_FTUN_WRITE(enable)


void hw_write_ftun_rotation(uint32_t rotation)
{
    config_space->ftune_control2 = rotation & 0xFFFF;
}


/* * * * * * * * * * * * * * * * * * * * * */
/* SEQ: Programmed Bunch and Sweep Control */

void hw_write_seq_entries(
    unsigned int bank0, struct seq_entry entries[MAX_SEQUENCER_COUNT])
{
    LOCK();
    config_space->write_select = 0;    // Select seq state file
    /* State zero is special: everything except the bank selection must be
     * written as zero.  Unfortunately, these aren't the zeros in the seq_entry,
     * so we have to write this out. */
    config_space->sequencer_write = 0;
    config_space->sequencer_write = 0;
    config_space->sequencer_write = 0;
    /* Set bank0 as requested and disable NCO output in state zero. */
    config_space->sequencer_write = (bank0 & 0x3) << 12 | 0xF << 14;
    config_space->sequencer_write = 0;
    config_space->sequencer_write = 0;
    config_space->sequencer_write = 0;
    config_space->sequencer_write = 0;

    for (int i = 0; i < MAX_SEQUENCER_COUNT; i ++)
    {
        struct seq_entry *entry = &entries[i];
        config_space->sequencer_write = entry->start_freq;
        config_space->sequencer_write = entry->delta_freq;
        config_space->sequencer_write = entry->dwell_time - 1;
        config_space->sequencer_write =
            ((entry->capture_count - 1) & 0xFFF) |  // bits 11:0
            (entry->bunch_bank & 0x3) << 12 |       //      13:12
            (entry->hom_gain & 0xF) << 14 |         //      17:14
            entry->enable_window << 18 |            //      18
            entry->write_enable << 19 |             //      19
            entry->enable_blanking << 20;           //      20
        config_space->sequencer_write = entry->window_rate;
        config_space->sequencer_write = entry->holdoff & 0xFFFF;
        config_space->sequencer_write = 0;
        config_space->sequencer_write = 0;
    }
    UNLOCK();
}

void hw_write_seq_count(unsigned int sequencer_pc)
{
    WRITE_CONTROL_BITS(3, 3, sequencer_pc);
}

unsigned int hw_read_seq_state(void)
{
    return READ_STATUS_BITS(16, 3);
}

bool hw_read_seq_status(void)
{
    return READ_STATUS_BITS(20, 1) || READ_STATUS_BITS(22, 1); // Separate out?
}

void hw_write_seq_reset(void)
{
    pulse_control_bit(7);
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
    pulse_mask(make_mask2(0, 2, ddr, buf));
}

void hw_write_trg_soft_trigger(bool ddr, bool buf)
{
    pulse_mask(make_mask2(1, 3, ddr, buf));
}

void hw_write_trg_disarm(bool ddr, bool buf)
{
    pulse_mask(make_mask2(5, 6, ddr, buf));
}

void hw_write_trg_delays(int ddr_delay, int buf_delay)
{
    config_space->ddr_trigger_delay = ddr_delay;
    config_space->buf_trigger_delay = buf_delay;
}

void hw_write_trg_blanking(int trigger_blanking)
{
    config_space->trigger_blanking = trigger_blanking;
}

void hw_write_trg_arm_raw_phase(void)
{
    pulse_control_bit(11);
}

int hw_read_trg_raw_phase(void)
{
    return READ_STATUS_BITS(0, 4);
}


/******************************************************************************/

bool initialise_hardware(const char *config_file)
{
    hardware_config_file = config_file;
    int mem;
    return
        config_parse_file(
            config_file, hardware_config_defs,
            ARRAY_SIZE(hardware_config_defs))  &&
        TEST_IO(mem = open("/dev/mem", O_RDWR | O_SYNC))  &&
        TEST_IO(config_space = mmap(
            0, CONTROL_AREA_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
            mem, TMBF_CONFIG_ADDRESS))  &&
        TEST_IO(buffer_data = mmap(
            0, DATA_AREA_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
            mem, TMBF_DATA_ADDRESS))  &&
        DO_(fir_filter_length = (config_space->fpga_version >> 16) & 0xF);
}
