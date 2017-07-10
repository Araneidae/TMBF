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


#define TMBF_CONFIG_ADDRESS     0x1402C000
#define CONTROL_AREA_SIZE       (1 << 12)


/* Definition of hardware offsets.  These all define internal delays in the FPGA
 * which we compensate for when reading and writing data.  In all cases the
 * reference bunch "zero" is that selected by the output multiplexer.  Also, all
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

static int FTUN_ADC_OFFSET;     // Offset of tune following ADC bunch zero
static int FTUN_FIR_OFFSET;     // Offset of tune following FIR bunch zero
static int FTUN_ADC_DELAY;      // Group delay to FTUN detector for ADC
static int FTUN_FIR_DELAY;      // Group delay to FTUN detector for FIR

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

    CONFIG(FTUN_ADC_OFFSET),
    CONFIG(FTUN_FIR_OFFSET),
    CONFIG(FTUN_ADC_DELAY),
    CONFIG(FTUN_FIR_DELAY),
};

static const char *hardware_config_file;



struct tmbf_config_space
{
    /* Because each register has a different read and write meaning we define
     * different overlaid read and write names for each register. */

    union {
        /* The following registers are read only. */
        const struct {
            uint32_t fpga_version;      //  0  Version and FIR count
            uint32_t system_status;     //  1  Status register
            //  3:0     Trigger phase bits
            //  7:4     Bunch trigger phase bits
            //  15:8    (unused)
            //  18:16   Current sequencer state ("program counter")
            //  19      (unused)
            //  20      Buffer trigger armed
            //  21      Set if buffer busy
            //  22      Set if sequencer busy
            //  23      DDR trigger armed
            //  24      ADC clock dropout detect.
            //  31:25   (unused)
            uint32_t unused_r_2;        //  2   (unused)
            uint32_t latch_pulsed_r;    //  3  Pulsed bits readback
            //  0   FIR gain overflow
            //  1   DAC mux output overflow
            //  2   DAC pre-emphasis filter overflow
            //  4   IQ FIR input overflow
            //  5   IQ accumulator overflow
            //  6   IQ readout overflow
            uint32_t ddr_offset;        //  4  DDR capture count or offset
            //  23:0    Trigger offset into DDR buffer
            //  31      Set if DDR waiting for trigger
            uint32_t unused_r_5;        //  5   (unused)
            uint32_t unused_r_6;        //  6   (unused)
            uint32_t super_count_r;     //  7  Reads current super count
            uint32_t unused_r_8;        //  8   (unused)
            uint32_t unused_r_9;        //  9   (unused)
            uint32_t unused_r_10;       // 10   (unused)
            uint32_t unused_r_11;       // 11   (unused)
            uint32_t unused_r_12;       // 12   (unused)
            uint32_t unused_r_13;       // 13   (unused)
            uint32_t unused_r_14;       // 14   (unused)
            uint32_t unused_r_15;       // 15   (unused)
            uint32_t unused_r_16;       // 16   (unused)
            uint32_t unused_r_17;       // 17   (unused)
            uint32_t unused_r_18;       // 18   (unused)
            uint32_t unused_r_19;       // 19   (unused)
            uint32_t adc_minmax_read;   // 20  Read ADC min/max data
            uint32_t dac_minmax_read;   // 21  Read DAC min/max data
            uint32_t fast_buffer_read;  // 22  Read fast buffer data
            uint32_t unused_r_23;       // 23   (unused)

            // The following block of 8 registers is dedicated to tune following
            uint32_t ftune_status;      // 24  Tune following status
            // Status bits:
            //  0       Set if integrated frequency out of range
            //  1       Set if signal magnitude too small
            //  2       Set on detector output overflow
            //  3       Set on detector accumulator overflow
            //  4       Set on FIR input overflow
            //  5       Set if tune following feedback running
            //  7:6     (unused)
            //  12:8    Zero when feedback running, set to a copy of bits 4:0
            //          when feedback halted.
            uint32_t ftune_readout;     // 25  Tune following readout
            // When read returns frequency offset values from FIFO with status
            // and buffer info:
            //  17:0    Frequency offset
            //  30:21   Number of samples in buffer (including value being read)
            //  31      Set of FIFO overflow has occurred
            uint32_t ftune_iq;          // 26  Filtered IQ readback
            uint32_t ftune_freq_offset; // 27  Filtered frequency offset
            uint32_t ftune_i_minmax;    // 28  I min and max
            uint32_t ftune_q_minmax;    // 29  Q min and max
            uint32_t unused_r_30;       // 30   (unused)
            uint32_t unused_r_31;       // 31   (unused)
        };

        /* The following registers are write only. */
        struct {
            uint32_t pulse;             //  0  Pulse event register
            // All writes to this register generate single clock pulses for all
            // written bits with the following effect:
            //  0       Arm DDR
            //  1       Soft trigger DDR
            //  2       Arm buffer and sequencer
            //  3       Soft trigger buffer and sequencer
            //  4       Arm bunch counter sync
            //  5       Disarm DDR, pulsed
            //  7       Abort sequencer operation
            //  8       Enable DDR capture (must be done before triggering)
            //  9       Initiate ADC min/max readout
            //  10      Initiate DAC min/max readout
            //  11      Arm trigger phase capture
            //  12      Enable tune following start
            //  13      Initiate fast buffer readout
            uint32_t write_select;      //  1  Initiate write register
            uint32_t control;           //  2  System control register
            //  0       Global DAC output enable (1 => enabled)
            //  1       Enable tune following feedback
            //  2       Detector input select
            //  5:3     Sequencer starting state
            //  7:6     (unused)
            //  8       Detector bunch mode enable
            //  9       Select blanking source
            //  11:10   Buffer data select (FIR+ADC/IQ/FIR+DAC/ADC+DAC)
            //  14:12   Select sequencer state for trigger generation
            //  15      Select debug data for IQ buffer input
            //  19:16   HOM gain select (in 6dB steps)
            //  22:20   FIR gain select (in 6dB steps)
            //  23      Enable internal loopback (testing only!)
            //  26:24   Detector gain select (in 6dB steps)
            //  27      Front panel LED
            //  29:28   DDR input select (0 => ADC, 1 => DAC, 2 => FIR, 3 => 0)
            //  31:30   ACD input fine delay (2ns steps)
            uint32_t latch_pulsed;      //  3  Latch pulsed bit status
            uint32_t control2;          //  4  Second control register
            //  9:0     DAC output delay in 2ns steps
            //  11:10   DAC pre-emphasis filter group delay in 2ns steps
            //  12      Whether DDR trigger source respects blanking
            //  15:13   Select DDR trigger source
            uint32_t bunch_select;      //  5  Detector bunch selections
            uint32_t adc_offsets;       //  6  ADC channel offsets (A/B)
            uint32_t super_count;       //  7  Sequencer super state count
            uint32_t dac_preemph_taps;  //  8  DAC pre-emphasis filter
            uint32_t adc_filter_taps;   //  9  ADC compensation filter
            uint32_t control3;          // 10  Decimation control
            uint32_t bunch_zero_offset; // 11  Bunch zero offset
            uint32_t ddr_trigger_delay; // 12  DDR Trigger delay control
            uint32_t buf_trigger_delay; // 13  BUF Trigger delay control
            uint32_t trigger_blanking;  // 14  Trigger blanking length in turns
            uint32_t unused_w_15;       // 15   (unused)
            uint32_t unused_w_16;       // 16   (unused)
            uint32_t fir_write;         // 17  Write FIR coefficients
            uint32_t adc_limit;         // 18  Configure ADC limit threshold
            uint32_t bunch_write;       // 19  Write bunch configuration
            uint32_t unused_w_20;       // 20   (unused)
            uint32_t unused_w_21;       // 21   (unused)
            uint32_t unused_w_22;       // 22   (unused)
            uint32_t sequencer_write;   // 23  Write sequencer data

            // The following block of 8 registers is dedicated to tune following
            uint32_t ftune_control;     // 24  Tune following master control
            // For writing supports the following fields:
            //  15:0    Dwell time in turns
            //  16      Blanking enable
            //  17      Multibunch enable
            //  19:18   Channel selection
            //  27:20   Single bunch selection
            //  28      Input selection
            //  31:29   Detector gain
            uint32_t ftune_target;      // 25  Target phase and control
            // For writing supports these fields:
            //  17:0    Target phase
            //  20:18   IIR scaling
            uint32_t ftune_i_scale;     // 26  Tune following integral scaling
            uint32_t ftune_min_mag;     // 27  Magnitude threshold for feedback
            uint32_t ftune_max_offset;  // 28  Frequency offset feedback limit
            uint32_t nco_frequency;     // 29  Fixed NCO generator frequency
            uint32_t ftune_p_scale;     // 30  Feedback proportional scale
            uint32_t ftune_read_control; // 31  Latches readback values
            // Control bits:
            //  0       Latches and resets accumulated ftune_status bits 4:0
            //  1       Latches and resets ftune_i_minmax readout
            //  2       Latches and resets ftune_q_minmax readout
        };
    };
};

/* TMBF register control space. */
static volatile struct tmbf_config_space *config_space;



/******************************************************************************/
/* Helper routines and definitions. */

static pthread_mutex_t global_lock = PTHREAD_MUTEX_INITIALIZER;

#define LOCK()      pthread_mutex_lock(&global_lock);
#define UNLOCK()    pthread_mutex_unlock(&global_lock);

/* Images of what was last written to the three control bit fields. */
static uint32_t control_field_1 = 0;
static uint32_t control_field_2 = 0;
static uint32_t control_field_3 = 0;


static uint32_t read_bit_field(
    uint32_t value, unsigned int start, unsigned int length)
{
    return (value >> start) & ((1U << length) - 1);
}

/* Writes to a sub field of a register by reading the register and writing
 * the appropriately masked value.  The register *must* be locked. */
static void write_control_bit_field(
    volatile uint32_t *control, uint32_t *memory,
    unsigned int start, unsigned int bits, uint32_t value)
{
    uint32_t mask = ((1U << bits) - 1U) << start;
    uint32_t update = (*memory & ~mask) | ((value << start) & mask);
    *memory = update;
    *control = update;
}

static void write_control_bits(
    unsigned int start, unsigned int bits, uint32_t value)
{
    write_control_bit_field(
        &config_space->control, &control_field_1, start, bits, value);
}

static void write_control_bits_2(
    unsigned int start, unsigned int bits, uint32_t value)
{
    write_control_bit_field(
        &config_space->control2, &control_field_2, start, bits, value);
}

static void write_control_bits_3(
    unsigned int start, unsigned int bits, uint32_t value)
{
    write_control_bit_field(
        &config_space->control3, &control_field_3, start, bits, value);
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
    pulse_mask(1U << bit);
}

#define WRITE_CONTROL_BITS(start, length, value) \
    LOCK(); \
    write_control_bits(start, length, value); \
    UNLOCK()

#define WRITE_CONTROL_BITS_2(start, length, value) \
    LOCK(); \
    write_control_bits_2(start, length, value); \
    UNLOCK()

#define WRITE_CONTROL_BITS_3(start, length, value) \
    LOCK(); \
    write_control_bits_3(start, length, value); \
    UNLOCK()

#define READ_STATUS_BITS(start, length) \
    read_bit_field(config_space->system_status, start, length)


/* Used to compensate a value by subtracting a bunch count offset. */
static unsigned int subtract_offset(
    unsigned int value, int offset, unsigned int max_count)
{
    unsigned int result;
    if (offset > 0)
        result = value + max_count - (unsigned int) offset;
    else
        result = value + (unsigned int) - offset;
    if (result >= max_count)
        result -= max_count;
    return result;
}


/* Reads packed array of min/max values using readout selector.  Used for ADC
 * and DAC readouts. */
static void read_minmax(
    unsigned int pulse_bit, volatile const uint32_t *read_register,
    int delay, short min_out[], short max_out[])
{
    LOCK();
    pulse_control_bit(pulse_bit);
    unsigned int out_ix = subtract_offset(0, 4*delay, BUNCHES_PER_TURN);
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

unsigned int fpga_version;


unsigned int hw_read_version(void)
{
    return fpga_version;
}


static uint32_t bool_array_to_bits(size_t count, const bool array[])
{
    uint32_t result = 0;
    for (size_t i = 0; i < count; i ++)
        result |= array[i] << i;
    return result;
}

static void bits_to_bool_array(size_t count, bool array[], uint32_t bits)
{
    for (size_t i = 0; i < count; i ++)
        array[i] = (bits >> i) & 1;
}


void hw_read_pulsed_bits(
    const bool read_bits[PULSED_BIT_COUNT],
    bool pulsed_bits[PULSED_BIT_COUNT])
{
    LOCK();
    config_space->latch_pulsed =
        bool_array_to_bits(PULSED_BIT_COUNT, read_bits);
    bits_to_bool_array(
        PULSED_BIT_COUNT, pulsed_bits, config_space->latch_pulsed_r);
    UNLOCK();
}


void hw_write_loopback_enable(bool loopback)
{
    WRITE_CONTROL_BITS(23, 1, loopback);
}


void hw_write_compensate_disable(bool disable)
{
    size_t config_defs_count = ARRAY_SIZE(hardware_config_defs);
    if (disable)
        for (size_t i = 0; i < config_defs_count; i ++)
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

void hw_write_adc_offsets(int offsets[4])
{
    LOCK();
    config_space->write_select = 0;
    for (int i = 0; i < 4; i ++)
        config_space->adc_offsets = (uint32_t) offsets[i];
    UNLOCK();
}

void hw_write_adc_filter(int taps[12])
{
    LOCK();
    config_space->write_select = 0;
    for (int i = 2; i >= 0; i --)
        for (int j = 0; j < 4; j ++)
            config_space->adc_filter_taps = (uint32_t) taps[3*j + i];
    UNLOCK();
}

void hw_write_adc_filter_delay(unsigned int delay)
{
    WRITE_CONTROL_BITS_2(22, 2, delay);
}

void hw_read_adc_minmax(
    short min[BUNCHES_PER_TURN], short max[BUNCHES_PER_TURN])
{
    read_minmax(9, &config_space->adc_minmax_read, MINMAX_ADC_DELAY, min, max);
    /* After ADC minmax readout re-enable ADC threshold detection. */
    pulse_control_bit(17);
}

void hw_write_adc_skew(unsigned int skew)
{
    WRITE_CONTROL_BITS(30, 2, skew);
}

void hw_write_adc_limit(int limit)
{
    config_space->adc_limit = (uint32_t) limit;
}


/* * * * * * * * * * * * * * * */
/* FIR: Filtering for Feedback */

static unsigned int fir_filter_length;      // Initialised at startup

void hw_write_fir_gain(unsigned int gain)
{
    WRITE_CONTROL_BITS_3(9, 4, gain);
}

void hw_write_fir_taps(unsigned int bank, const int taps[])
{
    LOCK();
    config_space->write_select = bank;
    for (unsigned int i = 0; i < fir_filter_length; i++)
        config_space->fir_write = (uint32_t) taps[fir_filter_length - i - 1];
    UNLOCK();
}

void hw_write_fir_decimation(unsigned int decimation)
{
    WRITE_CONTROL_BITS_3(0, 7, decimation - 1);
}

void hw_write_fir_decimation_gain(unsigned int gain)
{
    WRITE_CONTROL_BITS_3(7, 2, gain);
}

unsigned int hw_read_fir_length(void)
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

void hw_write_dac_enable(bool enable)
{
    WRITE_CONTROL_BITS(0, 1, enable);
}

void hw_write_dac_preemph(int taps[3])
{
    LOCK();
    config_space->write_select = 0;
    for (int i = 2; i >= 0; i --)
        for (int j = 0; j < 4; j ++)
            config_space->dac_preemph_taps = (uint32_t) taps[i];
    UNLOCK();
}

void hw_write_dac_delay(unsigned int dac_delay)
{
    WRITE_CONTROL_BITS_2(0, 10, dac_delay + 4);
}

void hw_write_dac_filter_delay(unsigned int preemph_delay)
{
    WRITE_CONTROL_BITS_2(30, 2, preemph_delay);
}


/* * * * * * * * * * * * * * * * * * * * * * */
/* DDR: High Speed High Volume Data Capture */

static unsigned int ddr_selection;

void hw_write_ddr_select(unsigned int selection)
{
    LOCK();
    ddr_selection = selection;
    write_control_bits(28, 2, selection);
    write_control_bits(6, 1, selection >> 2);
    UNLOCK();
}

void hw_write_ddr_enable(void)
{
    pulse_control_bit(8);
}

void hw_write_ddr_disable(void)
{
    pulse_control_bit(16);
}

int hw_read_ddr_delay(void)
{
    switch (ddr_selection)
    {
        case DDR_SELECT_ADC:     return DDR_ADC_DELAY;
        case DDR_SELECT_FIR:     return DDR_FIR_DELAY;
        case DDR_SELECT_RAW_DAC: return DDR_RAW_DAC_DELAY;
        case DDR_SELECT_DAC:     return DDR_DAC_DELAY;
        case DDR_SELECT_IQ:      return 0;
        case DDR_SELECT_DEBUG:   return 0;
        default: ASSERT_FAIL();
    }
}

uint32_t hw_read_ddr_offset(void)
{
    return config_space->ddr_offset & 0xFFFFFF;
}

void hw_read_ddr_status(bool *armed, bool *busy, bool *iq_select)
{
    uint32_t status = config_space->system_status;
    *armed = read_bit_field(status, 23, 1);
    *busy  = read_bit_field(status, 26, 1);
    *iq_select = ddr_selection >= DDR_SELECT_IQ;
}


/* * * * * * * * * * * * * * * * * * * * */
/* BUN: Bunch by Bunch Banked Selection */

void hw_write_bun_entry(
    unsigned int bank, const struct bunch_entry entries[BUNCHES_PER_TURN])
{
    LOCK();
    config_space->write_select = bank;
    for (unsigned int i = 0; i < BUNCHES_PER_TURN; i ++)
    {
        /* Take bunch offsets into account when writing the bunch entry. */
        unsigned int gain_ix =
            subtract_offset(i, 4*BUNCH_GAIN_OFFSET, BUNCHES_PER_TURN);
        unsigned int output_ix = i;  // Reference bunch, no offset required
        unsigned int fir_ix =
            subtract_offset(i, 4*BUNCH_FIR_OFFSET, BUNCHES_PER_TURN);
        uint32_t bunch_gain    = (uint32_t) entries[gain_ix].bunch_gain;
        uint32_t output_select = (uint32_t) entries[output_ix].output_select;
        uint32_t fir_select    = (uint32_t) entries[fir_ix].fir_select;
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

void hw_write_bun_zero_bunch(unsigned int bunch)
{
    config_space->bunch_zero_offset = bunch | ((ATOMS_PER_TURN-1) << 16);
}

unsigned int hw_read_bun_trigger_phase(void)
{
    return READ_STATUS_BITS(4, 4);
}


/* * * * * * * * * * * * * * * * * */
/* BUF: High Speed Internal Buffer */

static unsigned int buf_selection;

void hw_write_buf_select(unsigned int selection)
{
    LOCK();
    buf_selection = selection;
    /* The buffer selection is a bit weird.  The bottom bit selects whether
     * debug data is routed to IQ, the top two bits are the actual selection,
     * and debug is only available on IQ. */
    bool enable_debug = selection == BUF_SELECT_DEBUG;
    write_control_bits(10, 2, enable_debug ? BUF_SELECT_IQ : selection);
    write_control_bits(15, 1, enable_debug);
    UNLOCK();
}

void hw_read_buf_status(bool *armed, bool *busy, bool *iq_select)
{
    uint32_t status = config_space->system_status;
    *armed = read_bit_field(status, 20, 1);
    *busy  = read_bit_field(status, 21, 1);
    *iq_select = buf_selection == BUF_SELECT_IQ;
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


static void reset_buf(short buf[BUF_DATA_LENGTH])
{
    memset(buf, 0, sizeof(short) * BUNCHES_PER_TURN);
    memset(buf + BUF_DATA_LENGTH - BUNCHES_PER_TURN,
        0, sizeof(short) * BUNCHES_PER_TURN);
}

static void update_buf(
    short buf[BUF_DATA_LENGTH], int delay, int ix, short value)
{
    ix -= 4 * delay;
    if (0 <= ix  &&  ix < BUF_DATA_LENGTH)
        buf[ix] = value;
}

void hw_read_buf_data(
    int raw[BUF_DATA_LENGTH],
    short low[BUF_DATA_LENGTH], short high[BUF_DATA_LENGTH])
{
    /* Reset ends of buffers in case we don't overwrite them because of delay
     * compensation. */
    reset_buf(low);
    reset_buf(high);

    LOCK();
    int low_delay, high_delay;
    get_buf_delays(&low_delay, &high_delay);

    pulse_control_bit(13);
    for (int i = 0; i < BUF_DATA_LENGTH; i++)
    {
        uint32_t data = config_space->fast_buffer_read;

        raw[i] = (int) data;
        update_buf(low, low_delay, i, (short) (data & 0xFFFF));
        update_buf(high, high_delay, i, (short) (data >> 16));
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
    WRITE_CONTROL_BITS(8, 1, bunch_mode);
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

    config_space->write_select = 0;
    for (int i = 0; i < 4; i ++)
        config_space->bunch_select =
            subtract_offset(det_bunches[i], offset, BUNCHES_PER_TURN/4);
}

void hw_write_det_input_select(unsigned int input)
{
    LOCK();
    det_input = input;
    write_control_bits(2, 1, input);
    /* As bunch offset compensation depends on which source we have to rewrite
     * the bunches when the input changes. */
    update_det_bunch_select();
    UNLOCK();
}

void hw_write_det_bunches(const unsigned int bunch[4])
{
    LOCK();
    memcpy(det_bunches, bunch, sizeof(det_bunches));
    update_det_bunch_select();
    UNLOCK();
}

void hw_write_det_gain(unsigned int gain)
{
    WRITE_CONTROL_BITS(24, 3, gain);
}

void hw_write_det_window(const int window[DET_WINDOW_LENGTH])
{
    LOCK();
    config_space->write_select = 1;    // Select sequencer window
    for (int i = 0; i < DET_WINDOW_LENGTH; i ++)
        config_space->sequencer_write = (uint32_t) window[i];
    UNLOCK();
}

void hw_read_det_delays(int *adc_delay, int *fir_delay)
{
    *adc_delay = DET_ADC_DELAY;
    *fir_delay = DET_FIR_DELAY;
}


/* * * * * * * * * * * * * * * * * * * * * */
/* FTUN: Fast Tune Following and Detector  */

void hw_write_ftun_control(const struct ftun_control *control)
{
    int offset = control->input_select ? FTUN_FIR_OFFSET : FTUN_ADC_OFFSET;
    unsigned int channel = (unsigned int) control->bunch & 3;
    unsigned int bunch = subtract_offset(
        (unsigned int) control->bunch >> 2, offset, BUNCHES_PER_TURN/4);

    LOCK();
    config_space->ftune_control =
        ((control->dwell - 1) & 0xFFFF) |       // bits 15:0
        control->blanking << 16 |               //      16
        control->multibunch << 17 |             //      17
        (channel & 3) << 18 |                   //      19:18
        (bunch & 0x1FF) << 20;                  //      27:20
    config_space->ftune_target =
        (control->target_phase & 0x3FFFF) |     // bits 17:0
        (control->iir_rate & 0x7) << 18 |       //      20:18
        (control->input_select & 0x1) << 28 |   //      28
        (control->det_gain & 0x7) << 29;        //      31:29
    config_space->ftune_i_scale = (uint32_t) -control->i_scale;
    config_space->ftune_min_mag =
        (control->min_magnitude & 0xFFFF) |
        (control->iq_iir_rate & 0x7) << 16 |
        (control->freq_iir_rate & 0x7) << 22;
    config_space->ftune_max_offset = control->max_offset;
    config_space->ftune_p_scale = (uint32_t) -control->p_scale;
    UNLOCK();
}

void hw_write_ftun_enable(bool enable)
{
    WRITE_CONTROL_BITS(1, 1, enable);
}

void hw_write_ftun_start(void)
{
    pulse_control_bit(12);
}

void hw_write_ftun_arm(void)
{
    pulse_control_bit(14);
}

void hw_write_ftun_disarm(void)
{
    pulse_control_bit(15);
}

enum ftun_status hw_read_ftun_status(void)
{
    bool armed = READ_STATUS_BITS(25, 1);
    bool running = config_space->ftune_status & (1 << FTUN_STAT_RUNNING);
    if (running)
        return FTUN_RUNNING;
    else if (armed)
        return FTUN_ARMED;
    else
        return FTUN_STOPPED;
}


void hw_read_ftun_status_bits(bool *status)
{
    LOCK();
    config_space->ftune_read_control = 1;
    bits_to_bool_array(FTUN_BIT_COUNT, status, config_space->ftune_status);
    UNLOCK();
}

static void read_signed_pair(uint32_t pair, int *low, int *high)
{
    *low = (int) (pair << 16) >> 16;
    *high = (int) pair >> 16;
}

void hw_read_ftun_iq(int *ftun_i, int *ftun_q)
{
    read_signed_pair(config_space->ftune_iq, ftun_i, ftun_q);
}

bool hw_read_ftun_frequency(int *frequency)
{
    uint32_t freq_word = config_space->ftune_freq_offset;
    *frequency = (int) (freq_word << 2) >> 2;
    return freq_word >> 31;
}

bool hw_read_ftun_i_minmax(int *min, int *max)
{
    LOCK();
    config_space->ftune_read_control = 2;
    read_signed_pair(config_space->ftune_i_minmax, min, max);
    UNLOCK();
    return *max >= *min;
}

bool hw_read_ftun_q_minmax(int *min, int *max)
{
    LOCK();
    config_space->ftune_read_control = 4;
    read_signed_pair(config_space->ftune_q_minmax, min, max);
    UNLOCK();
    return *max >= *min;
}


static size_t read_ftun_buffer_word(int *buffer, bool *dropout)
{
    /* Each word read has the following bit fields:
     *  17:0    Payload (current frequency offset)
     *  30:21   Words remaining in FIFO
     *  31      Set if FIFO overrun detected. */
    uint32_t word = config_space->ftune_readout;
    *dropout |= word >> 31;
    size_t fifo_entries = (word >> 21) & 0x3FF;
    if (fifo_entries)
        *buffer = (int) (word << 14) >> 14;
    return fifo_entries;
}

size_t hw_read_ftun_buffer(int buffer[FTUN_FIFO_SIZE], bool *dropout)
{
    /* Because of the strange interface between the processor and the FPGA the
     * readout tends to be one sample behind.  This means we must start by
     * always reading at least two words, as the first word read can be pretty
     * old. */
    *dropout = false;
    size_t read_count = 0;
    if (read_ftun_buffer_word(&buffer[read_count], dropout))
        read_count += 1;

    /* Now read until the FIFO is empty or our buffer is full. */
    while (read_count < FTUN_FIFO_SIZE  &&
           read_ftun_buffer_word(&buffer[read_count], dropout))
        read_count += 1;
    return read_count;
}

void hw_read_ftun_delays(int *adc_delay, int *fir_delay)
{
    *adc_delay = FTUN_ADC_DELAY;
    *fir_delay = FTUN_FIR_DELAY;
}


/* * * * * * * * * * * * * * * * * * * * * */
/* SEQ: Programmed Bunch and Sweep Control */

static unsigned int sequencer_pc;
static unsigned int seq_trig_source;

void hw_write_seq_entries(
    unsigned int bank0, const struct seq_entry entries[MAX_SEQUENCER_COUNT])
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
        const struct seq_entry *entry = &entries[i];
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

void hw_write_seq_count(unsigned int pc)
{
    LOCK();
    sequencer_pc = pc;
    write_control_bits(3, 3, sequencer_pc);
    UNLOCK();
}

void hw_write_seq_trig_source(unsigned int source)
{
    LOCK();
    seq_trig_source = source;
    if (seq_trig_source == SEQ_DISABLED)
        write_control_bits(3, 3, 0);
    else
        write_control_bits(3, 3, sequencer_pc);
    write_control_bits(7, 1, source - 1);
    UNLOCK();
}

void hw_write_seq_trig_state(unsigned int state)
{
    WRITE_CONTROL_BITS(12, 3, state);
}

unsigned int hw_read_seq_state(void)
{
    return READ_STATUS_BITS(16, 3);
}

unsigned int hw_read_seq_super_state(void)
{
    return config_space->super_count_r;
}

void hw_read_seq_status(bool *busy, enum seq_trig_source *trig_source)
{
    *busy = READ_STATUS_BITS(22, 1);
    *trig_source = seq_trig_source;
}

void hw_write_seq_reset(void)
{
    pulse_control_bit(7);
}

void hw_write_seq_super_state(
    unsigned int super_count, const uint32_t offsets[SUPER_SEQ_STATES])
{
    ASSERT_OK(0 < super_count  &&  super_count <= SUPER_SEQ_STATES);

    LOCK();
    config_space->super_count = super_count - 1;
    config_space->write_select = 2;     // Select sequencer offset memory
    /* When writing the offsets memory we have to write in reverse order to
     * match the fact that states will be read from count down to 0, and we
     * only need to write the states that will actually be used. */
    for (unsigned int i = 0; i < super_count; i ++)
        config_space->sequencer_write = offsets[super_count - 1 - i];
    UNLOCK();
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

void hw_write_trg_seq_source(unsigned int source)
{
    WRITE_CONTROL_BITS(7, 1, source);
}

void hw_write_trg_disarm(bool ddr, bool buf)
{
    pulse_mask(make_mask2(5, 6, ddr, buf));
}

void hw_write_trg_ddr_delay(unsigned int ddr_delay)
{
    config_space->ddr_trigger_delay = ddr_delay;
}

void hw_write_trg_buf_delay(unsigned int buf_delay)
{
    config_space->buf_trigger_delay = buf_delay;
}

void hw_write_trg_blanking(unsigned int trigger_blanking)
{
    config_space->trigger_blanking = trigger_blanking;
}

void hw_write_trg_blanking_source(unsigned int source)
{
    WRITE_CONTROL_BITS(9, 1, source);
}

void hw_write_trg_arm_raw_phase(void)
{
    pulse_control_bit(11);
}

unsigned int hw_read_trg_raw_phase(void)
{
    return READ_STATUS_BITS(0, 4);
}

void hw_write_trg_ddr_source(const bool source[DDR_SOURCE_COUNT])
{
    WRITE_CONTROL_BITS_2(12, 5, bool_array_to_bits(DDR_SOURCE_COUNT, source));
}

void hw_write_trg_ddr_blanking(const bool blanking[DDR_SOURCE_COUNT])
{
    WRITE_CONTROL_BITS_2(17, 5, bool_array_to_bits(DDR_SOURCE_COUNT, blanking));
}

void hw_read_trg_ddr_source(bool source[DDR_SOURCE_COUNT])
{
    bits_to_bool_array(DDR_SOURCE_COUNT, source, READ_STATUS_BITS(8, 5));
}

void hw_write_trg_buf_source(const bool source[BUF_SOURCE_COUNT])
{
    WRITE_CONTROL_BITS_2(24, 3, bool_array_to_bits(BUF_SOURCE_COUNT, source));
}

void hw_write_trg_buf_blanking(const bool blanking[BUF_SOURCE_COUNT])
{
    WRITE_CONTROL_BITS_2(27, 3, bool_array_to_bits(BUF_SOURCE_COUNT, blanking));
}

void hw_read_trg_buf_source(bool source[BUF_SOURCE_COUNT])
{
    bits_to_bool_array(BUF_SOURCE_COUNT, source, READ_STATUS_BITS(13, 3));
}


/******************************************************************************/

bool initialise_hardware(const char *config_file, unsigned int expected_version)
{
    // Ensure BUNCHES_PER_TURN is a multiple of 4.
    COMPILE_ASSERT(BUNCHES_PER_TURN == ATOMS_PER_TURN * SAMPLES_PER_ATOM);

    hardware_config_file = config_file;
    int mem;
    bool ok =
        config_parse_file(
            config_file, hardware_config_defs,
            ARRAY_SIZE(hardware_config_defs))  &&
        TEST_IO(mem = open("/dev/mem", O_RDWR | O_SYNC))  &&
        TEST_IO(config_space = mmap(
            0, CONTROL_AREA_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
            mem, TMBF_CONFIG_ADDRESS));
    if (ok)
    {
        hw_write_bun_zero_bunch(0);

        uint32_t version = config_space->fpga_version;
        fpga_version            = read_bit_field(version, 0, 16);
        fir_filter_length       = read_bit_field(version, 16, 4);
        unsigned int max_bunches =
            SAMPLES_PER_ATOM * (1U << read_bit_field(version, 20, 4));
        printf(
            "FPGA version %04x, %u taps, %u max bunches, %u bunches per turn\n",
            fpga_version, fir_filter_length, max_bunches, BUNCHES_PER_TURN);
        ok =
            TEST_OK_((fpga_version & 0xFFF0) == expected_version,
                "FPGA version %04x seen, expected version %04x",
                fpga_version, expected_version)  &&
            TEST_OK_(BUNCHES_PER_TURN <= max_bunches,
                "Too many bunches per turn for FPGA");
    }
    return ok;
}
