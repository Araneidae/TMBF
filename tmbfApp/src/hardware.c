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



#define TMBF_CONFIG_ADDRESS       0x1402C000
#define TMBF_DATA_ADDRESS         0x14028000

struct tmbf_config_space
{
    unsigned int Ctrl;                  // 000  Main control register
    unsigned int BunchSelect;           // 004  Bunch number selection
    unsigned int DDC_dwellTime;         // 008  DDC dwell time (in 8ns clocks)
    unsigned int NCO_frequency;         // 00C  Fixed NCO output frequency
    unsigned int FPGA_version;          // 010  FPGA version number
    unsigned int Delay;                 // 014  DAC output delay (in 2ns steps)
    unsigned int SweepStartFreq;        // 018  Sweep start
    unsigned int SweepStopFreq;         // 01C  --- and stop frequecies
    int          FIR_coeffs[12];        // 020  FIR coefficents
    unsigned int AdcOffAB;              // 050  Used to correct DC offsets
    unsigned int AdcOffCD;              // 054  --- between four ADC channels.
    unsigned int AdcChnSel;             // 058  Select ADC readout channel
    unsigned int DacChnSel;             // 05C  Select DAC readout channel
    unsigned int SweepStep;             // 060  Sweep frequency step
    unsigned int DummyArry[231];        // 064  (padding)
    int          BB_Gain_Coeffs[256];   // 400  Bunch gain control
    int          BB_Adc_MinMax[256];    // 800  ADC readout
    int          BB_Dac_MinMax[256];    // C00  DAC readout
};

/* These three pointers directly overlay the FF memory. */
static struct tmbf_config_space *ConfigSpace;
static int *DataSpace;



static pthread_mutex_t global_lock = PTHREAD_MUTEX_INITIALIZER;

static void Lock(void)
{
    pthread_mutex_lock(&global_lock);
}

static void Unlock(void)
{
    pthread_mutex_unlock(&global_lock);
}



/*****************************************************************************/
/*                      Direct register access definitions                   */
/*****************************************************************************/

/* Hardware field definitions.  These are offset, length pairs specifically
 * designed to be used with the ReadBitField and WriteBitField routines
 * below. */

#define CTRL_DAC_ENA_FIELD        0, 1
#define CTRL_ARM_DDR_FIELD        1
#define CTRL_TRIG_DDR_FIELD       2
#define CTRL_SOFT_TRIG_FIELD      3, 1
#define CTRL_ARCHIVE_FIELD        4, 2
#define CTRL_FIR_GAIN_FIELD       6, 4
#define CTRL_HOM_GAIN_FIELD       10, 4
#define CTRL_TRIG_SEL_FIELD       14, 1
#define CTRL_ARM_SEL_FIELD        15, 1
#define CTRL_SOFT_ARM_FIELD       16, 1
#define CTRL_GROW_DAMP_FIELD      17, 1
#define CTRL_DDR_INPUT_FIELD      21, 1
#define CTRL_CH_SELECT_FIELD      23, 2
#define CTRL_DDC_INPUT_FIELD      25, 1
#define CTRL_IQ_SCALE_FIELD       27, 3
#define CTRL_BUNCH_SYNC_FIELD     30, 1

#define DELAY_DAC_FIELD           0, 10
#define DELAY_TUNE_SWEEP_FIELD    18, 1
#define DELAY_GROW_DAMP_FIELD     24, 8



/* Writes to a sub field of a register by reading the register and writing
 * the appropriately masked value. */
static void WriteBitField(
    volatile unsigned int *Register, unsigned int startbit,
    unsigned int numbits, unsigned int value)
{
    unsigned int mask = ((1 << numbits) - 1) << startbit;
    *Register = (*Register & ~mask) | ((value << startbit) & mask);
}


/* Reads a bit field from a register. */
static unsigned int ReadBitField(
    unsigned int Register, unsigned int startbit, unsigned int numbits)
{
    return (Register >> startbit) & ((1 << numbits) - 1);
}


static void PulseBit(unsigned int *Register, unsigned int startbit)
{
    Lock();
    WriteBitField(Register, startbit, 1, 0);    // Shouldn't be necessary
    WriteBitField(Register, startbit, 1, 1);
    WriteBitField(Register, startbit, 1, 0);
    Unlock();
}


#define DEFINE_BIT_FIELD_R(name, register, start, count) \
    DECLARE_REGISTER_R(name) \
    { \
        Lock(); \
        unsigned int result = ReadBitField(register, start, count); \
        Unlock(); \
        return result; \
    }

#define DEFINE_BIT_FIELD_W(name, register, start, count) \
    DECLARE_REGISTER_W(name) \
    { \
        Lock(); \
        WriteBitField(&register, start, count, value); \
        Unlock(); \
    }

#define DEFINE_PULSE(name, register, start) \
    DECLARE_PULSE(name) \
    { \
        PulseBit(&register, start); \
    }

#define DEFINE_BIT_FIELD(name, register, start_count) \
    DEFINE_BIT_FIELD_R(name, register, start_count) \
    DEFINE_BIT_FIELD_W(name, register, start_count)

#define CTRL_REGISTER(name) \
    DEFINE_BIT_FIELD(name, ConfigSpace->Ctrl, name##_FIELD)

#define CTRL_PULSE(name) \
    DEFINE_PULSE(name, ConfigSpace->Ctrl, name##_FIELD)

#define DELAY_REGISTER(name) \
    DEFINE_BIT_FIELD(name, ConfigSpace->Delay, name##_FIELD)


/* Direct access to control register fields. */
CTRL_REGISTER(CTRL_DAC_ENA)
CTRL_REGISTER(CTRL_SOFT_TRIG)
CTRL_REGISTER(CTRL_ARCHIVE)
CTRL_REGISTER(CTRL_FIR_GAIN)
CTRL_REGISTER(CTRL_HOM_GAIN)
CTRL_REGISTER(CTRL_TRIG_SEL)
CTRL_REGISTER(CTRL_ARM_SEL)
CTRL_REGISTER(CTRL_SOFT_ARM)
CTRL_REGISTER(CTRL_GROW_DAMP)
CTRL_REGISTER(CTRL_DDR_INPUT)
CTRL_REGISTER(CTRL_CH_SELECT)
CTRL_REGISTER(CTRL_DDC_INPUT)
CTRL_REGISTER(CTRL_IQ_SCALE)
CTRL_REGISTER(CTRL_BUNCH_SYNC)

CTRL_PULSE(CTRL_ARM_DDR)
CTRL_PULSE(CTRL_TRIG_DDR)

/* Direct access to delay register fields. */
DELAY_REGISTER(DELAY_DAC)
DELAY_REGISTER(DELAY_TUNE_SWEEP)
DELAY_REGISTER(DELAY_GROW_DAMP)


#define DIRECT_REGISTER_R(name) \
    DECLARE_REGISTER_R(name) \
    { \
        Lock(); \
        unsigned int result = ConfigSpace->name; \
        Unlock(); \
        return result; \
    }

#define DIRECT_REGISTER_W(name) \
    DECLARE_REGISTER_W(name) \
    { \
        Lock(); \
        ConfigSpace->name = value; \
        Unlock(); \
    }

#define DIRECT_REGISTER(name) \
    DIRECT_REGISTER_R(name) \
    DIRECT_REGISTER_W(name)

DIRECT_REGISTER(BunchSelect)
DIRECT_REGISTER(DDC_dwellTime)
DIRECT_REGISTER(NCO_frequency)
DIRECT_REGISTER_R(FPGA_version)
DIRECT_REGISTER(SweepStartFreq)
DIRECT_REGISTER(SweepStopFreq)
DIRECT_REGISTER(SweepStep)
DIRECT_REGISTER(AdcOffAB)
DIRECT_REGISTER(AdcOffCD)


/* Helper routine for control register bit access, as this is a particularly
 * common form. */
#define WRITE_CTRL(field, value) \
    WriteBitField(&ConfigSpace->Ctrl, CTRL_##field##_FIELD, value)



/*****************************************************************************/
/*                          Waveform Memory Access                           */
/*****************************************************************************/


void read_FIR_coeffs(int coeffs[MAX_FIR_COEFFS])
{
    Lock();
    for (int i = 0; i < MAX_FIR_COEFFS; i ++)
        coeffs[i] = ConfigSpace->FIR_coeffs[i];
    Unlock();
}

void write_FIR_coeffs(int coeffs[MAX_FIR_COEFFS])
{
    Lock();
    for (int i = 0; i < MAX_FIR_COEFFS; i ++)
        ConfigSpace->FIR_coeffs[i] = coeffs[i];
    Unlock();
}


/* Both ADC and data buffer memories are packed into alternating 16-bit
 * words, but have to be read as 32-bit words.  This structure is used for
 * unpacking. */
union packed_data
{
    struct
    {
        short int lower;
        short int upper;
    };
    int packed;
};


union bunch_packed_data
{
    struct
    {
        short int gain;
        char      dac;
        char      tempdac;
    };
    int packed;
};


/* Reads packed array of min/max values using readout selector.  Used for ADC
 * and DAC readouts. */
static void read_minmax(
    volatile int *minmax, volatile unsigned int *channel,
    short *min_out, short *max_out)
{
    Lock();
    for (int n = 0; n < 4; n++)
    {
        /* Channel select and read enable for ADC */
        *channel = 2*n + 1;
        for (int i = 0; i < MAX_BUNCH_COUNT/4; i++)
        {
            union packed_data packed;
            packed.packed = minmax[i];
            min_out[4*i + n] = packed.lower;
            max_out[4*i + n] = packed.upper;
        }
    }
    *channel = 0;
    Unlock();
}

void read_ADC_MinMax(
    short ADC_min[MAX_BUNCH_COUNT], short ADC_max[MAX_BUNCH_COUNT])
{
    read_minmax(
        ConfigSpace->BB_Adc_MinMax, &ConfigSpace->AdcChnSel, ADC_min, ADC_max);
}

void read_DAC_MinMax(
    short DAC_min[MAX_BUNCH_COUNT], short DAC_max[MAX_BUNCH_COUNT])
{
    read_minmax(
        ConfigSpace->BB_Dac_MinMax, &ConfigSpace->DacChnSel, DAC_min, DAC_max);
}


void read_DataSpace(
    short int HighData[MAX_DATA_LENGTH], short int LowData[MAX_DATA_LENGTH])
{
    Lock();
    for (int n = 0; n < 4; n++)
    {
        WRITE_CTRL(CH_SELECT, n);
        for (int i = 0; i < MAX_DATA_LENGTH/4; i++)
        {
            union packed_data packed = { .packed = DataSpace[i] };
            LowData [4*i + n] = packed.lower;
            HighData[4*i + n] = packed.upper;
        }
    }
    Unlock();
}

void read_bunch_configs(
    short int bunch_gains[MAX_BUNCH_COUNT],
    short int bunch_dacs[MAX_BUNCH_COUNT],
    short int bunch_tempdacs[MAX_BUNCH_COUNT])
{
    Lock();
    for (int n=0; n < 4; n++)
    {
        WRITE_CTRL(CH_SELECT, n);
        for (int i=0; i < MAX_BUNCH_COUNT/4; i++)
        {
            union bunch_packed_data packed;
            packed.packed = ConfigSpace->BB_Gain_Coeffs[i];
            bunch_gains[4*i + n] = packed.gain;
            bunch_dacs[4*i + n] = packed.dac;
            bunch_tempdacs[4*i + n] = packed.tempdac;
        }
    }
    Unlock();
}


void write_BB_gains(short int gains[MAX_BUNCH_COUNT])
{
    Lock();
    for (int n=0; n < 4; n++)
    {
        WRITE_CTRL(CH_SELECT, n);
        for (int i=0; i < MAX_BUNCH_COUNT/4; i++)
        {
            union bunch_packed_data packed;
            packed.packed = ConfigSpace->BB_Gain_Coeffs[i];
            packed.gain = gains[4*i + n];
            ConfigSpace->BB_Gain_Coeffs[i] = packed.packed;
        }
    }
    Unlock();
}

void write_BB_DACs(short int dacs[MAX_BUNCH_COUNT])
{
    Lock();
    for (int n=0; n < 4; n++)
    {
        WRITE_CTRL(CH_SELECT, n);
        for (int i=0; i < MAX_BUNCH_COUNT/4; i++)
        {
            union bunch_packed_data packed;
            packed.packed = ConfigSpace->BB_Gain_Coeffs[i];
            packed.dac = dacs[4*i + n];
            ConfigSpace->BB_Gain_Coeffs[i] = packed.packed;
        }
    }
    Unlock();
}

void write_BB_TEMPDACs(short int tempdacs[MAX_BUNCH_COUNT])
{
    Lock();
    for (int n=0; n < 4; n++)
    {
        WRITE_CTRL(CH_SELECT, n);
        for (int i=0; i < MAX_BUNCH_COUNT/4; i++)
        {
            union bunch_packed_data packed;
            packed.packed = ConfigSpace->BB_Gain_Coeffs[i];
            packed.tempdac = tempdacs[4*i + n];
            ConfigSpace->BB_Gain_Coeffs[i] = packed.packed;
        }
    }
    Unlock();
}


/*****************************************************************************/
/*                          Other Interface Routines                         */
/*****************************************************************************/


void set_softTrigger(void)
{
    Lock();
    /* The sequence is as follows:
     *  1. Select soft trigger
     *  2. Select soft arming
     *  3. Arm (by writing 1 then 0)
     *  4. Trigger (ditto) */
    WRITE_CTRL(SOFT_ARM,  0);
    usleep(1000);
    WRITE_CTRL(SOFT_ARM,  1);
    usleep(1000);
    WRITE_CTRL(SOFT_ARM,  0);
    usleep(1000);

    WRITE_CTRL(SOFT_TRIG, 0);
    usleep(1000);
    WRITE_CTRL(SOFT_TRIG, 1);
    usleep(1000);
    WRITE_CTRL(SOFT_TRIG, 0);
    Unlock();
}

void set_bunchSync(void)
{
    Lock();
    WRITE_CTRL(BUNCH_SYNC,  0);
    usleep(1000);
    WRITE_CTRL(BUNCH_SYNC,  1);
    usleep(1000);
    WRITE_CTRL(BUNCH_SYNC,  0);
    Unlock();
}



/*****************************************************************************/
/*                          Component Initialisation                         */
/*****************************************************************************/


/* Paging information. */
static unsigned int OsPageSize;
static unsigned int OsPageMask;


/* Mapped area sizes. */
#define CONTROL_AREA_SIZE   (1 << 12)
#define DATA_AREA_SIZE      (1 << 14)

static bool MapTmbfMemory(void)
{
    int mem;
    void *map_config_base;
    void *map_data_base;
    return
        TEST_IO(mem = open("/dev/mem", O_RDWR | O_SYNC))  &&
        TEST_IO(map_config_base = mmap(
            0, CONTROL_AREA_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
            mem, TMBF_CONFIG_ADDRESS & ~OsPageMask))  &&
        TEST_IO(map_data_base = mmap(
            0, DATA_AREA_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
            mem, TMBF_DATA_ADDRESS & ~OsPageMask))  &&
        DO_(
            ConfigSpace =
                (map_config_base + (TMBF_CONFIG_ADDRESS & OsPageMask));
            DataSpace = (map_data_base + (TMBF_DATA_ADDRESS & OsPageMask)));
}


bool InitialiseHardware(void)
{
    OsPageSize = getpagesize();
    OsPageMask = OsPageSize - 1;

    return MapTmbfMemory();
}
