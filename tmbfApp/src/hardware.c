#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <pthread.h>

#include "test_error.h"
#include "hardware.h"



#define TMBF_CONFIG_ADDRESS       0x1402C000
#define TMBF_DATA_ADDRESS         0x14028000

typedef struct 
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
} TMBF_CONFIG_SPACE;

/* These three pointers directly overlay the FF memory. */
static TMBF_CONFIG_SPACE * ConfigSpace;
static int * DataSpace;



static pthread_mutex_t global_lock = PTHREAD_MUTEX_INITIALIZER;

static void Lock()
{
    pthread_mutex_lock(&global_lock);
}

static void Unlock()
{
    pthread_mutex_unlock(&global_lock);
}



/*****************************************************************************/
/*                      Direct register access definitions                   */
/*****************************************************************************/

/* Hardware field definitions.  These are offset, length pairs specifically
 * designed to be used with the ReadBitField and WriteBitField routines
 * below. */

#define CTRL_DAC_OUT_FIELD        0, 2
#define CTRL_FIR_INVERT_FIELD     2, 1
#define CTRL_SOFT_TRIG_FIELD      3, 1
#define CTRL_ARCHIVE_FIELD        4, 2
#define CTRL_FIR_GAIN_FIELD       6, 4
#define CTRL_HOM_GAIN_FIELD       10, 4
#define CTRL_TRIG_SEL_FIELD       14, 1
#define CTRL_ARM_SEL_FIELD        15, 1
#define CTRL_SOFT_ARM_FIELD       16, 1
#define CTRL_GROW_DAMP_FIELD      17, 1
#define CTRL_TEMP_DAC_OUT_FIELD   18, 3
#define CTRL_DDR_INPUT_FIELD      21, 1
#define CTRL_SET_PLANE_FIELD      22, 1
#define CTRL_CH_SELECT_FIELD      23, 2
#define CTRL_DDC_INPUT_FIELD      25, 1
#define CTRL_BUNCH_MODE_FIELD     26, 1
#define CTRL_IQ_SCALE_FIELD       27, 3
#define CTRL_BUNCH_SYNC_FIELD     30, 1

#define DELAY_DAC_FIELD           0, 10
#define DELAY_TUNE_SWEEP_FIELD    18, 1
#define DELAY_GROW_DAMP_FIELD     24, 8



/* Writes to a sub field of a register by reading the register and writing
 * the appropriately masked value. */
static void WriteBitField(
    unsigned int *Register, unsigned int startbit, unsigned int numbits,
    unsigned int value)
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

#define DEFINE_BIT_FIELD(name, register, start_count) \
    DEFINE_BIT_FIELD_R(name, register, start_count) \
    DEFINE_BIT_FIELD_W(name, register, start_count) 

#define CTRL_REGISTER(name) \
    DEFINE_BIT_FIELD(name, ConfigSpace->Ctrl, name##_FIELD)
    
#define DELAY_REGISTER(name) \
    DEFINE_BIT_FIELD(name, ConfigSpace->Delay, name##_FIELD)
    

/* Direct access to control register fields. */
CTRL_REGISTER(CTRL_DAC_OUT)
CTRL_REGISTER(CTRL_FIR_INVERT)
CTRL_REGISTER(CTRL_SOFT_TRIG)
CTRL_REGISTER(CTRL_ARCHIVE)
CTRL_REGISTER(CTRL_FIR_GAIN)
CTRL_REGISTER(CTRL_HOM_GAIN)
CTRL_REGISTER(CTRL_TRIG_SEL)
CTRL_REGISTER(CTRL_ARM_SEL)
CTRL_REGISTER(CTRL_SOFT_ARM)
CTRL_REGISTER(CTRL_GROW_DAMP)
CTRL_REGISTER(CTRL_TEMP_DAC_OUT)
CTRL_REGISTER(CTRL_DDR_INPUT)
CTRL_REGISTER(CTRL_SET_PLANE)
CTRL_REGISTER(CTRL_CH_SELECT)
CTRL_REGISTER(CTRL_DDC_INPUT)
CTRL_REGISTER(CTRL_BUNCH_MODE)
CTRL_REGISTER(CTRL_IQ_SCALE)
CTRL_REGISTER(CTRL_BUNCH_SYNC)

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

void write_FIR_coeffs(const int coeffs[MAX_FIR_COEFFS])
{
    Lock();
    WRITE_CTRL(SET_PLANE, 1);
    for (int i = 0; i < MAX_FIR_COEFFS; i ++)
        ConfigSpace->FIR_coeffs[i] = coeffs[i];
    Unlock();
}


/* Both ADC and data buffer memories are packed into alternating 16-bit
 * words, but have to be read as 32-bit words.  This structure is used for
 * unpacking. */
typedef union
{
    struct
    {
        short int lower;
        short int upper;
    };
    int packed;
} PACKED_DATA;


void read_ADC_MinMax(
    short ADC_min[MAX_BUNCH_COUNT], short ADC_max[MAX_BUNCH_COUNT])
{
    Lock();
    for (int n = 0; n < 4; n++)
    {
        /* Channel select and read enable for ADC */
        ConfigSpace->AdcChnSel = 2*n + 1; 
        for (int i = 0; i < MAX_BUNCH_COUNT/4; i++)
        {
            PACKED_DATA packed;
            packed.packed = ConfigSpace->BB_Adc_MinMax[i];
            ADC_max[4*i + n] = packed.upper;
            ADC_min[4*i + n] = packed.lower;
        }
    }
    ConfigSpace->AdcChnSel = 0;
    Unlock();
}

/* copy paste of ADC buffer below, with rename to DAC, surely there's a better
 * way...*/
void read_DAC_MinMax(
    short DAC_min[MAX_BUNCH_COUNT], short DAC_max[MAX_BUNCH_COUNT])
{
    Lock();
    for (int n = 0; n < 4; n++)
    {
        /* Channel select and read enable for DAC */
        ConfigSpace->DacChnSel = 2*n + 1; 
        for (int i = 0; i < MAX_BUNCH_COUNT/4; i++)
        {
            PACKED_DATA packed;
            packed.packed = ConfigSpace->BB_Dac_MinMax[i];
            DAC_max[4*i + n] = packed.upper;
            DAC_min[4*i + n] = packed.lower;
        }
    }
    ConfigSpace->DacChnSel = 0;
    Unlock();
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
            PACKED_DATA packed;
            packed.packed = DataSpace[i];
            LowData [4*i + n] = packed.lower;
            HighData[4*i + n] = packed.upper;
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
            PACKED_DATA packed;
            packed.packed = ConfigSpace->BB_Gain_Coeffs[i];
            packed.lower = gains[4*n + i];
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
            PACKED_DATA packed;
            packed.packed = ConfigSpace->BB_Gain_Coeffs[i];
            packed.upper = dacs[4*n + i];
            ConfigSpace->BB_Gain_Coeffs[i] = packed.packed;
        }
    }
    Unlock();
}



/*****************************************************************************/
/*                          Other Interface Routines                         */
/*****************************************************************************/


void set_softTrigger()
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

void set_bunchSync()
{
    Lock();
    WRITE_CTRL(BUNCH_SYNC,  0);
    usleep(1000);
    WRITE_CTRL(BUNCH_SYNC,  1);
    usleep(1000);
    WRITE_CTRL(BUNCH_SYNC,  0);
    Unlock();
}



void dump_registers()
{
    unsigned int * registers = (unsigned int *) ConfigSpace;
    for (int i = 0; i < 4; i ++)
    {
        printf("%04x  ", 0x20 * i);
        for (int j = 0; j < 8; j ++)
            printf(" %08x", registers[8*i + j]);
        printf("\n");
    }

    unsigned int ctrl = ConfigSpace->Ctrl;
    printf("Ctrl: %d %d %d %d  %d %d %d %d  %d %d %d %d  %d %d %d %d  %d %d\n",
        ReadBitField(ctrl, CTRL_DAC_OUT_FIELD),
        ReadBitField(ctrl, CTRL_FIR_INVERT_FIELD),
        ReadBitField(ctrl, CTRL_SOFT_TRIG_FIELD),
        ReadBitField(ctrl, CTRL_ARCHIVE_FIELD),
        
        ReadBitField(ctrl, CTRL_FIR_GAIN_FIELD),
        ReadBitField(ctrl, CTRL_HOM_GAIN_FIELD),
        ReadBitField(ctrl, CTRL_TRIG_SEL_FIELD),
        ReadBitField(ctrl, CTRL_ARM_SEL_FIELD),
        
        ReadBitField(ctrl, CTRL_SOFT_ARM_FIELD),
        ReadBitField(ctrl, CTRL_GROW_DAMP_FIELD),
        ReadBitField(ctrl, CTRL_TEMP_DAC_OUT_FIELD),
        ReadBitField(ctrl, CTRL_DDR_INPUT_FIELD),
        
        ReadBitField(ctrl, CTRL_SET_PLANE_FIELD),
        ReadBitField(ctrl, CTRL_CH_SELECT_FIELD),
        ReadBitField(ctrl, CTRL_DDC_INPUT_FIELD),
        ReadBitField(ctrl, CTRL_BUNCH_MODE_FIELD),
        
        ReadBitField(ctrl, CTRL_IQ_SCALE_FIELD),
        ReadBitField(ctrl, CTRL_BUNCH_SYNC_FIELD));
    unsigned int delay = ConfigSpace->Delay;
    printf("Delay: %d %d %d\n",
        ReadBitField(delay, DELAY_DAC_FIELD),
        ReadBitField(delay, DELAY_TUNE_SWEEP_FIELD),
        ReadBitField(delay, DELAY_GROW_DAMP_FIELD));
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

static bool MapTmbfMemory()
{
    int mem;
    char * map_config_base;
    char * map_data_base;
    return
        TEST_IO(mem, open, "/dev/mem", O_RDWR | O_SYNC)  &&
        TEST_IO(map_config_base, mmap,
            0, CONTROL_AREA_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
            mem, TMBF_CONFIG_ADDRESS & ~OsPageMask)  &&
        TEST_IO(map_data_base, mmap,
            0, DATA_AREA_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
            mem, TMBF_DATA_ADDRESS & ~OsPageMask)  &&
        DO_(
            ConfigSpace = (TMBF_CONFIG_SPACE *)
                (map_config_base + (TMBF_CONFIG_ADDRESS & OsPageMask));
            DataSpace = (int *)
                (map_data_base + (TMBF_DATA_ADDRESS & OsPageMask)));
}


bool InitialiseHardware()
{
    OsPageSize = getpagesize();
    OsPageMask = OsPageSize - 1;
    
    return MapTmbfMemory();
}
