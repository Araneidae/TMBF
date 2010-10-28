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

#define XBPM_SBC_IF_BASEADDR    0x7C200000
#define XBPM_SBC_IF_HIGHADDR    0x7C201FFF
#define XBPM_SBC_IF_MEMSIZE     (XBPM_SBC_IF_HIGHADDR - XBPM_SBC_IF_BASEADDR)

#define SBC_CTRL_OFFSET          0x0
#define SBC_FA_OFFSET           (0x200/4)
#define SBC_SA_OFFSET           (0x400/4)
#define SBC_CC_CTRL_OFFSET      (0x1000/4)
#define SBC_CC_STATUS_OFFSET    (0x1C00/4)

typedef struct
{
    unsigned int fai_cfg_val;
    unsigned int fai_pos_sel;
} SBC_CTRL_SPACE;


typedef struct
{
    int fa0_x_ofs;
    int fa0_y_ofs;
    int fa0_q_ofs;
    int fa0_k_x;
    int fa0_k_y;
    int fa0_dbg;
    int fa0_adcout;
} SBC_FA_SPACE;

typedef struct
{
    int sa0_clear;
    int sa0_xpos;
    int sa0_ypos;
    int sa0_test;
} SBC_SA_SPACE;

typedef struct 
{
    int cc_bpmid;
    int cc_framelen;
    int cc_powerdown;
    int cc_loopback;
} SBC_CC_CTRL_SPACE;

typedef struct 
{
    int cc_version;
    int cc_status;
    int cc_link1_partner;
    int cc_link2_partner;
    int cc_link3_partner;
    int cc_link4_partner;
    int cc_linkup;
    int cc_tframe_counter;
    int cc_link1_herror;
    int cc_link2_herror;
    int cc_link3_herror;
    int cc_link4_herror;
    int cc_link1_serror;
    int cc_link2_serror;
    int cc_link3_serror;
    int cc_link4_serror;
    int cc_link1_ferror;
    int cc_link2_ferror;
    int cc_link3_ferror;
    int cc_link4_ferror;
    int cc_link1_rx_cnt;
    int cc_link2_rx_cnt;
    int cc_link3_rx_cnt;
    int cc_link4_rx_cnt;
    int cc_link1_tx_cnt;
    int cc_link2_tx_cnt;
    int cc_link3_tx_cnt;
    int cc_link4_tx_cnt;
    int cc_process_time;
    int cc_bpmcount;
} SBC_CC_STATUS_SPACE;


static int * sbc_if_baseaddr;
static SBC_CTRL_SPACE       * sbc_ctrl_addr;
static SBC_FA_SPACE         * sbc_fa_addr;
static SBC_SA_SPACE         * sbc_sa_addr;
static SBC_CC_CTRL_SPACE    * sbc_cc_ctrl_addr;
static SBC_CC_STATUS_SPACE  * sbc_cc_status_addr;


/* Boolean values extracted from LinkUp field.  This array is updated once a
 * second. */
static bool TxLinkUp[4];
static bool RxLinkUp[4];

/* Mirrors of configuration values.  These values cannot be read and written
 * directly, and so are instead read and written here. */
static bool DataSourceSelect = false;
static bool GlobalEnable = true;
static bool LinkEnable[4] = { true, true, true, true };
static int LoopBack[4] = { 0, 0, 0, 0 };


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

#define DIRECT_REGISTER_R(space, name) \
    DECLARE_REGISTER_R(name) \
    { \
        Lock(); \
        unsigned int result = space->name; \
        Unlock(); \
        return result; \
    }

#define DIRECT_REGISTER_W(space, name) \
    DECLARE_REGISTER_W(name) \
    { \
        Lock(); \
        space->name = value; \
        Unlock(); \
    }

#define DIRECT_REGISTER(space, name) \
    DIRECT_REGISTER_R(space, name) \
    DIRECT_REGISTER_W(space, name)


/* CTRL */
DIRECT_REGISTER(sbc_ctrl_addr, fai_cfg_val)

/* FA */
DIRECT_REGISTER(sbc_fa_addr, fa0_x_ofs)
DIRECT_REGISTER(sbc_fa_addr, fa0_y_ofs)
DIRECT_REGISTER(sbc_fa_addr, fa0_q_ofs)
DIRECT_REGISTER(sbc_fa_addr, fa0_k_x)
DIRECT_REGISTER(sbc_fa_addr, fa0_k_y)
DIRECT_REGISTER(sbc_fa_addr, fa0_dbg)
DIRECT_REGISTER(sbc_fa_addr, fa0_adcout)

/* SA */
DIRECT_REGISTER(sbc_sa_addr,   sa0_clear)
DIRECT_REGISTER_R(sbc_sa_addr, sa0_xpos)
DIRECT_REGISTER_R(sbc_sa_addr, sa0_ypos)
DIRECT_REGISTER_R(sbc_sa_addr, sa0_test)

/* CC STATUS */
DIRECT_REGISTER_R(sbc_cc_status_addr, cc_version)
DIRECT_REGISTER_R(sbc_cc_status_addr, cc_link1_partner)
DIRECT_REGISTER_R(sbc_cc_status_addr, cc_link2_partner)
DIRECT_REGISTER_R(sbc_cc_status_addr, cc_link3_partner)
DIRECT_REGISTER_R(sbc_cc_status_addr, cc_link4_partner)
DIRECT_REGISTER_R(sbc_cc_status_addr, cc_tframe_counter)
DIRECT_REGISTER_R(sbc_cc_status_addr, cc_bpmcount)
DIRECT_REGISTER_R(sbc_cc_status_addr, cc_link1_rx_cnt)
DIRECT_REGISTER_R(sbc_cc_status_addr, cc_link1_tx_cnt)
DIRECT_REGISTER_R(sbc_cc_status_addr, cc_link1_herror)
DIRECT_REGISTER_R(sbc_cc_status_addr, cc_link1_serror)
DIRECT_REGISTER_R(sbc_cc_status_addr, cc_link1_ferror)
DIRECT_REGISTER_R(sbc_cc_status_addr, cc_process_time)

/*****************************************************************************/
/*                    Communication Controller Functions                    */
/*****************************************************************************/
void ResetSA()
{
    Lock();
    sbc_sa_addr-> sa0_clear = 0x1;
    Unlock();
}


/*****************************************************************************/
/*                    Communication Controller Functions                    */
/*****************************************************************************/
/* Combines configured with dynamic values to generate a control register
 * value.  The Handshake bit is used to enable the reading of configuration
 * space values on its rising edge. */

static int ControlValue(bool Handshake)
{
    int ctrl;
    ctrl = (Handshake << 0) | (DataSourceSelect << 1) | (GlobalEnable << 3);
    return ctrl;
}

/* This is called each time the status and monitor fields are about to be
 * read.  We simply update the up bits, as all other fields can be read
 * directly. */

void ProcessRead()
{
    int UpMask;
    Lock();
    UpMask = sbc_cc_status_addr->cc_linkup;
    Unlock();
    for (int i = 0; i < 4; i ++)
    {
        RxLinkUp[i] = (UpMask & (1 << i)) != 0;
        TxLinkUp[i] = (UpMask & (1 << (i + 4))) != 0;
    }
}

/* Called each time a configuration value has changed. */
static void ProcessWrite()
{
    /* For simplicity we assemble the PowerDown and LoopBack values each time
     * any field is written.  This should be harmless enough, as the only
     * side effects happen when the control register handshake is toggled
     * below. */
    sbc_cc_ctrl_addr->cc_powerdown =
        (!LinkEnable[0] << 0) | (!LinkEnable[1] << 1) |
        (!LinkEnable[2] << 2) | (!LinkEnable[3] << 3);
    sbc_cc_ctrl_addr->cc_loopback =
        (LoopBack[0] << 0) | (LoopBack[1] << 2) |
        (LoopBack[2] << 4) | (LoopBack[3] << 6);

    /* Force the configuration values to be read by toggling the handshake
       bit in the control register. */
    Lock();
    sbc_ctrl_addr->fai_cfg_val = ControlValue(true);
    Unlock();
    Lock();
    sbc_ctrl_addr->fai_cfg_val = ControlValue(false);
    Unlock();
}

bool read_cc_link1_rxup()
{
    return TxLinkUp[0];
}

bool read_cc_link1_txup()
{
    return RxLinkUp[0];
}

void write_cc_bpmid(unsigned int value)
{
    sbc_cc_ctrl_addr->cc_bpmid = value;
    ProcessWrite();
}

unsigned int read_cc_bpmid()
{
    return sbc_cc_ctrl_addr->cc_bpmid;
}

void write_cc_framelen(unsigned int value)
{
    sbc_cc_ctrl_addr->cc_framelen = value;
    ProcessWrite();
}

unsigned int read_cc_framelen()
{
    return sbc_cc_ctrl_addr->cc_framelen;
}

void write_cc_powerdown(bool value)
{
    LinkEnable[0] = value;
    ProcessWrite();
}

bool read_cc_powerdown()
{
    int mask;
    mask = sbc_cc_ctrl_addr->cc_powerdown;

    return !((mask & (1 << 0)) != 0);
}

void write_cc_loopback(unsigned int value)
{
    LoopBack[0] = value;
    ProcessWrite();
}

unsigned int read_cc_loopback()
{
    int mask;
    mask = sbc_cc_ctrl_addr->cc_loopback;

    return (mask & 3);
}

void write_cc_dataselect(bool value)
{
    DataSourceSelect = value;
    Lock();
    sbc_ctrl_addr->fai_cfg_val = ControlValue(false);
    Unlock();
}

bool read_cc_dataselect()
{
    int mask;
    mask = sbc_ctrl_addr->fai_cfg_val;

    return ((mask & (1 << 2)) != 0);
}

void write_fai_xpos_sel(unsigned int value)
{
    int mask;
    Lock();
    mask = sbc_ctrl_addr-> fai_pos_sel;
    sbc_ctrl_addr-> fai_pos_sel = (mask & 0xF0) | value;
    Unlock();
}

unsigned int read_fai_xpos_sel()
{
    int mask;
    Lock();
    mask = sbc_ctrl_addr-> fai_pos_sel;
    Unlock();
    return (mask & 0xF);
}

void write_fai_ypos_sel(unsigned int value)
{
    int mask;
    Lock();
    mask = sbc_ctrl_addr-> fai_pos_sel;
    sbc_ctrl_addr-> fai_pos_sel = (mask & 0x0F) | (value << 4);
    Unlock();
}

unsigned int read_fai_ypos_sel()
{
    int mask;
    Lock();
    mask = sbc_ctrl_addr-> fai_pos_sel;
    Unlock();
    return ((mask & 0xF0) >> 4);
}

bool InitialiseCommsController()
{

    Lock();
    sbc_cc_ctrl_addr->cc_bpmid = 254;
    Unlock();
    Lock();
    sbc_cc_ctrl_addr->cc_framelen = 7500;
    Unlock();

    ProcessWrite();
    return true;
}

void read_all_hb(HB_DMA_FRAME waveform_buffer[XBPM_HB_WFORM_SIZE])
{
    int hb;

    printf("\nReading whole history buffer\n");

    if ((hb = open("/dev/xbpm_hb0", O_RDONLY)) <= 0)
    {
        printf("HB device cannot be opened\n");
        exit(1);
    }

    if (read(hb, waveform_buffer, XBPM_HB_WFORM_SIZE * sizeof(HB_DMA_FRAME)) < XBPM_HB_BLOCK_SIZE)
    {
        printf("Unable to read all HB\n");
        exit(1);
    }

    close(hb);

}

/*****************************************************************************/
/*                          Component Initialisation                         */
/*****************************************************************************/

/* Paging information. */
static unsigned int OsPageSize;
static unsigned int OsPageMask;

static bool MapSBCMemory()
{
    int mem;
    char * map_sbc_if_base;

    TEST_IO(mem, open, "/dev/mem", O_RDWR | O_SYNC);
    TEST_IO(map_sbc_if_base, mmap, 0, XBPM_SBC_IF_MEMSIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem, XBPM_SBC_IF_BASEADDR & ~OsPageMask);

    sbc_if_baseaddr = (int *) (map_sbc_if_base + (XBPM_SBC_IF_BASEADDR & OsPageMask));
    printf("Base address = %08x\n", (unsigned int) sbc_if_baseaddr);

    sbc_ctrl_addr = (SBC_CTRL_SPACE *) (sbc_if_baseaddr + SBC_CTRL_OFFSET);
    sbc_fa_addr  = (SBC_FA_SPACE *) (sbc_if_baseaddr + SBC_FA_OFFSET);
    sbc_sa_addr  = (SBC_SA_SPACE *) (sbc_if_baseaddr + SBC_SA_OFFSET);
    sbc_cc_ctrl_addr  = (SBC_CC_CTRL_SPACE *) (sbc_if_baseaddr + SBC_CC_CTRL_OFFSET);
    sbc_cc_status_addr  = (SBC_CC_STATUS_SPACE *) (sbc_if_baseaddr + SBC_CC_STATUS_OFFSET);
    return true;
}


bool InitialiseHardware()
{
    OsPageSize = getpagesize();
    OsPageMask = OsPageSize - 1;

    return MapSBCMemory();
}

