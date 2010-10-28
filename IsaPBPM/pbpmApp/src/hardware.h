/* Abstraction of TMBF register interface.
 *
 * Mostly a very thin wrapper over the existing registers, largely designed
 * so that the hardware interface can be separated from the code that uses
 * it. */

#define XBPM_HB_BLOCK_SHIFT     20      // 2**20 = 1M
#define XBPM_HB_BLOCK_SIZE      (1 << XBPM_HB_BLOCK_SHIFT)
#define XBPM_HB_DMA_SIZE        64
#define XBPM_HB_WFORM_SIZE      (XBPM_HB_BLOCK_SIZE/XBPM_HB_DMA_SIZE)

typedef struct
{
    int A[2];
    int B[2];
    int C[2];
    int D[2];
    int S[2];
    int Q[2];
    int X[2];
    int Y[2];
} HB_DMA_FRAME;



/* Register read access. */
#define DECLARE_REGISTER_R(name) \
    unsigned int read_##name()

/* Register write access. */
#define DECLARE_REGISTER_W(name) \
    void write_##name(unsigned int value)

/* Each configuration DECLARE_REGISTER has read and write access routines. */
#define DECLARE_REGISTER(name) \
    DECLARE_REGISTER_R(name); \
    DECLARE_REGISTER_W(name)

/* Ctrl Address Space Registers */
DECLARE_REGISTER(fai_cfg_val);

/* FA */
DECLARE_REGISTER(fa0_x_ofs);
DECLARE_REGISTER(fa0_y_ofs);
DECLARE_REGISTER(fa0_q_ofs);
DECLARE_REGISTER(fa0_k_x);
DECLARE_REGISTER(fa0_k_y);
DECLARE_REGISTER(fa0_dbg);
DECLARE_REGISTER(fa0_adcout);


/* SA Address Space Registers */
DECLARE_REGISTER(sa0_clear);
DECLARE_REGISTER_R(sa0_xpos);
DECLARE_REGISTER_R(sa0_ypos);
DECLARE_REGISTER_R(sa0_test);

/* CC STATUS */
DECLARE_REGISTER_R(cc_version);
DECLARE_REGISTER_R(cc_link1_partner);
DECLARE_REGISTER_R(cc_link2_partner);
DECLARE_REGISTER_R(cc_link3_partner);
DECLARE_REGISTER_R(cc_link4_partner);
DECLARE_REGISTER_R(cc_tframe_counter);
DECLARE_REGISTER_R(cc_bpmcount);
DECLARE_REGISTER_R(cc_link1_rx_cnt);
DECLARE_REGISTER_R(cc_link1_tx_cnt);
DECLARE_REGISTER_R(cc_link1_herror);
DECLARE_REGISTER_R(cc_link1_serror);
DECLARE_REGISTER_R(cc_link1_ferror);
DECLARE_REGISTER_R(cc_process_time);

/* Read 1M history buffer */
void read_all_hb(HB_DMA_FRAME waveform_buffer[XBPM_HB_WFORM_SIZE]);

bool InitialiseCommsController();
bool InitialiseHardware();
void ResetSA();
void ProcessRead();
bool read_cc_link1_rxup();
bool read_cc_link1_txup();
unsigned int read_cc_bpmid();
void write_cc_bpmid(unsigned int value);
unsigned int read_cc_framelen();
void write_cc_framelen(unsigned int value);
bool read_cc_powerdown();
void write_cc_powerdown(bool value);
unsigned int read_cc_loopback();
void write_cc_loopback(unsigned int value);
bool read_cc_dataselect();
void write_fai_xpos_sel(unsigned int value);
unsigned int read_fai_xpos_sel();
void write_fai_ypos_sel(unsigned int value);
unsigned int read_fai_ypos_sel();
void write_cc_dataselect(bool value);

#undef DECLARE_REGISTER
