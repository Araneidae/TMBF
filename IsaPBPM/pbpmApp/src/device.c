#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <math.h>
#include <stddef.h>

#include "hardware.h"
#include "test_error.h"
#include "epics_device.h"

#include "device.h"

/*
 * Read History Buffer
 */

HB_DMA_FRAME waveform_buffer[XBPM_HB_WFORM_SIZE];

void read_waveform_buffer(void)
{
    read_all_hb(waveform_buffer);
}

bool read_hb_field(void *context, int *array, size_t max_length, size_t *new_length)
{
    int offset = (int) context;
    printf("Reading individual buffer\n");
    HB_DMA_FRAME *field_array = (HB_DMA_FRAME *) ((char *) waveform_buffer + offset);
    for (int i = 0; i < XBPM_HB_WFORM_SIZE; i++)
        *array++ = (field_array++)->A[0];
    *new_length = XBPM_HB_WFORM_SIZE;
    return true;
}

PUBLISH_METHOD("FA:PROC", read_waveform_buffer);
#define PUBLISH_ABCDSQXY(name, field) \
    PUBLISH_WAVEFORM(int, name, read_hb_field, XBPM_HB_WFORM_SIZE, \
    .context = (void *) offsetof(HB_DMA_FRAME, field))
PUBLISH_ABCDSQXY("FA0:A", A[0])
PUBLISH_ABCDSQXY("FA0:B", B[0])
PUBLISH_ABCDSQXY("FA0:C", C[0])
PUBLISH_ABCDSQXY("FA0:D", D[0])
PUBLISH_ABCDSQXY("FA0:S", S[0])
PUBLISH_ABCDSQXY("FA0:Q", Q[0])
PUBLISH_ABCDSQXY("FA0:X", X[0])
PUBLISH_ABCDSQXY("FA0:Y", Y[0])
PUBLISH_ABCDSQXY("FA1:A", A[1])
PUBLISH_ABCDSQXY("FA1:B", B[1])
PUBLISH_ABCDSQXY("FA1:C", C[1])
PUBLISH_ABCDSQXY("FA1:D", D[1])
PUBLISH_ABCDSQXY("FA1:S", S[1])
PUBLISH_ABCDSQXY("FA1:Q", Q[1])
PUBLISH_ABCDSQXY("FA1:X", X[1])
PUBLISH_ABCDSQXY("FA1:Y", Y[1])


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                            ??????????????????                             */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* Wrappers for hardware access functions to make them amenable to EPICS
 * export. */


#define PUBLISH_REGISTER_R(record, name, register) \
    PUBLISH_SIMPLE_READ (record, name, read_##register) 
#define PUBLISH_REGISTER_W(record, name, register) \
    PUBLISH_SIMPLE_WRITE_INIT(record, name, write_##register, read_##register) 

/* Register access PVs have a very consistent style. */
/*
#define GENERIC_REGISTER(pv_name, field) \
    PUBLISH_REGISTER_W(longout, pv_name "_W", field) \
    PUBLISH_REGISTER_R(longin,  pv_name "_R", field)
*/

/* CTRL space */
PUBLISH_REGISTER_W(longout, "FAI:CFGVAL",           fai_cfg_val)
PUBLISH_REGISTER_W(mbbo,    "FAI:XPOSSEL",          fai_xpos_sel)
PUBLISH_REGISTER_W(mbbo,    "FAI:YPOSSEL",          fai_ypos_sel)

/* FA */
PUBLISH_REGISTER_W(longout, "FA0:XOFS",             fa0_x_ofs)
PUBLISH_REGISTER_W(longout, "FA0:YOFS",             fa0_y_ofs)
PUBLISH_REGISTER_W(longout, "FA0:QOFS",             fa0_q_ofs)
PUBLISH_REGISTER_W(longout, "FA0:KX",               fa0_k_x)
PUBLISH_REGISTER_W(longout, "FA0:KY",               fa0_k_y)
PUBLISH_REGISTER_W(longout, "FA0:DBG",              fa0_dbg)
PUBLISH_REGISTER_W(bo,      "FA0:ADCOUT",           fa0_adcout)

/* SA space */
PUBLISH_METHOD("SA0:RESET",  ResetSA)

PUBLISH_REGISTER_W(longout, "SA0:CLEAR",            sa0_clear)
PUBLISH_REGISTER_R(longin,  "SA0:XPOS",             sa0_xpos)
PUBLISH_REGISTER_R(longin,  "SA0:YPOS",             sa0_ypos)
PUBLISH_REGISTER_R(longin,  "SA0:TEST",             sa0_test)

/* CC STATUS space */

/* The ProcessRead function updates the LinkUp array.  All the other
 * fields can be read directly by EPICS, as no special synchronisation or
 * other treatment is required. */
PUBLISH_METHOD("FF:PROCESS", ProcessRead)

PUBLISH_REGISTER_W(longout, "FF:BPMID",             cc_bpmid)
PUBLISH_REGISTER_W(longout, "FF:FRAMELEN",          cc_framelen)
PUBLISH_REGISTER_W(bo,      "FF:LINK1:ENABLE",      cc_powerdown)
PUBLISH_REGISTER_W(mbbo,    "FF:LINK1:LOOPBACK",    cc_loopback)

PUBLISH_REGISTER_W(bo,      "FF:DATA_SELECT",       cc_dataselect)

PUBLISH_REGISTER_R(longin,  "FF:VERSION",           cc_version)
PUBLISH_REGISTER_R(bi,      "FF:LINK1:TX_UP",       cc_link1_txup)
PUBLISH_REGISTER_R(bi,      "FF:LINK1:RX_UP",       cc_link1_rxup)
PUBLISH_REGISTER_R(longin,  "FF:LINK1:PARTNER",     cc_link1_partner)
PUBLISH_REGISTER_R(longin,  "FF:LINK2:PARTNER",     cc_link2_partner)
PUBLISH_REGISTER_R(longin,  "FF:LINK3:PARTNER",     cc_link3_partner)
PUBLISH_REGISTER_R(longin,  "FF:LINK4:PARTNER",     cc_link4_partner)
PUBLISH_REGISTER_R(longin,  "FF:TIMEFRAME",         cc_tframe_counter)
PUBLISH_REGISTER_R(longin,  "FF:BPM_COUNT",         cc_bpmcount)
PUBLISH_REGISTER_R(longin,  "FF:LINK1:RX_CNT",      cc_link1_rx_cnt)
PUBLISH_REGISTER_R(longin,  "FF:LINK1:TX_CNT",      cc_link1_tx_cnt)
PUBLISH_REGISTER_R(longin,  "FF:LINK1:HARD_ERR",    cc_link1_herror)
PUBLISH_REGISTER_R(longin,  "FF:LINK1:SOFT_ERR",    cc_link1_serror)
PUBLISH_REGISTER_R(longin,  "FF:LINK1:FRAME_ERR",   cc_link1_ferror)
PUBLISH_REGISTER_R(longin,  "FF:PROCESS_TIME",      cc_process_time)


#ifndef __DEFINE_EPICS__
#include "device.EPICS"
#endif

int GenericInit()
{
    printf("Registering Generic Device functions\n");
    return PUBLISH_EPICS_DATA();
}
