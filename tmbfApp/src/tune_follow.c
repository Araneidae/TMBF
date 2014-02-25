/* Tune following. */

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <string.h>

#include "error.h"
#include "epics_device.h"
#include "hardware.h"

#include "tune_follow.h"


bool initialise_tune_follow(void)
{
    PUBLISH_WRITER_P(longout, "FTUN:DWELL", hw_write_ftun_dwell);
    PUBLISH_WRITER_P(longout, "FTUN:BUNCH", hw_write_ftun_bunch);
    PUBLISH_WRITER_P(bo,      "FTUN:MULTIBUNCH", hw_write_ftun_multibunch);
    PUBLISH_WRITER_P(mbbo,    "FTUN:INPUT", hw_write_ftun_input_select);
    PUBLISH_WRITER_P(mbbo,    "FTUN:GAIN",  hw_write_ftun_det_gain);
    PUBLISH_WRITER_P(bo,      "FTUN:ENABLE", hw_write_ftun_enable);
    return true;
}
