/* Tune following. */

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <string.h>
#include <math.h>

#include "error.h"
#include "epics_device.h"
#include "epics_extra.h"
#include "hardware.h"

#include "tune_follow.h"


#define ROTATE_BITS 14  // Plus two bits for the quadrant

/* Table of rotations, each field is initialised with tan^-1(2^-n) for
 * n:1..ROTATE_BITS. */
static uint32_t cordic_angles[ROTATE_BITS];

/* Almost identical to detector.c:tune_to_freq, just different scaling. */
static uint32_t degrees_to_int32(double angle)
{
    double integral;
    double fraction = modf(angle / 360, &integral);
    if (fraction < 0.0)
        fraction += 1.0;
    return (uint32_t) round(fraction * pow(2, 32));

}

static void initialise_rotation(void)
{
    double tangent = 0.5;
    for (int i = 0; i < ROTATE_BITS; i++)
    {
        double fraction = atan(tangent) / 2 / M_PI;
        cordic_angles[i] = (uint32_t) round(fraction * pow(2, 32));
        tangent /= 2;
    }
}

/* The rotation written to hardware is encoded from the angle as a bit array
 * consisting of a quadrant selector followed by a sequence of rotation
 * direction selections.  Each of the rotation bits corresponds to a rotation of
 * +-tan^-1(2^-n) where n identifies the bit, starting with 1 for the most
 * significant bit, thus:
 *
 *      +----+---+---+---+-----+---+
 *      | qq | 1 | 2 | 3 | ... | n |    (qq is two bits to identify quadrant).
 *      +----+---+---+---+-----+---+
 *
 * The setting of the bit identifies the direction of rotation. */
static void write_rotation(double angle)
{
    uint32_t fraction = degrees_to_int32(angle);
    uint32_t rotate_bits = ((fraction >> 30) + ((fraction >> 29) & 1)) & 0x3;
    int32_t residue = fraction - (rotate_bits << 30);
    for (int i = 0; i < ROTATE_BITS; i ++)
    {
        rotate_bits <<= 1;
        if (residue >= 0)
        {
            residue -= cordic_angles[i];
            rotate_bits |= 1;
        }
        else
            residue += cordic_angles[i];
    }
    hw_write_ftun_rotation(rotate_bits);
}


bool initialise_tune_follow(void)
{
    initialise_rotation();

    PUBLISH_WRITER_P(longout, "FTUN:DWELL", hw_write_ftun_dwell);
    PUBLISH_WRITER_P(longout, "FTUN:BUNCH", hw_write_ftun_bunch);
    PUBLISH_WRITER_P(bo,      "FTUN:MULTIBUNCH", hw_write_ftun_multibunch);
    PUBLISH_WRITER_P(mbbo,    "FTUN:INPUT", hw_write_ftun_input_select);
    PUBLISH_WRITER_P(bo,      "FTUN:ENABLE", hw_write_ftun_enable);
    PUBLISH_WRITER_P(ao,      "FTUN:ANGLE", write_rotation);
    return true;
}
