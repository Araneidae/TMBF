/* This file contains definitions for which there is no other natural home. */

/* This function converts an array of floats into the corresponding array of
 * integer values by multiplying each value by 2^(bits-scale_bits-1).  The
 * floating point values are clipped to the extreme possible values as
 * determined by bits.
 *    The parameter bits determines the total number of bits available, so the
 * output will be in the range [-2^(bits-1)..2^(bits-1)-1].  The parameter
 * scale_bits determines the range of valid input values, so the input should be
 * in the range [-2^scale_bits..2^scale_bits). */
static inline void float_array_to_int(
    size_t count, float in[], int out[], int bits, int scale_bits)
{
    int fraction_bits = bits - scale_bits - 1;
    float scaling = (float) (1 << fraction_bits);
    float max_val = (float) (1 << scale_bits) - 1.0F / scaling;
    float min_val = - (float) (1 << scale_bits);
    for (size_t i = 0; i < count; i ++)
    {
        if (in[i] > max_val)
            in[i] = max_val;
        else if (in[i] < min_val)
            in[i] = min_val;
        out[i] = lroundf(in[i] * scaling);
        in[i] = (float) out[i] / scaling;
    }
}
