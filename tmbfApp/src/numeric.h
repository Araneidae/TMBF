/* This file is part of the Libera TMBF Driver,
 * Copyright (C) 2005-2009  Michael Abbott, Diamond Light Source Ltd.
 *
 * The Libera EPICS Driver is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * The Libera EPICS Driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 *
 * Contact:
 *      Dr. Michael Abbott,
 *      Diamond Light Source Ltd,
 *      Diamond House,
 *      Chilton,
 *      Didcot,
 *      Oxfordshire,
 *      OX11 0DE
 *      michael.abbott@diamond.ac.uk
 */

/* High efficiency numeric support routines:
 *
 *      CLZ             Counts leading zeros.
 *      MulUU           Scaled multiplication routines
 *      MulSS                   "
 *      MulUS                   "
 *      Reciprocal      Computes 1/x
 *      nmTOmm          Multiplies by 1e-6 and converts to float.
 */


/* Returns the number of leading zeros in an integer.  On the v5 ARM this
 * generates a single clz instruction. */
#define CLZ(x)  ((unsigned int) __builtin_clz(x))

/* Implementation of CLZ for 64 bit integers. */
static inline unsigned int clz_64(uint64_t x)
{
    if (x >> 32)
        return CLZ(x >> 32);
    else
        return 32 + CLZ((uint32_t) (x & 0xFFFFFFFF));
}




/* Returns 2^-32 * x * y, signed or unsigned.  This is particularly convenient
 * for fixed point arithmetic, and is remarkably inexpensive (approximately
 * 30ns).
 *    We do these using inline assembler because the compiler is too
 * dim-witted to do this right otherwise (gcc 3.4.5 does the whole 64 bit
 * right shift).  Note the "=&r" modifiers: the true constraint on -mull is
 * that the first three registers be distinct, but the only practical way to
 * achieve this seems to be to mark the two (already separate) outputs as
 * "early clobber" to force them to be distinct from both inputs. */
static inline unsigned int MulUU(unsigned int x, unsigned int y)
{
    unsigned int result, temp;
    __asm__("umull   %1, %0, %2, %3" :
        "=&r"(result), "=&r"(temp) : "r"(x), "r"(y));
    return result;
}

static inline int MulSS(int x, int y)
{
    unsigned int result, temp;
    __asm__("smull   %1, %0, %2, %3" :
        "=&r"(result), "=&r"(temp) : "r"(x), "r"(y));
    return result;
}


/* To retain the maximum possible number of bits we have to take a bit of
 * care when multiplying a signed by an unsigned integer.  This routine works
 * by writing the signed part as
 *      y = y0 - s*2^31
 * where s is the sign bit and y0 the bottom 31 bits of y.
 *
 * As an unsigned number, this same y is treated as
 *      y_ = y0 + s*2^31 ,
 * so when we calculcate x*y_ (for unsigned x) we get
 *      x*y_ = x*y0 + s*x*2^31 = x*y + s*x*2^32
 * and so we can use unsigned multiplication to compute
 *      x*y = x*y_ - s*x*2^32 .
 *
 * If it is known that x < 2^31 (and so cannot be mistaken for a signed
 * value) then it will be faster to use MulSS instead. */
static inline int MulUS(unsigned int x, int y)
{
    int result = (int) MulUU(x, y);
    if (y < 0)
        result -= x;
    return result;
}


/* The following routines are a kind of "poor man's floating point": we
 * perform long multiplication without loss of precision, while maintaining
 * the residual offset as a separately returned result. */

/* Returns 2^s * x * y and s, where s+32 is the maximum shift that can be
 * applied to both arguments to ensure that as few significant bits as
 * possible are lost.  The value for s lies in the range +32 to -32.
 *     The shift is accumulated to assist with extended expressions. */
static inline unsigned int MulUUshift(
    unsigned int x, unsigned int y, int *shift)
{
    int sx = CLZ(x);
    int sy = CLZ(y);
    *shift += sx + sy - 32;
    return MulUU(x << sx, y << sy);
}

/* The same game with signed arguments is somewhat more difficult, so we pass
 * for now. */
// inline int MulSSshift(int x, int y, int *shift)


/* Takes into account an existing shift and applies it to the argument,
 * taking care to allow for overflow.   Returns 2^-s * X, but takes overflow
 * into account. */
unsigned int Denormalise(unsigned int x, int shift);


/* Returns (Y, s) such that:
 *
 *               s
 *  1.  X * Y = 2    (to approximately 31 bits of precision)
 *
 *       31         32
 *  2.  2   <= Y < 2   .
 *
 * From this the following further facts follow:
 *
 *           s
 *      Y = 2  / X
 *
 *       s-32         s-31
 *      2     < X <= 2
 *
 *      32 <= s <= 63.
 *
 * If X is zero then Y=0, s=31 is returned: this is an accident of
 * implementation of little significance, it's up to the caller not to pass
 * X=0.
 *
 * The value assigned to shift is accumulated by s to assist with extended
 * expressions. */
unsigned int Reciprocal(unsigned int X, int *shift);



/* Fast fixed point computation of logarithm base 2 with about 20 bits of
 * precision using pure integer arithmetic.  The input has 16 bits after the
 * binary point and the output 27 bits, so we can say:
 *
 *                          16               27
 *      Takes argument X = 2  x and returns 2  log x.
 *                                                2
 *
 * Alternatively, we can write the relationship between this function (log2)
 * and the true base 2 logarithm (log_2) as
 *
 *                 27      -16      27
 *      log2(X) = 2  log (2   X) = 2  (log X - 16)
 *                      2                 2
 * or
 *               -27
 *      log X = 2   log2(X) + 16.
 *         2
 *
 * Output is clipped in response to extreme inputs. */
int log2i(unsigned int X);

/* Fast fixed point computation of exponential 2^X with about 20 bits of
 * precision using pure integer arithmetic.  The input has 27 bits after the
 * binary point and the output 16 bits:
 *
 *                          27               s 16 x   s+16+x
 *      Takes argument X = 2  x and returns 2 2 2  = 2      , where s is
 *      returned as the argument shift.  Thus:
 *
 *                       -27         -27
 *                 s 16 2   X   s + 2   X + 16
 *      exp2(X) = 2 2  2     = 2
 *
 * or
 *       X    -s      27           -s-16      27
 *      2  = 2  exp2(2  (X-16)) = 2     exp2(2  X)
 *
 * This is the inverse of the log2 function, and the format of arguments
 * matches precisely, except for the shift which is separated out here. */
unsigned int exp2i(int X, int *shift);

/* Returns 1e6 * 20 * log_10(X): computes the dB value corresponding to X. */
int to_dB(unsigned int X);

/* Converts a dB value into the corresponding exponential value.
 * Returns 2^s * 10^(X/(20 * 1e6)) together with s as shift.
 *    The range of possible input values is approximately +-67e6.  Note that
 * there is an offset of 16 on the shift, so the range of possible shift
 * values is 16 to 48. */
unsigned int from_dB(int X, int *shift);


/* Computes simultaneous cos and sin of the given angle.  The angle is a
 * 32-bit fraction of a complete rotation, so to compute angle_in from an
 * angle in radians compute
 *
 *                  32   angle
 *      angle_in = 2   * -----
 *                       2 pi
 *
 * The computed values are scaled by 2^30, so that the signed result fits in
 * 32 bits without overflow. */
void cos_sin(int angle_in, int *c_out, int *s_out);


/* Converts fixed point integer to floating point applying the conversion
 *  *result = input * scaling * 2**scaling_shift
 * For optimal results scaling and scaling_shift should be precomputed so that
 *      2**31 <= scaling < 2**32 . */
void fixed_to_single(
    int32_t input, float *result, uint32_t scaling, int scaling_shift);
/* Computes scaling factors for fixed_to_single above so that
 *      target = scaling * 2**scaling_shift .
 *
 * This ensures that computing
 *      compute_scaling(target, &scaling, &scaling_shift);
 *      fixed_to_single(input, &result, scaling, scaling_shift);
 * sets
 *      result = input * target; */
void compute_scaling(float target, uint32_t *scaling, int *scaling_shift);
