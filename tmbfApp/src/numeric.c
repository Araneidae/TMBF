/* This file is part of the Libera EPICS Driver,
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

/* High efficiency numeric support routines. */

#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include <limits.h>
#include <stdint.h>

#include "error.h"
#include "numeric.h"

#include "numeric-lookup.h"

#define unlikely(x)   __builtin_expect((x), 0)

#define SINGLE_EXPONENT_BIAS 127
#define DOUBLE_EXPONENT_BIAS 1023



/* This symbol can be set to squeeze the last reluctant bit of precision out
 * of the Reciprocal routine below.  Setting this option costs an extra 30%
 * of execution time, bumping the time on Libera from 137ns to 179ns per
 * call.  As time is more important that one last bit of precision, I've not
 * set this here.
 *    Tests on random data show the following bounds on the error in the
 * divisor:
 *                          Error magnitude     Residual error      Time
 *      LAST_BIT not set    -1.000  +1.077      -0.500  +0.577      137ns
 *      LAST_BIT set        -0.500  +0.615      -0.000  +0.115      179ns
 *
 * The residual error takes into accound the need to round to the very last
 * bit (so +-0,5 of error is unavoidable).  The residual error of 0.115 here
 * corresponds to more than 35 bits of precision before rounding. */
// #define LAST_BIT


/* Computes (2^s/D, s) with 31 or 32 bits of precision (see discussion above
 * about LAST_BIT).  The result is scaled so that it lies in the range
 * 2^31..2^32-1, ensuring the maximum available precision.
 *    The processing cost of this algorithm is one table lookup using a 256
 * byte table, so the cache impact should be small, and four 32x32->64 bit
 * multiplies (or five if LAST_BIT is set).  This routine takes around 140ns
 * (or 180ns if LAST_BIT is set).
 *
 * The algorithm uses two rounds of Newton-Raphson to solve the equation
 *
 *      1/x - D = 0
 *
 * This, rather fortunately, has the Newton-Raphson step
 *
 *      x' = x(2 - xD)
 *
 * which we can do with two multiplies per step.  To see that this works,
 * write x = (1+e)/D, where e represents the error.  Then
 *
 *      x' = (1/D)(1+e)(2 - (1+e)) = (1/D)(1+e)(1-e)
 *
 *                2
 *           1 - e
 *         = ------ ,
 *             D
 *
 * in other words the error is squared at each stage.  We use a small table
 * lookup to give us a worst initial error somewhat better than one part in
 * 2^-8, and so two rounds are enough to reduce the error to less than one bit
 * (if we're careful enough we get 35 bits before rounding at the end). */

unsigned int Reciprocal(unsigned int D, int *shift)
{
    /* Start by normalising D.  This ensures that we have as many bits as
     * possible (and is required for the rest of the algorithm to work). */
    unsigned int norm = CLZ(D);
    D <<= norm;
    *shift += 63 - (int) norm;

    if (unlikely(D == 0x80000000))
    {
        /* Need to handle the case of a one bit quotient specially, as in this
         * one case the clever stuff below just overflows.  This overflow is
         * also evident in this shift fixup. */
        *shift -= 1;
        return D;
    }
    else
    {
        /* Get our first 8 significant bits by table lookup.  We use a nice
         * small table to ensure a small cache footprint (256 bytes). */
        unsigned int A = (D >> 23) & 0xff;
        unsigned int X = 0x80000000 | ((unsigned int) DivideLookup[A] << 23);

        /* We repeat the calculation x' = x * (2 - D * x), working in fixed
         * point with X = 2^63 x; this means we really want to calculate
         *
         *            -63       64
         *      X' = 2    X * (2   - D * X) .
         *
         * We can use a nice scaling trick to avoid representing that
         * troublesome 2^64 value: recall that (2-Dx) = (1-e), with |e| << 1.
         * This means that
         *
         *       64             64
         *      2   - D * X = (2   - 1) & - (D * X) ,
         *
         * which makes for a nice optimisation.  We can then use two rounds of
         * MulUU(A,B) = 2^-32 A * B, leaving us one bit shift short: */
        X = MulUU(X, -MulUU(D, X)) << 1;
#ifndef LAST_BIT
        /* It would be almost good enough to repeat the above step once more:
         *
         *  return MulUU(X, -MulUU(D, X)) << 1;
         *
         * but if so, we lose the very bottom bit, which perturbs the accuracy
         * of the penultimate bit, so we write it out more fully.  Recovering
         * the very last bit is somewhat harder... */
        uint64_t XL = (uint64_t) X * (uint64_t) - MulUU(D, X);
#else
        /* If we're determined, we can squeeze out the last bit of precision
         * (well, we're still shy a residual eighth of a bit, too bad!) using
         * 64 bit arithmetic for the final stages. */
        uint64_t E = - ((uint64_t) D * X);  // Compute (2 - D*x)
        uint64_t XL = (uint64_t) X * (E >> 32) + MulUU(X, (uint32_t) E);
        XL += 0x40000000;   // Bias the final error to middle of last bit.
#endif
        return (uint32_t) (XL >> 31);
    }
}



/* Denormalising, the conversion of a number together with its shift, into a
 * simple integer, is on the face of it as simple as returning X >> shift.
 * However, here we also take overflow into account, which complicates
 * things. */

unsigned int Denormalise(unsigned int X, int shift)
{
    if (shift < 0)
        /* Negative residual shift is a sign of probable trouble: numbers
         * should be arranged so there's some shift left to play with!  Never
         * mind, let's do the best we can... */
        if (CLZ(X) >= (unsigned int) -shift)
            /* Ok, we can afford this much left shift. */
            return X << -shift;
        else
            /* Out of bits.  Return maximum possible value, unless X is zero,
             * in which case we just return 0. */
            if (X == 0)
                return 0;
            else
                return UINT_MAX;
    else if (shift < 32)
        /* The normal case. */
        return X >> shift;
    else
        /* Shifting by more than 32 is not properly defined, but we know to
         * return 0! */
        return 0;
}



/* Computes logarithm base 2 of input to about 22 bits of precision using
 * table lookup and linear interpolation, as for Reciprocal above.
 *
 * Here the input argument is taken to have 16 bits of fraction and 16 bits
 * of integer: this gives us a sensible output dynamic range, with an output
 * in the range +- 16.
 *
 * Computation proceeds as follows:
 *
 *  1. The input is normalised.  The normalising shift will simply be added
 *     into the final result (and is part of the reason for choosing base 2).
 *  2. The normalised input is separated into three fields, 1, A, B, exactly
 *     as for Reciprocal.
 *  3. The logarithm of A is computed by direct lookup.
 *  4. The remaining offset B is corrected for by linear interpolation.  In
 *     this case the scaling factor for B is also looked up.
 *
 * After normalisation write X = 2^31 x and X is decomposed into
 *
 *           31    m
 *      X = 2   + 2  A + B    (A n bits wide, B m bits wide, n+m=31).
 *
 *                -n                -31      m-1
 * Write a = 1 + 2  (A + 0.5), b = 2   (B - 2   ) and then x = a+b and we
 * compute
 *                                   b                      b
 *      log x = log (a+b) = log (a(1+-)) = log a + log (1 + -)
 *         2       2           2     a        2       2     a
 *                         b
 *            ~~ log a + ------
 *                  2    a ln 2
 *
 * The values log_2 a and 1/(a ln 2) are precomputed.  The offsets on A and B
 * used to calculate a and be are used to reduce the maximum value of b to
 * 2^(n+1), thus reducing the residual error. */

int log2i(unsigned int X)
{
    /* First need to check for overflow.  Because linear approximation
     * overestimates the logarithm, we can't go all the way to the maximum
     * possible input without overflow.  Also, we have to return something
     * for log2(0), and we might as well return the smallest value (rather
     * than something close to the largest!) */
    if (X >= 0xFFFFFF80)
        return 0x7FFFFFFF;
    else if (X == 0)
        return (int) 0x80000000;
    else
    {
        unsigned int shift = CLZ(X);
        X <<= shift;
        unsigned int A = (X & 0x7FFFFFFF) >> LOG2_M_BITS;
        int B = (int) (X & LOG2_M_MASK) - LOG2_B_OFFSET;
        struct LOG2_LOOKUP Lookup = Log2Lookup[A];
        return ((15 - (int) shift) << 27) + Lookup.Log + MulSS(Lookup.Scale, B);
    }
}



/* Computes exponential to the power 2 to about 22 bits of precision using
 * algorithms similar to those for Reciprocal and log2 above.
 *
 * Here the input argument has 27 bits of fraction and 5 signed bits of
 * integer, yielding 16 bits of fraction and 16 bits of integer.
 *
 * The computation process is very similar to that for log2, but the input
 * does not need normalisation: instead, the integer part of the input is
 * treated separately (as a shift on the final output).
 *
 * The input X = 2^27 x is decomposed into
 *
 *           27     m
 *      X = 2  S + 2 A + B      (A n bits wide, B m bits wide, n+m=27)
 *
 *                   -n                -27      m-1
 * and we write a = 2  (A + 0.5), b = 2   (B - 2   ) and x = S+a+b.  Then
 *
 *       x    S+a+b    S  a  b     S  a
 *      2  = 2      = 2  2  2  ~~ 2  2  (1 + b ln 2)
 *
 *             a      a
 *         = (2  + b 2  ln 2) << S.
 *
 * The constant 2^a is precomputed.  The multiplier 2^a ln 2 could also be
 * precomputed, but in this implementation is multiplied on the fly.
 *
 * The final required shift is returned instead of being applied to the
 * result: this allows accumulation of shifts if required without loss of
 * precision. */

unsigned int exp2i(int X, int *shift)
{
    *shift += 15 - (X >> 27);
    unsigned int A = (unsigned int) (X & 0x07FFFFFF) >> EXP2_M_BITS;
    int B = (X & EXP2_M_MASK) - EXP2_B_OFFSET;
    unsigned int E = Exp2Lookup[A];
    return E + (unsigned int) MulSU(B << 6, MulUU(E, EXP2_LN2));
}



/* Returns 1e6 * 20 * log_10(X), used for computing dB values for output to
 * the user.
 *
 * Calculate
 *                                 2e7            2e7     -27
 *      to_dB(X) = 2e7 * log  X = ------ log X = ------ (2   log2(X) + 16)
 *                          10    log 10    2    log 10
 *                                   2              2
 *
 * The two constants, 2^32 * 2^-27 * 2e7 / log_2 10, and 16 * 2e7 / log_2 10,
 * are precomputed by support-header.ph. */

int to_dB(unsigned int X)
{
    return TO_DB_OFFSET + MulSS(log2i(X), TO_DB_FACTOR);
}


/* Returns 2^s * 10^(X/(20 * 1e6)), intended as an inverse to to_dB above,
 * where s is a shift normalisation to be applied by the caller.  Calculated
 * as:
 *                                            log 10
 *                      X               X        2
 *                     ---   ( log 10) ---    ------ X
 *                     2e7   (    2  ) 2e7      2e7      KX
 *      from_dB(X) = 10    = (2      )     = 2        = 2
 *
 *                    -s-16      27
 *                 = 2     exp2(2  KX)
 *
 * where
 *          log 10
 *             2
 *      K = ------
 *            2e7
 *
 * We now have to be rather careful about scaling X.  The factor 2^27*K above
 * is about 22.3, which restricts the maximum value of X/1e6 to 93.
 * Furthermore, to avoid losing precision, represent K below as 2^27 * K. */

unsigned int from_dB(int X, int *shift)
{
    /* Check for limits: if computing X<<5 loses bits then we overflow. */
    int XX = X << 5;
    if ((XX >> 5) == X)
    {
        unsigned int result = exp2i(MulUS(FROM_DB_FACTOR, XX), shift);
        *shift += 16;
        return result;
    }
    else
    {
        /* Oops.  Overflow!  Return a limiting value. */
        if (X > 0)
            *shift += 16;
        else
            *shift += 48;
        return 0xFFFFFFFF;
    }
}


/* Computes simultaneous cos and sin by means of a lookup table.  It turns out
 * that using a lookup table is perhaps twice as fast as using CORDIC:
 * unfortunately administering the angle is quite costly.
 *
 * The basic lookup handles angles in the range 0 to 45 degrees, returning
 * precomputed sin and cos values.  A linear fix up then doubles the available
 * precision at the cost of three multiplies using the simple approximation
 *
 *      sin(x + d) ~~ sin(x) + d cos(x)
 *      cos(x + d) ~~ cos(x) - d sin(x)
 *
 * The other octets are selected by the top three bits of the angle (2^32
 * represents a complete rotation), with three bits of administration needed:
 *
 *  octet: in alternating octets we can compute using the formulae
 *
 *      cos(x) = sin(pi/2 - x)
 *      sin(x) = sin(pi/2 - x)
 *
 *  quadrants: in the four quadrants can express the rotations by multiples
 *      of pi/2 as interchanges of sin and cos (as also done for the octets)
 *      together with changes of sign for sin and for cos.
 *
 * It turns out that the fixups (as captured in the code below) can be simply
 * captured by the following triggers on the top three bits, b31, b30, b29:
 *
 *  b30 ^ b29 => Exchange sin & cos
 *  b31       => Negate sin
 *  b30 ^ b31 => Negate cos
 *
 * With SINCOS_N set to 10 this code runs in around 250-300ns. */

void cos_sin(int angle_in, int *c_out, int *s_out)
{
    /* Divide each quadrant into two octants; the angles go in opposite
     * directions on the two octants. */
    int angle = angle_in;
    if (angle_in & 0x20000000)
        angle = ~angle;
    angle &= 0x3FFFFFFF;

    /* Split angle into index and residue. */
    int shift = 32 - COS_SIN_N - 3;
    int index = angle >> shift;
    int residue = angle - (index << shift) - (1 << (shift - 1));

    /* Compute accurate sin/cos in current octant: lookup table followed by
     * linear fixup of the residue. */
    const struct COS_SIN_LOOKUP * lut = &cos_sin_lookup_table[index];
    int c = lut->cos;
    int s = lut->sin;
    /* Finally perform a linear fixup. */
    residue = MulSS(COS_SIN_PI, residue << 4);   // Scale residue by 2 PI
    c -= MulSS(residue, s);
    s += MulSS(residue, c);

    /* Fix up for appropriate octant. */
    if (((angle_in >> 30) ^ (angle_in >> 29)) & 1)
    {
        int t = s;
        s = c; c = t;
    }
    if (angle_in < 0)
        s = - s;
    if (((angle_in >> 31) ^ (angle_in >> 30)) & 1)
        c = - c;

    *c_out = c;
    *s_out = s;
}


/* IEEE 754 binary32 floating point format consists of a 32 bit value structured
 * as three bit fields thus:
 *
 *  31      s   Sign
 *  23-30   e   Exponent (8 bits)
 *  0-22    f   Fraction (23 bits)
 *
 * The represented floating point value (interpreting these fields as unsigned
 * integers) is:
 *              s             -23     e-127
 *      v = (-1)  * (1 + f * 2   ) * 2                  (when 0 < e < 255)
 *
 * except when e=0 or e=255 in which case
 *
 *              s        -23    -126
 *      v = (-1)  * f * 2    * 2                        (when e = 0)
 *
 *              s
 *      v = (-1)  * infinity                            (when e = 255, f = 0)
 *
 *      v = NaN                                         (when e = 255, f > 0)
 *
 * Similarly the representation for binary64 floating point consists of a 64 bit
 * value containing three bit fields thus:
 *
 *  63      s   Sign
 *  52-62   e   Exponent (11 bits)
 *  0-51    f   Fraction (52 bits)
 *
 * The represented floating point value (interpreting these fields as unsigned
 * integers) is:
 *              s             -53     e-1023
 *      v = (-1)  * (1 + f * 2   ) * 2                  (when 0 < e < 2047)
 *
 * except when e=0 or e=255 in which case
 *
 *              s        -53    -1022
 *      v = (-1)  * f * 2    * 2                        (when e = 0)
 *
 *              s
 *      v = (-1)  * infinity                            (when e = 2047, f = 0)
 *
 *      v = NaN                                         (when e = 2047, f > 0)
 *
 * One extra annoying complication is that the little endian base ABI for ARM
 * represents the undering 64 bit pattern with the two 32 bit fields reversed;
 * this is not an issue with the little endian EABI.
 *
 * Here we use bit manipulation only to scale input and convert it into this
 * format. */
void unsigned_fixed_to_single(
    uint32_t input, float *result, uint32_t scaling, int scaling_shift)
{
    uint32_t *iresult = (uint32_t *) result;
    if (input == 0)
        // Special case for 0
        *iresult = 0;
    else
    {
        /* Normalise the input to maintain largest possible dynamic range.
         * After this we have:
         *      2**31 <= fraction < 2**32
         *      input = sign * fraction * 2**-shift_in */
        uint32_t shift_in = CLZ(input);
        uint32_t fraction = input << shift_in;

        /* Rescale the fraction.  The following is optimal, assuming that the
         * scaling factor has been computed to be in the range:
         *      2**31 <= scaling < 2**32
         *
         * If so then the new fraction satisfies
         *      2**30 <= fraction' < 2**32
         *      input = (fraction' / scaling) * 2**(32-shift_in) */
        fraction = MulUU(fraction, scaling);
        /* Alas, one more normalisation step required: may need to fix up
         * fraction by one further bit!  This is as good a point as any to
         * finally discard the top fraction bit (but we'll pretend it's still
         * there in the analysis below). */
        unsigned int fixup = CLZ(fraction) + 1;
        fraction <<= fixup;
        shift_in += fixup;

        /* Finally ready to assemble the final result.  The exponent is
         * computed so that
         *      result = sign * fraction * 2**(exponent - BIAS - 32)
         * As it is, we want
         *      result = input * scaling * 2**scaling_shift
         *             = fraction' * 2**(scaling_shift + 32 - shift_in) ,
         * in other words we want
         *      exponent - BIAS - 32 = scaling_shift + 32 - shift_in */
        int exponent =
            SINGLE_EXPONENT_BIAS + 64 + scaling_shift - (int) shift_in;
        if (exponent <= 0)
        {
            /* Whoops: underflow.  Need to treat underflow to zero
             * separately, as >> is only computed modulo 32. */
            if (exponent < -22)
                /* Complete underflow to zero */
                *iresult = 0;
            else
            {
                /* Restore the missing fraction bit and return a denormalised
                 * result. */
                fraction >>= 1;
                fraction |= 1U << 31;
                *iresult = (fraction >> (9 - exponent));
            }
        }
        else if (exponent >= 255)
            /* Overflow!  Return the appropriate infinity */
            *iresult = 0x7F800000;
        else
            /* 0 < exponent < 255 -- normal case */
            *iresult = ((uint32_t) exponent << 23) | (fraction >> 9);
    }
}

void fixed_to_single(
    int32_t input, float *result, uint32_t scaling, int scaling_shift)
{
    uint32_t uinput = (uint32_t) input;
    uint32_t sign = uinput & 0x80000000;
    if (sign != 0)
        uinput = -uinput;

    unsigned_fixed_to_single(uinput, result, scaling, scaling_shift);
    uint32_t *iresult = (uint32_t *) result;
    *iresult |= sign;
}

/* Computes factors scaling and scaling_shift for use by fixed_to_single() above
 * so that
 *      |target| = scaling * 2**scaling_shift
 * and
 *      2**31 <= scaling < 2**32 .
 * Note that the sign of target is ignored, and we assume that it is a properly
 * normalised number; any errors are completely ignored. */
void compute_scaling(float target, uint32_t *scaling, int *scaling_shift)
{
    uint32_t itarget = REINTERPRET_CAST(uint32_t, target);
    *scaling = 0x80000000 | (itarget << 8);
    /* The exponent scaling is a little delicate.  Put t=target, s=*scaling,
     * h=*scaling_shift, e=raw extracted exponent, itarget[23:30], and
     * f=adjusted fraction, 1:itarget[22:0].  Then we have (from the floating
     * point definition)
     *
     *               -23    e-127        e-150
     *      t = f * 2    * 2      = f * 2
     *
     * We have computed s = f * 2**8, and as we want t = s * 2**h we compute
     * h = e - 158. */
    *scaling_shift = (int) ((itarget >> 23) & 0xFF) - 158;
}
