/* Implemention of equation solving for least squares fitting by Cholesky
 * decomposition.
 *
 * The standard technique for linear equation solving for a matrix equation of
 * the form M x = v is to decompose M into a lower-triangular and an
 * upper-triangular part M = L U, and then solving L U x = v is particularly
 * straightforward.
 *
 * However, when solving a matrix arising from least squares fitting it turns
 * out that a different decomposition, called the Cholesky decomposition, is
 * available to us.  This is available for matrices which are Hermitian (ie,
 * M^H = M) and positive-definite (x^H M x > 0 for all x != 0), and the least
 * squares matrix is of precisely this form.
 *
 * Cholesky decomposition write a Hermitian positive definite matrix M as
 *               H
 *      M = L D L       where D is diagonal, L is lower trianglar with ones on
 *                      the diagonal.
 * Again, solving L D L^H x = v is straighforward. */

#include <stdbool.h>
#include <stdio.h>

#include <complex.h>

#include "tune_support.h"       // For cabs2()
#include "cholesky.h"


/* Choelsky decomposition: given the components of a complex positive definite
 * Hermitian matrix M (passed as separate real diagonal and complex lower
 * triangle components) computes the decomposition M = L D L^H.
 *
 * The components of the NxN matrix M in terms of the parameters d=diag_in and
 * m=mat_in can be written:
 *
 *      [ d[0]    m[0]*   m[1]*   m[3]*   ... ]
 *  M = [ m[0]    d[1]    m[2]*   m[4]*   ... ]
 *      [ m[1]    m[2]    d[2]    m[5]*   ... ]
 *      [ m[3]    m[4]    m[5]    d[3]    ... ]
 *      [ ...     ...     ...     ...     ... ]
 */
void cholesky_decompose(
    unsigned int count, const double diag_in[], const double complex mat_in[],
    double diag_out[], double complex mat_out[])
{
    unsigned int j_base = 0;
    for (unsigned int j = 0; j < count; j ++)
    {
        /* The diagonal term is:
         *                     2
         *  D  = M  - SUM |L  |  D
         *   j    jj  k<j   jk    k
         */
        double d = diag_in[j];
        for (unsigned int k = 0; k < j; k ++)
            d -= cabs2(mat_out[j_base + k]) * diag_out[k];  // |L_jk|^2 * D_k
        diag_out[j] = d;

        /* Now for all rows below D compute
         *         1               __
         *  L   = --- (M  - SUM L  L   D )
         *   ij    D    ij  k<j  ik jk  k
         *          j
         */
        unsigned int i_base = j_base + j;
        for (unsigned int i = j + 1; i < count; i ++)
        {
            double complex l = mat_in[i_base + j];      // M_ij
            for (unsigned int k = 0; k < j; k ++)
                // L_ik L_jk* D_k
                l -= mat_out[i_base + k] * conj(mat_out[j_base + k]) *
                     diag_out[k];
            mat_out[i_base + j] = l / d;
            i_base += i;
        }

        j_base += j;
    }
}


/* Here we invert L D L^H x = V to compute x.  The algebra is simple enough, but
 * the access pattern to the matrices makes things a little tricky. */
void cholesky_substitution(
    unsigned int count, const double diag_in[], const double complex mat_in[],
    const double complex vec_in[], double complex vec_out[])
{
    /* We can use the out vector as our workspace because of the access
     * sequence. */
    double complex *work = vec_out;

    /* First invert L x = V. */
    unsigned int i_base = 0;
    for (unsigned int i = 0; i < count; i ++)
    {
        complex double x = vec_in[i];
        for (unsigned int k = 0; k < i; k ++)
            x -= work[k] * mat_in[i_base + k];
        work[i] = x;
        i_base += i;
    }

    /* Divide by the diagonal */
    for (unsigned int i = 0; i < count; i ++)
        work[i] /= diag_in[i];

    /* Finally invert L^H x = V.  Here we have to work backwards, which is a bit
     * more tricky.  Fortunately i_base is already nearly in the right place. */
    for (unsigned int i = count; i > 0; )
    {
        i -= 1;

        complex double x = work[i];
        unsigned int ji = i_base + i;
        for (unsigned int j = i + 1; j < count; j ++)
        {
            x -= vec_out[j] * conj(mat_in[ji]);
            ji += j;
        }
        vec_out[i] = x;
        i_base -= i;
    }
}


void cholesky_solve(
    unsigned int count, const double diag_in[], const double complex mat_in[],
    const double complex vec_in[], double complex vec_out[])
{
    double diag_ch[count];
    double complex mat_ch[TRIANGLE_SIZE(count)];
    cholesky_decompose(count, diag_in, mat_in, diag_ch, mat_ch);
    cholesky_substitution(count, diag_ch, mat_ch, vec_in, vec_out);
}
