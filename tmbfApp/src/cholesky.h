/* Definitions for inverting matrices derived from solving equations. */

/* Number of points required to represent the off-diagonal elements of a
 * symmetric or Hermitian matrix. */
#define TRIANGLE_SIZE(N)    ((N) * ((N) - 1)/2)

/* Given a Hermitian and strictly positive matrix represented by its diagonal
 * elements and lower triangle, computes its Cholesky decomposition. */
void cholesky_decompose(
    unsigned int count, const double diag_in[], const double complex mat_in[],
    double diag_out[], double complex mat_out[]);

/* Given a Cholesky decomposition (L,D) solves the equation L D L^H x = v for
 * unknown x. */
void cholesky_substitution(
    unsigned int count, const double diag_in[], const double complex mat_in[],
    const double complex vec_in[], double complex vec_out[]);

/* Solves problem M x = v. */
void cholesky_solve(
    unsigned int count, const double diag_in[], const double complex mat_in[],
    const double complex vec_in[], double complex vec_out[]);
