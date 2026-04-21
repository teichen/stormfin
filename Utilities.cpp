#include "Utilities.h"
#include <stdlib.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>

using namespace std;

Utilities::Utilities()
{
}

void Utilities::matrix_transpose(double* a, int n_a0, int n_a1, double* a_transpose)
{
    int i,j;
    for (i=0; i<n_a0; i++)
    {
        for (j=0; j<n_a1; j++)
        {
            a_transpose[i * n_a1 + j] = a[j * n_a0 + i];
        }
    }
}

void Utilities::matrix_inv(double* a, int n_a0, int n_a1, double* a_inv)
{
    /* matrix inversion, inv(A) = A_inv
    */
    gsl_matrix_view a_matrix = gsl_matrix_view_array(a, n_a0, n_a1);
    gsl_matrix_view a_inv_matrix = gsl_matrix_view_array(a_inv, n_a0, n_a1);

    int s;
    gsl_permutation * p = gsl_permutation_alloc (n_a0);
    gsl_linalg_LU_decomp(&a_matrix.matrix, p, &s);
    gsl_linalg_LU_invert(&a_matrix.matrix, p, &a_inv_matrix.matrix);

    gsl_permutation_free (p);
}

void Utilities::matrix_mult(double* a, int n_a0, int n_a1, double* b, int n_b0, int n_b1, double* c, int n_c0, int n_c1)
{
    /* matrix multiplication, C = AB
    */
    gsl_matrix_view a_matrix = gsl_matrix_view_array(a, n_a0, n_a1);
    gsl_matrix_view b_matrix = gsl_matrix_view_array(b, n_b0, n_b1);
    gsl_matrix_view c_matrix = gsl_matrix_view_array(c, n_a0, n_b1);

    gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, &a_matrix.matrix, &b_matrix.matrix, 0.0, &c_matrix.matrix);
}

Utilities::~Utilities()
{
}
