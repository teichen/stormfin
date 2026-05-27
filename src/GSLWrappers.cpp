#include "GSLWrappers.h"
#include <stdlib.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv2.h>
#include <gsl/gsl_mode.h>

using namespace std;

GSLWrappers::GSLWrappers()
{
}

void GSLWrappers::matrix_inv(double* a, int n_a0, int n_a1, double* a_inv)
{
    /* matrix inversion, inv(A) = A_inv
    */
    gsl_matrix_view a_matrix = gsl_matrix_view_array(a, n_a0, n_a1);
    gsl_matrix_view a_inv_matrix = gsl_matrix_view_array(a_inv, n_a0, n_a1);

    int s;
    gsl_permutation * p = gsl_permutation_alloc (n_a0);
    gsl_matrix *LU = gsl_matrix_alloc(n_a0, n_a1);
    gsl_matrix_memcpy(LU, &a_matrix.matrix);
    gsl_linalg_LU_decomp(LU, p, &s);
    gsl_linalg_LU_invert(LU, p, &a_inv_matrix.matrix);

    gsl_permutation_free (p);
}

void GSLWrappers::matrix_exponential(double* a, int n_a, double* a_exp)
{
    /* A_exp = expm(A)
    */
    gsl_matrix_view a_matrix = gsl_matrix_view_array(a, n_a, n_a);
    gsl_matrix_view a_exp_matrix = gsl_matrix_view_array(a_exp, n_a, n_a);

    gsl_linalg_exponential_ss(&a_matrix.matrix, &a_exp_matrix.matrix, GSL_PREC_DOUBLE);
}

void GSLWrappers::matrix_mult(double* a, int n_a0, int n_a1, double* b, int n_b0, int n_b1, double* c, int n_c0, int n_c1)
{
    /* matrix multiplication, C = AB
    */
    gsl_matrix_view a_matrix = gsl_matrix_view_array(a, n_a0, n_a1);
    gsl_matrix_view b_matrix = gsl_matrix_view_array(b, n_b0, n_b1);
    gsl_matrix_view c_matrix = gsl_matrix_view_array(c, n_a0, n_b1);

    gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, &a_matrix.matrix, &b_matrix.matrix, 0.0, &c_matrix.matrix);
}

void GSLWrappers::ode_iv(LaminarModel& model, double* x0, double* x, int n_x, double dt, double* u)
{
    /* integrate an ode rate function, f
    */
    size_t size_x = n_x;

    // RK8 with adaptive step size
    gsl_odeiv2_system sys = {model.rate, model.jacobian, size_x, u};
    gsl_odeiv2_control *c = gsl_odeiv2_control_y_new(1e-10, 1e-10);
    gsl_odeiv2_evolve *e = gsl_odeiv2_evolve_alloc(n_x);
    gsl_odeiv2_step *s = gsl_odeiv2_step_alloc(gsl_odeiv2_step_rk8pd, n_x);

    double h = 0.01; // step size
    double x_err[n_x];

    double t0 = 0.0;
    double t = t0;

    int i, status;

    while (t < dt)
    {
        // RK8 with adaptive step size
        status = gsl_odeiv2_evolve_apply (e, c, s, &sys, &t, dt, &h, x);

        if (status != GSL_SUCCESS) {
            fprintf(stderr, "Error in evolution\n");
            break;
        }
    }

    gsl_odeiv2_evolve_free(e);
    gsl_odeiv2_control_free(c);
    gsl_odeiv2_step_free (s);
}

GSLWrappers::~GSLWrappers()
{
}
