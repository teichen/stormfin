#include "LaminarModel.h"
#include <stdlib.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv2.h>

using namespace std;

static int n_states = 4;
static int n_inputs = 3;

LaminarModel::LaminarModel()
{
    /* underactuated tethered underwater drone
       lakes and rivers, assume no turbulence (predictable current or no current)
    */
    mem_test = false;
    initarrays();

    int i,j;

    // linearized rate of change of state
    for (i=0; i<n_states; i++)
    {
        for (j=0; j<n_states; j++)
        {
            linearized_rate[i*n_states + j] = 0;
        }
    }

    for (i=0; i<(int)(n_states/2); i++)
    {
        linearized_rate[2*i*n_states + 2*(i+1)] = 1;
        linearized_rate[(2*i+1)*n_states + 2*i] = -1;
    }

    // linearized jacobian of input estimates
    // TODO: generalize jacobian, using linearized placeholder
    for (i=0; i<n_inputs; i++)
    {
        for (j=0; j<n_states; j++)
        {
            linearized_jacobian[i*n_states + j] = 0;
        }
    }
    for (i=0; i<n_inputs; i++)
    {
        linearized_jacobian[i*n_states + i] = 1.0;
    }

}

int LaminarModel::rate(double t, const double x[], double f[], void *params)
{
    int i,j;

    for (i=0; i<n_states; i++)
    {
        for (j=0; j<n_states; j++)
        {
            f[i*n_states + j] = 0;
        }
    }

    for (i=0; i<(int)(n_states/2); i++)
    {
        f[2*i*n_states + 2*(i+1)] = x[2*(i+1)];
        f[(2*i+1)*n_states + 2*i] = -x[2*i];
    }
    return GSL_SUCCESS;
}

void LaminarModel::map_inputs_states(double* x, double* f)
{
    /* map of states onto inputs
    */
    int i,j;

    for (i=0; i<n_inputs; i++)
    {
        for (j=0; j<n_states; j++)
        {
            f[i*n_states + j] = 0;
        }
    }
    for (i=0; i<n_inputs; i++)
    {
        // TODO: placeholder linear mapping
        f[i*n_states + i] = x[i];
    }
}

void LaminarModel::ode_iv(double* x0, double* x, int n_x, double dt)
{
    /* RK4 integrate an ode rate function, f
    */
    double param = 0.0; // placeholder, rates can take parameters
    size_t size_x = n_x;

    const gsl_odeiv2_step_type * T = gsl_odeiv2_step_rk4;
    gsl_odeiv2_step * s = gsl_odeiv2_step_alloc (T, n_x);

    gsl_odeiv2_system sys = {rate, NULL, size_x, &param}; // TODO: NULL <- jacobian argument

    double h = 0.01; // step size
    double x_err[n_x];

    double t0 = 0.0;
    double t = t0;

    double dxdt_in[n_x], dxdt_out[n_x];

    int i, status;
    while (t < dt)
    {
        status = gsl_odeiv2_step_apply (s, t, h, x, x_err, dxdt_in, dxdt_out, &sys);

        if (status != GSL_SUCCESS) {
            fprintf(stderr, "Error in evolution\n");
            break;
        }

        // update
        for (i=0; i<n_x; i++)
        {
            dxdt_in[i] = dxdt_out[i];
        }
        t += h;
    }

    gsl_odeiv2_step_free (s);
}

void LaminarModel::initarrays()
{
    linearized_rate = (double*) calloc (n_states * n_states, sizeof(double));
    linearized_jacobian = (double*) calloc (n_inputs * n_states, sizeof(double));

    mem_test = true;
}

LaminarModel::~LaminarModel()
{
    if(mem_test==true)
    {
    delete [] linearized_rate;
    delete [] linearized_jacobian;
    }
}

