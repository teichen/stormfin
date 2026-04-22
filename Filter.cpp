#include "Filter.h"
#include <stdlib.h>

using namespace std;

Filter::Filter()
{
    mem_test = false;

    n_s = model.n_s;
    n_m = model.n_m;

    initarrays();

    initialize_state();
}

void Filter::process(double dt, double* x_post, double* s2_post, double* thrust, double* measurements)
{
    /* fusion of IMU+GPS+ultrasonic data
       propagate the prior estimate
       update the posterior estimate

       accelerometer bias, beta, propagates into velocity and position error
       50 <= beta <= 1000 microg
       ~2500 (microg) ** 2 / Hz white noise

       gyro bias or drift rate, epsilon, integrates into attitude error
       0.01 <= epsilon <= 10 deg / hr
       ~1.e-6 (deg / s) ** 2 / Hz white noise
    */
    set_prior(x_post, s2_post);
    utilities.ode_iv(model, x_post, s2_post, n_s, dt);

    set_posterior(x_prior, s2_prior);
    update(x_post, measurements);
}

void Filter::update(double* x, double* measurements)
{
    double gain[n_m * n_s];
    double gain_T[n_m * n_s];
    double noise[n_m * n_m];
    double residuals[n_m];
    double zhat[n_m];
    double jacobian[n_s * n_m];
    double jacobian_T[n_m * n_s];

    int i,j;

    for (i=0; i<n_m; i++)
    {
        noise[i * n_m + i] = 0.001;
    }

    utilities.matrix_transpose(jacobian, n_s, n_m, jacobian_T);

    estimate_measurements(x, zhat);

    double jac_sig[n_m * n_s];
    double jac_sig_T[n_m * n_s];
    double s2_meas[n_m * n_m];

    utilities.matrix_mult(jacobian, n_m, n_s, s2_prior, n_s, n_s, jac_sig, n_m, n_s);
    utilities.matrix_transpose(jac_sig, n_m, n_s, jac_sig_T);

    utilities.matrix_mult(jac_sig, n_m, n_s, jacobian_T, n_s, n_m, s2_meas, n_m, n_m);

    double meas_noise[n_m * n_m];

    for (i=0; i<n_m; i++)
    {
        for (j=0; j<n_m; j++)
        {
            meas_noise[i * n_m + j] = s2_meas[i * n_m + j] + noise[i * n_m + j];
        }
    }

    double meas_noise_inv[n_m * n_m];

    utilities.matrix_inv(meas_noise, n_m, n_m, meas_noise_inv);
    utilities.matrix_mult(meas_noise_inv, n_m, n_m, jac_sig, n_m, n_s, gain, n_m, n_s);
    utilities.matrix_transpose(gain, n_m, n_s, gain_T);

    for (i=0; i<n_m; i++)
    {
        residuals[i] = measurements[i] - zhat[i];
    }

    double dx[n_s];

    utilities.matrix_mult(gain_T, n_s, n_m, residuals, n_m, 1, dx, n_s, 1);

    for (i=0; i<n_s; i++)
    {
        x_post[i] = x_prior[i] + dx[i];
    }
}

void Filter::estimate_measurements(double* x, double* zhat)
{
    model.estimate_measurements(x, zhat);
}

void Filter::set_prior(double* x, double* s2)
{
    int i,j;
    for (i=0; i<n_s; i++)
    {
        x_prior[i] = x[i];
        for (j=0; j<n_s; j++)
        {
            s2_prior[i*n_s + j] = s2[i*n_s + j];
        }
    }
}

void Filter::set_posterior(double* x, double* s2)
{
    int i,j;
    for (i=0; i<n_s; i++)
    {
        x_post[i] = x[i];
        for (j=0; j<n_s; j++)
        {
            s2_post[i*n_s + j] = s2[i*n_s + j];
        }
    }
}

void Filter::initialize_state()
{
    int i, j;
    
    for (i=0; i<n_s; i++)
    {
        x_prior[i] = 0.0;
        x_post[i]  = 0.0;
        for (j=0; j<n_s; j++)
        {
            if (i == j)
            {
                s2_prior[i*n_s + j] = 0.001;
                s2_post[i*n_s + j]  = 0.001;
            }
            else
            {
                s2_prior[i*n_s + j] = 0.0;
                s2_post[i*n_s + j]  = 0.0;
            }
        }
    }
}

void Filter::initarrays()
{
    x_prior   = (double*) calloc (n_s, sizeof(double));
    x_post    = (double*) calloc (n_s, sizeof(double));
    s2_prior = (double*) calloc (n_s * n_s, sizeof(double));
    s2_post  = (double*) calloc (n_s * n_s, sizeof(double));

    linearized_rate = (double*) calloc (n_s, sizeof(double));
    linearized_jacobian = (double*) calloc (n_s * n_s, sizeof(double));

    mem_test  = true;
}

Filter::~Filter()
{
    if(mem_test==true)
    {
    delete [] x_prior;
    delete [] x_post;
    delete [] s2_prior;
    delete [] s2_post;
    delete [] linearized_rate;
    delete [] linearized_jacobian;
    }
}
