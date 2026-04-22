#include "Filter.h"
#include <stdlib.h>

using namespace std;

Filter::Filter()
{
    mem_test = false;
}

void Filter::init_model(Model& model)
{
    n         = 9; // placeholder
    n_in      = 15; // placeholder

    initarrays();

    initialize_state();

    t0 = 0.0;
}

void Filter::propagate_update(double t, double* input_data)
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
    double dt;
    dt = t - t0;

    set_prior(x_post, sig_post);
    // propagator.propagate(t0, t, dt, x_prior); // TODO

    set_posterior(x_prior, sig_prior);
    update(x_post, input_data);

    t0 = t;
}

void Filter::update(double* x, double* inputs)
{
    double gain[n_in * n];
    double gain_T[n_in * n];
    double noise[n_in * n_in];
    double residuals[n_in];
    double estimates[n_in];
    double jacobian[n * n_in];
    double jacobian_T[n_in * n];

    int i,j;

    for (i=0; i<n_in; i++)
    {
        noise[i * n_in + i] = 0.001;
    }

    utilities.matrix_transpose(jacobian, n, n_in, jacobian_T);

    calc_estimates(x, estimates);

    double jac_sig[n_in * n];
    double jac_sig_T[n_in * n];
    double sig_inputs[n_in * n_in];

    utilities.matrix_mult(jacobian, n_in, n, sig_prior, n, n, jac_sig, n_in, n);

    utilities.matrix_transpose(jac_sig, n_in, n, jac_sig_T);

    utilities.matrix_mult(jac_sig, n_in, n, jacobian_T, n, n_in, sig_inputs, n_in, n_in);

    double inputs_noise[n_in * n_in];

    for (i=0; i<n_in; i++)
    {
        for (j=0; j<n_in; j++)
        {
            inputs_noise[i * n_in + j] = sig_inputs[i * n_in + j] + noise[i * n_in + j];
        }
    }

    double inputs_noise_inv[n_in * n_in];

    utilities.matrix_inv(inputs_noise, n_in, n_in, inputs_noise_inv);

    utilities.matrix_mult(inputs_noise_inv, n_in, n_in, jac_sig, n_in, n, gain, n_in, n);

    utilities.matrix_transpose(gain, n_in, n, gain_T);

    for (i=0; i<n_in; i++)
    {
        residuals[i] = inputs[i] - estimates[i];
    }

    double dx[n];

    utilities.matrix_mult(gain_T, n, n_in, residuals, n_in, 1, dx, n, 1);

    for (i=0; i<n; i++)
    {
        x_post[i] = x_prior[i] + dx[i];
    }
}

void Filter::calc_estimates(double* x, double* estimates)
{
    // linearized jacobian to back out linearized estimates
    
    int i,j;
    for (i=0; i<n_in; i++)
    {
        for (j=0; j<n; j++)
        {
            estimates[i] = 0.0; // TODO: estimates[i] = jacobian[i*n + j] * x[j];
        }
    }
}

void Filter::set_prior(double* x, double* sigma)
{
    int i,j;
    for (i=0; i<n; i++)
    {
        x_prior[i] = x[i];
        for (j=0; j<n; j++)
        {
            sig_prior[i*n + j] = sigma[i*n + j];
        }
    }
}

void Filter::set_posterior(double* x, double* sigma)
{
    int i,j;
    for (i=0; i<n; i++)
    {
        x_post[i] = x[i];
        for (j=0; j<n; j++)
        {
            sig_post[i*n + j] = sigma[i*n + j];
        }
    }
}

void Filter::initialize_state()
{
    int i, j;
    
    for (i=0; i<n; i++)
    {
        x_prior[i] = 0.0;
        x_post[i]  = 0.0;
        for (j=0; j<n; j++)
        {
            if (i == j)
            {
                sig_prior[i*n + j] = 0.001;
                sig_post[i*n + j]  = 0.001;
            }
            else
            {
                sig_prior[i*n + j] = 0.0;
                sig_post[i*n + j]  = 0.0;
            }
        }
    }
}

void Filter::initarrays()
{
    x_prior   = (double*) calloc (n, sizeof(double));
    x_post    = (double*) calloc (n, sizeof(double));
    sig_prior = (double*) calloc (n * n, sizeof(double));
    sig_post  = (double*) calloc (n * n, sizeof(double));

    mem_test  = true;
}

Filter::~Filter()
{
    if(mem_test==true)
    {
    delete [] x_prior;
    delete [] x_post;
    delete [] sig_prior;
    delete [] sig_post;
    }
}
