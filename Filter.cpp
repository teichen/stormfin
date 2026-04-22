#include "Filter.h"
#include <stdlib.h>

using namespace std;

Filter::Filter()
{
    mem_test = false;
}

void Filter::init_model(Model& model)
{
    n_s = model.n_s;
    n_m = model.n_m;

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
    double gain[n_m * n_s];
    double gain_T[n_m * n_s];
    double noise[n_m * n_m];
    double residuals[n_m];
    double estimates[n_m];
    double jacobian[n_s * n_m];
    double jacobian_T[n_m * n_s];

    int i,j;

    for (i=0; i<n_m; i++)
    {
        noise[i * n_m + i] = 0.001;
    }

    utilities.matrix_transpose(jacobian, n_s, n_m, jacobian_T);

    calc_estimates(x, estimates);

    double jac_sig[n_m * n_s];
    double jac_sig_T[n_m * n_s];
    double sig_inputs[n_m * n_m];

    utilities.matrix_mult(jacobian, n_m, n_s, sig_prior, n_s, n_s, jac_sig, n_m, n_s);

    utilities.matrix_transpose(jac_sig, n_m, n_s, jac_sig_T);

    utilities.matrix_mult(jac_sig, n_m, n_s, jacobian_T, n_s, n_m, sig_inputs, n_m, n_m);

    double inputs_noise[n_m * n_m];

    for (i=0; i<n_m; i++)
    {
        for (j=0; j<n_m; j++)
        {
            inputs_noise[i * n_m + j] = sig_inputs[i * n_m + j] + noise[i * n_m + j];
        }
    }

    double inputs_noise_inv[n_m * n_m];

    utilities.matrix_inv(inputs_noise, n_m, n_m, inputs_noise_inv);

    utilities.matrix_mult(inputs_noise_inv, n_m, n_m, jac_sig, n_m, n_s, gain, n_m, n_s);

    utilities.matrix_transpose(gain, n_m, n_s, gain_T);

    for (i=0; i<n_m; i++)
    {
        residuals[i] = inputs[i] - estimates[i];
    }

    double dx[n_s];

    utilities.matrix_mult(gain_T, n_s, n_m, residuals, n_m, 1, dx, n_s, 1);

    for (i=0; i<n_s; i++)
    {
        x_post[i] = x_prior[i] + dx[i];
    }
}

void Filter::calc_estimates(double* x, double* estimates)
{
    // linearized jacobian to back out linearized estimates
    
    int i,j;
    for (i=0; i<n_m; i++)
    {
        for (j=0; j<n_s; j++)
        {
            estimates[i] = 0.0; // TODO: estimates[i] = jacobian[i*n + j] * x[j];
        }
    }
}

void Filter::set_prior(double* x, double* sigma)
{
    int i,j;
    for (i=0; i<n_s; i++)
    {
        x_prior[i] = x[i];
        for (j=0; j<n_s; j++)
        {
            sig_prior[i*n_s + j] = sigma[i*n_s + j];
        }
    }
}

void Filter::set_posterior(double* x, double* sigma)
{
    int i,j;
    for (i=0; i<n_s; i++)
    {
        x_post[i] = x[i];
        for (j=0; j<n_s; j++)
        {
            sig_post[i*n_s + j] = sigma[i*n_s + j];
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
                sig_prior[i*n_s + j] = 0.001;
                sig_post[i*n_s + j]  = 0.001;
            }
            else
            {
                sig_prior[i*n_s + j] = 0.0;
                sig_post[i*n_s + j]  = 0.0;
            }
        }
    }
}

void Filter::initarrays()
{
    x_prior   = (double*) calloc (n_s, sizeof(double));
    x_post    = (double*) calloc (n_s, sizeof(double));
    sig_prior = (double*) calloc (n_s * n_s, sizeof(double));
    sig_post  = (double*) calloc (n_s * n_s, sizeof(double));

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
    delete [] sig_prior;
    delete [] sig_post;
    delete [] linearized_rate;
    delete [] linearized_jacobian;
    }
}
