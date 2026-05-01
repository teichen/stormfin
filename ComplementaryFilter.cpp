#include "ComplementaryFilter.h"
#include <stdlib.h>
#include <cmath>

using namespace std;

ComplementaryFilter::ComplementaryFilter()
{
    mem_test = false;

    n_s = model.n_s;
    n_m = model.n_m;

    initarrays();

    initialize_state();
}

void ComplementaryFilter::process(double dt, double* x, double* s2, double* thrust, double* measurements)
{
    /* fusion of IMU+GPS+ultrasonic data
       propagate the prior estimate
       update the posterior estimate
    */
    // update the priors in case the propagation fails
    utilities.set_elements(x, x_prior, n_s, 1);
    utilities.set_elements(s2, s2_prior, n_s, 2);

    // 1. (a) propagate the mean state estimate, propagate x -> new x_prior
    gsl.ode_iv(model, x, x_prior, n_s, dt, thrust); // TODO: homegrown rk8 routine

    // 1. (b) propagate covariance as updated covariance of previous step with added process noise
    // s2 -> new s2_prior
    double q[n_s * n_s];
    utilities.set_elements(s2, q, n_s, 2);
    // TODO: consider q = 2 * s2 / tau; // approximation for continuous time white process noise
    double g[n_s * n_s]; // stochastic mapping
    double g_T[n_s * n_s];
    utilities.unity(n_s, g);
    utilities.matrix_transpose(g, n_s, n_s, g_T);

    double jac_lin[n_s * n_m]; // linearized jacobian
    model.linearized_jacobian(x, jac_lin); // propagate with the initial state (approx)
    // TODO: consider using full propagation history

    double phi[n_s * n_s]; // matrix exponential for the transition matrix
    double phi_T[n_s * n_s];
    double jac_dt[n_s * n_s];
    int i,j;
    for (i=0; i<n_s; i++)
    {
        for (j=0; j<n_s; j++)
        {
            jac_dt[i*n_s + j] = jac_lin[i*n_s + j] * dt;
        }
    }
    gsl.matrix_exponential(jac_dt, n_s, phi); // TODO: homegrown approx without gsl
    utilities.matrix_transpose(phi, n_s, n_s, phi_T);

    double phi_g[n_s * n_s];
    double phi_g_T[n_s * n_s];
    gsl.matrix_mult(phi, n_s, n_s, g, n_s, n_s, phi_g, n_s, n_s); // TODO: homegrown version in utilities
    utilities.matrix_transpose(phi_g, n_s, n_s, phi_g_T);
    
    double s2_prior_prev[n_s * n_s];
    gsl.matrix_mult(s2, n_s, n_s, phi_T, n_s, n_s, s2_prior_prev, n_s, n_s);
    gsl.matrix_mult(phi, n_s, n_s, s2_prior_prev, n_s, n_s, s2_prior, n_s, n_s);
    double process_noise[n_s * n_s];
    double phi_g_q[n_s * n_s];
    gsl.matrix_mult(phi_g, n_s, n_s, q, n_s, n_s, phi_g_q, n_s, n_s);
    gsl.matrix_mult(phi_g_q, n_s, n_s, phi_g_T, n_s, n_s, process_noise, n_s, n_s);

    for (i=0; i<n_s; i++)
    {
        for (j=0; j<n_s; j++)
        {
            s2_prior[i*n_s + j] += process_noise[i*n_s + j] * dt;
        }
    }

    // overwrite the posteriors in case the update fails
    utilities.set_elements(x_prior, x_post, n_s, 1);
    utilities.set_elements(s2_prior, s2_post, n_s, 2);

    // 2. update the posterior with sensor data
    // x2_prior -> x2_post, s2_prior -> s2_post
    j = 0;
    n_nonnan_z = 0;
    for (i=0; i<n_m; i++)
    {
        if (!std::isnan(measurements[i]))
        {
            nonnan_z_idx[j] = i;
            n_nonnan_z += 1;
        }
    }
    if (n_nonnan_z > 0)
    {
        update(measurements);
    }
    // housekeeping (unnecessary)
    for (i=j+1; i<n_m; i++)
    {
        nonnan_z_idx[i] = -1; // default
    }
}

void ComplementaryFilter::update(double* measurements)
{
    // TODO
}

void ComplementaryFilter::estimate_measurements(double* x, double* zhat)
{
    model.estimate_measurements(x, zhat);
}

void ComplementaryFilter::initialize_state()
{
    model.init_state(x_prior);
    model.init_covariance(s2_prior);

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

void ComplementaryFilter::initarrays()
{
    x_prior   = (double*) calloc (n_s, sizeof(double));
    x_post    = (double*) calloc (n_s, sizeof(double));
    s2_prior = (double*) calloc (n_s * n_s, sizeof(double));
    s2_post  = (double*) calloc (n_s * n_s, sizeof(double));
    nonnan_z_idx = (int*) calloc (n_m, sizeof(int));
    linearized_rate = (double*) calloc (n_s, sizeof(double));
    linearized_jacobian = (double*) calloc (n_s * n_s, sizeof(double));

    mem_test  = true;
}

ComplementaryFilter::~ComplementaryFilter()
{
    if(mem_test==true)
    {
    delete [] x_prior;
    delete [] x_post;
    delete [] s2_prior;
    delete [] s2_post;
    delete [] nonnan_z_idx;
    delete [] linearized_rate;
    delete [] linearized_jacobian;
    }
}
