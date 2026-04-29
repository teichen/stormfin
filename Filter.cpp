#include "Filter.h"
#include <stdlib.h>
#include <cmath>

using namespace std;

Filter::Filter()
{
    mem_test = false;

    n_s = model.n_s;
    n_m = model.n_m;

    initarrays();

    initialize_state();
}

void Filter::process(double dt, double* x, double* s2, double* thrust, double* measurements)
{
    /* fusion of IMU+GPS+ultrasonic data
       propagate the prior estimate
       update the posterior estimate
    */
    // update the priors in case the propagation fails
    utilities.set_elements(x, x_prior, n_s, 1);
    utilities.set_elements(s2, s2_prior, n_s, 2);

    // 1. (a) propagate the mean state estimate, propagate x -> new x_prior
    utilities.ode_iv(model, x, x_prior, n_s, dt, thrust);

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
    utilities.matrix_exponential(jac_dt, n_s, phi); 
    utilities.matrix_transpose(phi, n_s, n_s, phi_T);

    double phi_g[n_s * n_s];
    double phi_g_T[n_s * n_s];
    utilities.matrix_mult(phi, n_s, n_s, g, n_s, n_s, phi_g, n_s, n_s);
    utilities.matrix_transpose(phi_g, n_s, n_s, phi_g_T);
    
    double s2_prior_prev[n_s * n_s];
    utilities.matrix_mult(s2, n_s, n_s, phi_T, n_s, n_s, s2_prior_prev, n_s, n_s);
    utilities.matrix_mult(phi, n_s, n_s, s2_prior_prev, n_s, n_s, s2_prior, n_s, n_s);
    double process_noise[n_s * n_s];
    double phi_g_q[n_s * n_s];
    utilities.matrix_mult(phi_g, n_s, n_s, q, n_s, n_s, phi_g_q, n_s, n_s);
    utilities.matrix_mult(phi_g_q, n_s, n_s, phi_g_T, n_s, n_s, process_noise, n_s, n_s);

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

void Filter::update(double* measurements)
{
    double gain[n_m * n_s];
    double gain_T[n_m * n_s];
    double meas_noise[n_m * n_m];
    double residuals[n_m];
    double zhat[n_m];
    double jac_meas[n_m * n_s]; // linearized jacobian
    model.linearized_measurement_jacobian(x_prior, jac_meas);
    double jac_meas_T[n_m * n_s];

    int i,j;

    model.init_meas_noise(meas_noise);

    utilities.matrix_transpose(jac_meas, n_m, n_s, jac_meas_T);

    estimate_measurements(x_prior, zhat);

    double jac_meas_sig[n_m * n_s];
    double jac_meas_sig_T[n_m * n_s];
    double s2_meas[n_m * n_m];

    // project state covariance with measurement noise to calculate measurement covariance
    utilities.matrix_mult(jac_meas, n_m, n_s, s2_prior, n_s, n_s, jac_meas_sig, n_m, n_s);
    utilities.matrix_transpose(jac_meas_sig, n_m, n_s, jac_meas_sig_T);
    utilities.matrix_mult(jac_meas_sig, n_m, n_s, jac_meas_T, n_s, n_m, s2_meas, n_m, n_m);

    double meas_noise_tot[n_m * n_m];

    for (i=0; i<n_m; i++)
    {
        for (j=0; j<n_m; j++)
        {
            meas_noise_tot[i * n_m + j] = s2_meas[i * n_m + j] + meas_noise[i * n_m + j];
        }
    }

    double meas_noise_tot_inv[n_m * n_m];

    // calculate gain
    utilities.matrix_inv(meas_noise_tot, n_m, n_m, meas_noise_tot_inv);

    double s2_prior_jac_meas_T[n_s * n_m];
    utilities.matrix_mult(s2_prior, n_s, n_s, jac_meas_T, n_s, n_m, s2_prior_jac_meas_T, n_s, n_m);
    
    // inner product should involve only measurement space for data we have at this time
    // truncate gain, noise, residuals, zhat, jac_meas into space of nonnan measurements
    utilities.matrix_mult(s2_prior_jac_meas_T, n_s, n_m, meas_noise_tot_inv, n_m, n_m, gain, n_s, n_m);
    utilities.matrix_transpose(gain, n_s, n_m, gain_T);

    double s2_prior_jac_meas_T_trunc[n_s * n_nonnan_z];
    double meas_noise_tot_inv_trunc[n_nonnan_z * n_nonnan_z];
    double gain_trunc[n_s * n_nonnan_z];
    double gain_T_trunc[n_nonnan_z * n_s];
    utilities.get_cols(s2_prior_jac_meas_T, n_s, n_m, nonnan_z_idx, n_nonnan_z, s2_prior_jac_meas_T_trunc);
    utilities.get_rows_cols(meas_noise_tot_inv, n_m, n_m, nonnan_z_idx, n_nonnan_z, meas_noise_tot_inv_trunc);
    utilities.matrix_mult(s2_prior_jac_meas_T_trunc, n_s, n_nonnan_z, meas_noise_tot_inv_trunc, n_nonnan_z, n_nonnan_z, gain_trunc, n_s, n_nonnan_z);
    utilities.matrix_transpose(gain_trunc, n_s, n_nonnan_z, gain_T_trunc);

    double residuals_trunc[n_nonnan_z];
    j = 0;
    for (i=0; i<n_m; i++)
    {
        residuals[i] = measurements[i] - zhat[i];
        if (!std::isnan(measurements[i]))
        {
            residuals_trunc[j] = residuals[i];
            j++;
        }
    }

    double dx[n_s];

    // FULL: utilities.matrix_mult(gain, n_s, n_m, residuals, n_m, 1, dx, n_s, 1);
    utilities.matrix_mult(gain_trunc, n_s, n_nonnan_z, residuals_trunc, n_nonnan_z, 1, dx, n_s, 1);

    for (i=0; i<n_s; i++)
    {
        x_post[i] = x_prior[i] + dx[i];
    }

    // update covariance
    double eye[n_s * n_s];
    utilities.unity(n_s, eye);
    double gain_jac_meas[n_s * n_s];

    // FULL: utilities.matrix_mult(gain, n_s, n_m, jac_meas, n_m, n_s, gain_jac_meas, n_s, n_s);
    double jac_meas_trunc[n_nonnan_z * n_s];
    utilities.get_rows(jac_meas, n_m, n_s, nonnan_z_idx, n_nonnan_z, jac_meas_trunc);
    utilities.matrix_mult(gain_trunc, n_s, n_nonnan_z, jac_meas_trunc, n_nonnan_z, n_s, gain_jac_meas, n_s, n_s);

    double eye_gain_jac_meas[n_s * n_s];
    for (i=0; i<n_s; i++)
    {
        for (j=0; j<n_s; j++)
        {
            eye_gain_jac_meas[i*n_s + j] = eye[i*n_s + j] - gain_jac_meas[i*n_s + j];
        }
    }
    double s2_post_prev[n_s * n_s];
    double s2_tmp[n_s * n_s];
    utilities.matrix_mult(eye_gain_jac_meas, n_s, n_s, s2_prior, n_s, n_s, s2_tmp, n_s, n_s);
    double eye_gain_jac_meas_T[n_s * n_s];
    utilities.matrix_transpose(eye_gain_jac_meas, n_s, n_s, eye_gain_jac_meas_T);
    utilities.matrix_mult(s2_tmp, n_s, n_s, eye_gain_jac_meas_T, n_s, n_s, s2_post_prev, n_s, n_s);

    double kr[n_s * n_m];
    // FULL: utilities.matrix_mult(gain, n_s, n_m, meas_noise, n_m, n_m, kr, n_s, n_m);
    double meas_noise_trunc[n_nonnan_z * n_nonnan_z];
    double kr_trunc[n_s * n_nonnan_z];
    utilities.get_rows_cols(meas_noise, n_m, n_m, nonnan_z_idx, n_nonnan_z, meas_noise_trunc);
    utilities.matrix_mult(gain_trunc, n_s, n_nonnan_z, meas_noise_trunc, n_nonnan_z, n_nonnan_z, kr_trunc, n_s, n_nonnan_z);

    double proj_noise[n_s * n_s];
    // FULL: utilities.matrix_mult(kr, n_s, n_m, gain_T, n_m, n_s, proj_noise, n_s, n_s);
    utilities.matrix_mult(kr_trunc, n_s, n_nonnan_z, gain_T_trunc, n_nonnan_z, n_s, proj_noise, n_s, n_s);
    for (i=0; i<n_s; i++)
    {
        for (j=0; j<n_s; j++)
        {
            s2_post[i*n_s + j] = s2_post_prev[i*n_s + j] + proj_noise[i*n_s + j];
        }
    }
}

void Filter::estimate_measurements(double* x, double* zhat)
{
    model.estimate_measurements(x, zhat);
}

void Filter::initialize_state()
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

void Filter::initarrays()
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

Filter::~Filter()
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
