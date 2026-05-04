#include "ComplementaryFilter.h"
#include <stdlib.h>
#include <cmath>

using namespace std;

static double alpha_att = 0.9; // stronger weight gyro data over accelerometer data
static double alpha_trans = 0.9; // stronger weight accelerometer data over GPS data

ComplementaryFilter::ComplementaryFilter()
{
    mem_test = false;

    n_s = model.n_s;
    n_m = model.n_m;

    initarrays();

    initialize_state();
}

void ComplementaryFilter::process(double dt, double* x, double* thrust, double* measurements)
{
    /* sensor fusion of gyro and accelerometer data for attitude (orientation)
       sensor fusion of accelerometer and GPS data for translation
       using a low pass and high pass filtering scheme
    */
    utilities.set_elements(x, x_prior, n_s, 1);

    // reduced model contains the attitude model to propagate for the low pass filter of gyrometer data
    // reduced model contains the translation model to propagate for the low pass filter of accelerometer data
    utilities.ode_iv(model, x, x_prior, n_s, dt, thrust);

    // set prior with sensor data (assuming no noise)
    if (!std::isnan(measurements[model.mi_a_x]))
    {
        x_prior[model.si_a_x] = measurements[model.mi_a_x];
    }
    if (!std::isnan(measurements[model.mi_a_y]))
    {
        x_prior[model.si_a_y] = measurements[model.mi_a_y];
    }
    if (!std::isnan(measurements[model.mi_a_z]))
    {
        x_prior[model.si_a_z] = measurements[model.mi_a_z];
    }
    if (!std::isnan(measurements[model.mi_omega_x]))
    {
        x_prior[model.si_omega_x] = measurements[model.mi_omega_x];
    }
    if (!std::isnan(measurements[model.mi_omega_y]))
    {
        x_prior[model.si_omega_y] = measurements[model.mi_omega_y];
    }
    if (!std::isnan(measurements[model.mi_omega_z]))
    {
        x_prior[model.si_omega_z] = measurements[model.mi_omega_z];
    }

    update(measurements);
}

void ComplementaryFilter::update(double* measurements)
{
    utilities.set_elements(x_prior, x_post, n_s, 1);

    // attitude
    // high pass filter is input linear accelerometer and gyro magnetic data
    // low pass filter is integrated angular velocity
    double roll_from_acc, pitch_from_acc, yaw_from_acc;
    if (!std::isnan(measurements[model.mi_a_x]) and !std::isnan(measurements[model.mi_a_y]) and !std::isnan(measurements[model.mi_a_z]))
    {
        roll_from_acc = std::atan(measurements[model.mi_a_y] / measurements[model.mi_a_z]);
        pitch_from_acc = std::atan(-measurements[model.mi_a_x] / (measurements[model.mi_a_y] * std::sin(roll_from_acc) + measurements[model.mi_a_z] * std::cos(roll_from_acc)));
        if (!std::isnan(measurements[model.mi_b_x]) and !std::isnan(measurements[model.mi_b_y]) and !std::isnan(measurements[model.mi_b_z]))
        {
            yaw_from_acc = std::atan((measurements[model.mi_b_z] * std::sin(roll_from_acc) - measurements[model.mi_b_y] * std::cos(roll_from_acc)) / (measurements[model.mi_b_x] * std::cos(pitch_from_acc) + measurements[model.mi_b_y] * std::sin(pitch_from_acc) * std::sin(roll_from_acc) + measurements[model.mi_b_z] * std::sin(pitch_from_acc) * std::cos(roll_from_acc)));
        }
        else
        {
            yaw_from_acc = 0.0;
        }
    }
    else
    {
        roll_from_acc = 0.0;
        pitch_from_acc = 0.0;
    }

    if (roll_from_acc > 0.0)
    {
        x_post[model.si_theta_x] = alpha_att * x_prior[model.si_theta_x] + (1.0 - alpha_att) * roll_from_acc;
    }
    else
    {
        x_post[model.si_theta_x] = x_prior[model.si_theta_x];
    }
    if (pitch_from_acc > 0.0)
    {
        x_post[model.si_theta_y] = alpha_att * x_prior[model.si_theta_y] + (1.0 - alpha_att) * pitch_from_acc;
    }
    else
    {
        x_post[model.si_theta_y] = x_prior[model.si_theta_y];
    }
    if (yaw_from_acc > 0.0)
    {
        x_post[model.si_theta_z] = alpha_att * x_prior[model.si_theta_z] + (1.0 - alpha_att) * yaw_from_acc;
    }
    else
    {
        x_post[model.si_theta_z] = x_prior[model.si_theta_z];
    }

    // translation
    // GPS velocity uses Doppler shifts and is more reliable than position
    // high pass filter is input GPS velocity
    // low pass filter is integrated linear accelerometer data
    if (!std::isnan(measurements[model.mi_v_x]))
    {
        x_post[model.si_v_x] = alpha_trans * x_prior[model.si_v_x] + (1.0 - alpha_trans) * measurements[model.mi_v_x];
    }
    else
    {
        x_post[model.si_v_x] = x_prior[model.si_v_x];
    }
    if (!std::isnan(measurements[model.mi_v_y]))
    {
        x_post[model.si_v_y] = alpha_trans * x_prior[model.si_v_y] + (1.0 - alpha_trans) * measurements[model.mi_v_y];
    }
    else
    {
        x_post[model.si_v_y] = x_prior[model.si_v_y];
    }
}

void ComplementaryFilter::initialize_state()
{
    model.init_state(x_prior);

    int i, j;
    
    for (i=0; i<n_s; i++)
    {
        x_prior[i] = 0.0;
        x_post[i]  = 0.0;
    }
}

void ComplementaryFilter::initarrays()
{
    x_prior   = (double*) calloc (n_s, sizeof(double));
    x_post    = (double*) calloc (n_s, sizeof(double));

    mem_test  = true;
}

ComplementaryFilter::~ComplementaryFilter()
{
    if(mem_test==true)
    {
    delete [] x_prior;
    delete [] x_post;
    }
}
