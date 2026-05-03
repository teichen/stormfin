#include <iostream>
#include <cassert>
using std::cerr;
using std::cout;
using std::endl;
#include <math.h>
#include <cmath>
#include "../KalmanFilter.h"
#include "../ComplementaryFilter.h"

using namespace std;

// TODO: DRY
const double PI = 3.14159265358979323846;
static int SI_THETA_X = 0; // angular displacement
static int SI_THETA_Y = 1;
static int SI_THETA_Z = 2;
static int SI_OMEGA_X = 3; // angular velocity
static int SI_OMEGA_Y = 4;
static int SI_OMEGA_Z = 5;
static int SI_X = 6; // linear displacement
static int SI_Y = 7;
static int SI_Z = 8;
static int SI_V_X = 9; // linear velocity
static int SI_V_Y = 10;
static int SI_V_Z = 11;
static int SI_A_X = 12; // linear acceleration
static int SI_A_Y = 13;
static int SI_A_Z = 14;
static int SI_EPSILON_X = 15; // gyro drift
static int SI_EPSILON_Y = 16;
static int SI_EPSILON_Z = 17;
static int SI_BETA_X = 18; // accel bias
static int SI_BETA_Y = 19;
static int SI_BETA_Z = 20;

static int MI_OMEGA_X = 0; // IMU gyro
static int MI_OMEGA_Y = 1;
static int MI_OMEGA_Z = 2;
static int MI_A_X = 3; // IMU accel
static int MI_A_Y = 4;
static int MI_A_Z = 5;
static int MI_X = 6; // GPS
static int MI_Y = 7;
static int MI_V_X = 8;
static int MI_V_Y = 9;
static int MI_B_X = 10; // IMU mag
static int MI_B_Y = 11;
static int MI_B_Z = 12;

int main()
{
    KalmanFilter filter;

    // TEST-0 : propagate model state and covariance
    LaminarModel model;

    double x[model.n_s];
    double s2[model.n_s * model.n_s];
    double u[model.n_u];
    double z[model.n_m];

    int i, j;
    for (i=0; i<model.n_s; i++)
    {
        x[i] = 0.0;
        for (j=0; j<model.n_s; j++)
        {
            if (i == j)
            {
                s2[i * model.n_s + j] = 1.0e-5;
            }
            else
            {
                s2[i * model.n_s + j] = 0.0;
            }
        }
    }
    for (i=0; i<model.n_u; i++)
    {
        u[i] = 0.0; // no thrust
    }    
    for (i=0; i<model.n_m; i++)
    {
        z[i] = std::nan(""); // no sensor data
    }

    x[SI_OMEGA_X] = PI; // [=] rad / s
    double dt = 1.0; // [=] s

    filter.process(dt, x, s2, u, z);

    for (i=0; i<model.n_s; i++)
    {
        if ((i == SI_THETA_X) or (i == SI_OMEGA_X))
        {
            assert(std::abs(filter.x_post[i] - PI) < 0.1); // angular displacement integrated over 1s
        }
        else
        {
            assert(std::abs(filter.x_post[i]) < 1.0e-5);
        }
    }

    // TEST-1 : pass in omega_y as a measurement
    z[MI_OMEGA_Y] = PI;

    filter.process(dt, x, s2, u, z); // start from previous x, omega_y responds

    filter.utilities.set_elements(filter.x_post, x, model.n_s, 1);
    filter.utilities.set_elements(filter.s2_post, s2, model.n_s, 2);

    filter.process(dt, x, s2, u, z); // theta_y integrates omega_y
    // half of the innovation dumps onto epsilon_y unless hacked off meas jac

    for (i=0; i<model.n_s; i++)
    {
        if (i == SI_THETA_X)
        {
            assert(std::abs(filter.x_post[i] - 2*PI) < 0.1); // angular displacement integrated over 1s
        }
        else if (i == SI_OMEGA_X)
        {
            assert(std::abs(filter.x_post[i] - PI) < 0.1);
        }
        else if ((i == SI_THETA_Y) or (i == SI_OMEGA_Y)) // response to gyro measurement
        {
            // finite noise, not matching PI exactly
            assert(std::abs(filter.x_post[i] - PI) < 0.2); // angular displacement integrated over 1s
        }
        else
        {
            assert(std::abs(filter.x_post[i]) < 1.0e-4);
        }
    }

    return 0;
}
