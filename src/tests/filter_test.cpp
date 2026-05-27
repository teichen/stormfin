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

const double PI = 3.14159265358979323846;

int main()
{
    KalmanFilter kf;
    ComplementaryFilter cf;

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

    x[model.si_omega_x] = PI; // [=] rad / s
    double dt = 1.0; // [=] s

    // first, for the off-board sim Kalman filter
    kf.process(dt, x, s2, u, z);
    // now try the on-board embedded complementary filter
    cf.process(dt, x, u, z);

    for (i=0; i<model.n_s; i++)
    {
        if ((i == model.si_theta_x) or (i == model.si_omega_x))
        {
            assert(std::abs(kf.x_post[i] - PI) < 0.1); // angular displacement integrated over 1s
            assert(std::abs(cf.x_post[i] - PI) < 0.1);
        }
        else
        {
            assert(std::abs(kf.x_post[i]) < 1.0e-5);
            assert(std::abs(cf.x_post[i]) < 1.0e-5);
        }
    }

    // TEST-1 : pass in omega_y as a measurement
    z[model.mi_omega_y] = PI;

    // keep running the two filters in parallel for testing
    kf.process(dt, x, s2, u, z); // start from previous x, omega_y responds
    cf.process(dt, x, u, z);

    kf.utilities.set_elements(kf.x_post, x, model.n_s, 1);
    kf.utilities.set_elements(kf.s2_post, s2, model.n_s, 2);

    cf.utilities.set_elements(cf.x_post, x, model.n_s, 1);

    kf.process(dt, x, s2, u, z); // theta_y integrates omega_y
    // half of the innovation dumps onto epsilon_y unless hacked off meas jac

    cf.process(dt, x, u, z);

    for (i=0; i<model.n_s; i++)
    {
        if (i == model.si_theta_x)
        {
            assert(std::abs(kf.x_post[i] - 2*PI) < 0.1); // angular displacement integrated over 1s
            assert(std::abs(cf.x_post[i] - 2*PI) < 0.1);
        }
        else if (i == model.si_omega_x)
        {
            assert(std::abs(kf.x_post[i] - PI) < 0.1);
            assert(std::abs(cf.x_post[i] - PI) < 0.1);
        }
        else if ((i == model.si_theta_y) or (i == model.si_omega_y)) // response to gyro measurement
        {
            // finite noise, not matching PI exactly
            assert(std::abs(kf.x_post[i] - PI) < 0.2); // angular displacement integrated over 1s
            assert(std::abs(cf.x_post[i] - PI) < 0.2);
        }
        else
        {
            assert(std::abs(kf.x_post[i]) < 1.0e-4);
            assert(std::abs(cf.x_post[i]) < 1.0e-4);
        }
    }

    return 0;
}
