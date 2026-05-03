#include "LaminarModel.h"
#include <stdlib.h>

#define GSL_SUCCESS 0

using namespace std;

static int n_states = 21;
static int n_measurements = 13;
static int n_thrusters = 3;

// all states and measurements in the NAV frame

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

static int UI_L = 0; // left thruster (forward/reverse)
static int UI_R = 1; // right thruster (forward/reverse)
static int UI_V = 2; // vertical thruster (dive/surface)

static double thrust_utilization = 0.5; // forward thrust excluding rotation

LaminarModel::LaminarModel()
{
    /* underactuated tethered underwater drone
       lakes and rivers, assume no turbulence (predictable current or no current)
    */
    n_s = n_states;
    n_m = n_measurements;
    n_u = n_thrusters;
}

void LaminarModel::init_meas_noise(double *meas_noise)
{
    /*
       accelerometer bias, beta, propagates into velocity and position error
       50 <= beta <= 1000 microg
       ~2500 (microg) ** 2 / Hz white noise

       gyro bias or drift rate, epsilon, integrates into attitude error
       0.01 <= epsilon <= 10 deg / hr
       ~1.e-6 (deg / s) ** 2 / Hz white noise
    */
    int i;
    for (i=0; i<n_m; i++)
    {
        meas_noise[i * n_m + i] = 1.0e-6;
    }
}

void LaminarModel::init_state(double *x)
{
    int i;
    for (i=0; i<n_s; i++)
    {
        x[i] = 0.0;
    }
}

void LaminarModel::init_covariance(double *s2)
{
    int i, j;
    for (i=0; i<n_s; i++)
    {
        for (j=0; j<n_s; j++)
        {
            if (i == j)
            {
                s2[i*n_s + j] = 1.0e-6;
            }
            else
            {
                s2[i*n_s + j] = 0.0;
            }
        }
    }
}

int LaminarModel::rate(double t, const double x[], double f[], void *u)
{
    int i,j;

    double *thrust = (double *)u;

    f[SI_THETA_X] = x[SI_OMEGA_X];
    f[SI_THETA_Y] = x[SI_OMEGA_Y];
    f[SI_THETA_Z] = x[SI_OMEGA_Z];
    f[SI_OMEGA_X] = 0.0;
    f[SI_OMEGA_Y] = 0.0;
    f[SI_OMEGA_Z] = (5.0 / 2.0) * (thrust[UI_R] - thrust[UI_L]); // net torque causing rotation, M=R=1?
    f[SI_X] = x[SI_V_X];
    f[SI_Y] = x[SI_V_Y];
    f[SI_Z] = x[SI_V_Z];
    f[SI_V_X] = x[SI_A_X];
    f[SI_V_Y] = x[SI_A_Y] + thrust_utilization * (thrust[UI_R] + thrust[UI_L]);
    f[SI_V_Z] = x[SI_A_Z];
    f[SI_EPSILON_X] = 0.0;
    f[SI_EPSILON_Y] = 0.0;
    f[SI_EPSILON_Z] = 0.0;
    f[SI_BETA_X] = 0.0;
    f[SI_BETA_Y] = 0.0;
    f[SI_BETA_Z] = 0.0;

    return GSL_SUCCESS;
}

int LaminarModel::jacobian(double t, const double x[], double *dfdx, double dfdt[], void *params)
{
    int i, j;
    for (i=0; i<n_states; i++)
    {
        dfdt[i] = 0.0;
        for (j=0; j<n_states; j++)
        {
            dfdx[i * n_states + j] = 0.0;
        }
    }
    
    dfdt[SI_X] = x[SI_A_X];
    dfdt[SI_Y] = x[SI_A_Y];
    dfdt[SI_Z] = x[SI_A_Z];

    dfdx[SI_THETA_X * n_states + SI_THETA_X] = 1.0;
    dfdx[SI_THETA_Y * n_states + SI_THETA_Y] = 1.0;
    dfdx[SI_THETA_Z * n_states + SI_THETA_Z] = 1.0;
    dfdx[SI_X * n_states + SI_V_X] = 1.0;
    dfdx[SI_Y * n_states + SI_V_Y] = 1.0;
    dfdx[SI_Z * n_states + SI_V_Z] = 1.0;
    dfdx[SI_V_X * n_states + SI_A_X] = 1.0;
    dfdx[SI_V_Y * n_states + SI_A_Y] = 1.0;
    dfdx[SI_V_Z * n_states + SI_A_Z] = 1.0;

    return GSL_SUCCESS;
}

void LaminarModel::linearized_rate(double *x, double *f)
{
    double t = 0.0;
    int param = 0;
    rate(t, x, f, &param);
}

void LaminarModel::linearized_jacobian(double *x, double *dfdx)
{
    double t = 0.0;
    double dfdt[n_states];
    int param = 0;
    dfdt[SI_X] = x[SI_A_X]; // TODO: DRY
    dfdt[SI_Y] = x[SI_A_Y];
    dfdt[SI_Z] = x[SI_A_Z];

    jacobian(t, x, dfdx, dfdt, &param);
}

void LaminarModel::estimate_measurements(double *x, double *zhat)
{
    zhat[MI_OMEGA_X] = x[SI_OMEGA_X] + x[SI_EPSILON_X];
    zhat[MI_OMEGA_Y] = x[SI_OMEGA_Y] + x[SI_EPSILON_Y];
    zhat[MI_OMEGA_Z] = x[SI_OMEGA_Z] + x[SI_EPSILON_Z];
    zhat[MI_A_X] = x[SI_A_X] + x[SI_BETA_X];
    zhat[MI_A_Y] = x[SI_A_Y] + x[SI_BETA_Y];
    zhat[MI_A_Z] = x[SI_A_Z] + x[SI_BETA_Z];
    zhat[MI_X] = x[SI_X];
    zhat[MI_Y] = x[SI_Y];
    zhat[MI_V_X] = x[SI_V_X];
    zhat[MI_V_Y] = x[SI_V_Y];
}

void LaminarModel::measurement_jacobian(double *x, double *dzhatdx)
{
    int i,j;
    for (i=0; i<n_measurements; i++)
    {
        for (j=0; j<n_states; j++)
        {
            dzhatdx[i * n_states + j] = 0.0;
        }
    }
    dzhatdx[MI_OMEGA_X * n_states + SI_OMEGA_X] = 1.0;
    dzhatdx[MI_OMEGA_Y * n_states + SI_OMEGA_Y] = 1.0;
    dzhatdx[MI_OMEGA_Z * n_states + SI_OMEGA_Z] = 1.0;
    dzhatdx[MI_OMEGA_X * n_states + SI_EPSILON_X] = 1.0;
    dzhatdx[MI_OMEGA_Y * n_states + SI_EPSILON_Y] = 1.0;
    dzhatdx[MI_OMEGA_Z * n_states + SI_EPSILON_Z] = 1.0;
    dzhatdx[MI_A_X * n_states + SI_A_X] = 1.0;
    dzhatdx[MI_A_Y * n_states + SI_A_Y] = 1.0;
    dzhatdx[MI_A_Z * n_states + SI_A_Z] = 1.0;
    dzhatdx[MI_A_X * n_states + SI_BETA_X] = 1.0;
    dzhatdx[MI_A_Y * n_states + SI_BETA_Y] = 1.0;
    dzhatdx[MI_A_Z * n_states + SI_BETA_Z] = 1.0;
    dzhatdx[MI_X * n_states + SI_X] = 1.0;
    dzhatdx[MI_Y * n_states + SI_Y] = 1.0;
    dzhatdx[MI_V_X * n_states + SI_V_X] = 1.0;
    dzhatdx[MI_V_Y * n_states + SI_V_Y] = 1.0;
}

void LaminarModel::linearized_measurement_jacobian(double *x, double *dzhatdx)
{
    measurement_jacobian(x, dzhatdx);
    // gyro drift and accel bias hacked off of IMU innovations
    dzhatdx[MI_OMEGA_X * n_states + SI_EPSILON_X] = 0.0;
    dzhatdx[MI_OMEGA_Y * n_states + SI_EPSILON_Y] = 0.0;
    dzhatdx[MI_OMEGA_Z * n_states + SI_EPSILON_Z] = 0.0;
    dzhatdx[MI_A_X * n_states + SI_BETA_X] = 0.0;
    dzhatdx[MI_A_Y * n_states + SI_BETA_Y] = 0.0;
    dzhatdx[MI_A_Z * n_states + SI_BETA_Z] = 0.0;
}

void LaminarModel::initarrays()
{
}

LaminarModel::~LaminarModel()
{
}

