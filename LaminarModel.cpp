#include "LaminarModel.h"
#include <stdlib.h>
#include <gsl/gsl_errno.h>

using namespace std;

static int n_states = 21;
static int n_inputs = 8;

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

LaminarModel::LaminarModel()
{
    /* underactuated tethered underwater drone
       lakes and rivers, assume no turbulence (predictable current or no current)
    */
    n = n_states;
    n_in = n_inputs;
}


int LaminarModel::rate(double t, const double x[], double f[], void *params)
{
    int i,j;

    f[SI_THETA_X] = x[SI_OMEGA_X];
    f[SI_THETA_Y] = x[SI_OMEGA_Y];
    f[SI_THETA_Z] = x[SI_OMEGA_Z];
    f[SI_OMEGA_X] = 0.0;
    f[SI_OMEGA_Y] = 0.0;
    f[SI_OMEGA_Z] = 0.0;
    f[SI_X] = x[SI_V_X];
    f[SI_Y] = x[SI_V_Y];
    f[SI_Z] = x[SI_V_Z];
    f[SI_V_X] = x[SI_A_X];
    f[SI_V_Y] = x[SI_A_Y];
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


void LaminarModel::initarrays()
{
}

LaminarModel::~LaminarModel()
{
}

