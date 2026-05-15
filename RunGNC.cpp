#include <stdlib.h>
#include "RunGNC.h"

using namespace std;

RunGNC::RunGNC()
{
}

void RunGNC::acquire_target(double* q, double* u, double dt, double d, double* r_nav){
    /* calculate r_nav from damped inertia
       q: quarternion data, quat.w(), quat.x(), quat.y(), quat.z()
       u: last thrust configuration before damped inertia
       dt: time difference since r_body was acquired (start of STOP until target lost)
       d: distance at trigger of STOP state
       r_nav: estimate vector to target
    */
    // 2s velocity decay due to water drag, v(t) = v(0) * exp(-dt / tau) with tau=2s
    // on target initially
    // account for drag, 1N ~ 60cm/s
    double tau = 2.0;
    // TODO: DRY with LaminarModel
    double thrust_utilization = 0.5; // forward thrust excluding rotation

    double v[3];
    v[0] = (5.0 / 2.0) * (u[1] - u[0]); // omega_z * y, M=R=1?
    v[1] = thrust_utilization * (u[0] + u[1]);
    v[2] = 0.0;

    // initially oriented along y when target acquired
    double r_body[3];
    r_body[0] = 0.0 + v[0] * dt - v[0] / (2.0 * tau) * dt * dt;
    r_body[1] = d + v[1] * dt - v[1] / (2.0 * tau) * dt * dt;
    r_body[2] = 0.0;

    sensors.body_to_nav(q, r_body, r_nav);
}

void RunGNC::qrot_imu_data(double* q, double* omega_body, double* mag_body, double* a_body, double* omega_nav, double* mag_nav, double* a_nav)
{
    int i;
    for (i=0; i<4; i++)
    {
        omega_nav[i] = omega_body[i];
    }
    sensors.qrot_pure(q, omega_nav);

    for (i=0; i<4; i++)
    {
        mag_nav[i] = mag_body[i];
    }
    sensors.qrot_pure(q, mag_nav);

    for (i=0; i<4; i++)
    {
        a_nav[i] = a_body[i];
    }
    sensors.qrot_pure(q, a_nav);
}

RunGNC::~RunGNC()
{
}

