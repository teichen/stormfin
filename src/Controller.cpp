#include "Controller.h"
#include <gperftools/profiler.h>
#include <stdlib.h>
#include <cstring>

using namespace std;

static string filter_type = "on-board"; // on-board: ComplementaryFilter, off-board: KalmanFilter

Controller::Controller()
{
    /* controller to run Kalman filter
       body frame sufficient for stabilization, navigation frame needed for
       fusion with GPS and fault tolerance
    */
}

void Controller::process_data(double dt, double* x, double* s2, double* thrust, double* measurements)
{
    /* update best estimates forward in time with timestep dt
       (a) if no measurements processed, propagate with updated thrust
       (b) if measurements process, propagate and update with sensor data using extended Kalman filter
    */
    if (filter_type == "on-board")
    {
        cf.process(dt, x, thrust, measurements);
        utilities.set_elements(cf.x_post, x, cf.n_s, 1);
    }
    else if (filter_type == "off-board")
    {
        ekf.process(dt, x, s2, thrust, measurements);    

        // update input mean state and covariance with the posterior estimates in the filter
        utilities.set_elements(ekf.x_post, x, ekf.n_s, 1);
        utilities.set_elements(ekf.s2_post, s2, ekf.n_s, 2);
    }
}

void Controller::acquire_target(double* q, double* u, double dt, double d, double* r_nav){
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

void Controller::qrot_imu_data(double* q, double* omega_body, double* mag_body, double* a_body, double* omega_nav, double* mag_nav, double* a_nav)
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

Controller::~Controller()
{
}


