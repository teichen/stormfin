#include "Controller.h"
#include <gperftools/profiler.h>
#include <stdlib.h>
#include <cstring>
#include <chrono>

static int STOP = 0;
static int DIVE = 1;
static int SURFACE = 2;
static int SURVEILLANCE = 3;
static int STALK = 4;
static int COMMUNICATE = 5;

using namespace std;
using namespace std::chrono_literals; 

using SystemTimePoint = std::chrono::system_clock::time_point;

static string filter_type = "on-board"; // on-board: ComplementaryFilter, off-board: KalmanFilter
static auto dt1s = 1s;

static double dive_period = 50.0 * 60.0; // 5 min survey period
static double surface_period = 1.0 * 60.0; // 1 min GPS collection at surface
static double comms_off_period = 2.0 * 60.0; // 2 min acoustic signal silence, comms prioritized over surfacing
static double comms_on_period = 30.0; // 30 second acoustic communication period

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

void Controller::update_nav_state(int nav_state, SystemTimePoint t, SystemTimePoint t_comms_off, SystemTimePoint t_comms_on,
                                 SystemTimePoint t_surface, SystemTimePoint t_dive, double d, double* q,
                                 int idx_set, double* t_set, int n_t, double* u_set,
                                 double* u_saved, double d_stop, SystemTimePoint t_stop, double d0)
{
    double dt_surface = 0.0;
    double dt_dive = 0.0;
    double dt_comms_off = 0.0;
    double dt_comms_on = 0.0;
    double dt_closest = 3600.0;
    double dt_stop = 0.0;
    double r_nav[3];
    int i;

    dt_stop = (t - t_stop) / dt1s;
    dt_dive = (t - t_surface) / dt1s; // time since last surface
    dt_surface = (t - t_dive) / dt1s; // time since last dive

    dt_comms_off = (t - t_comms_off) / dt1s; // time since stop of last communication
    dt_comms_on = (t - t_comms_on) / dt1s; // time since start of communication

    if (nav_state == COMMUNICATE) // fixed communication period
    {
        if (dt_comms_on > comms_on_period)
        {
            nav_state = SURVEILLANCE; // resume navigation
            t_comms_off = t;
        }
    }
    if (dt_comms_off > comms_off_period) // regularized communication
    {
        nav_state = COMMUNICATE;
        t_comms_on = t;
    }
    else if (dt_dive > dive_period) // regularized surfacing
    {
        nav_state = SURFACE;
        t_surface = t;
    }
    else if (dt_surface > surface_period) // regularized diving
    {
        nav_state = DIVE;
        t_dive = t;
    }
    else
    {
        if (d < 30){
            // abandon
            nav_state = SURFACE;

            // TODO: communicate during events such as collision
        }
        else if (d > 500){ // 600cm (20ft) operating limit, lost the target
            // do we have target thrust from our collocation?
            idx_set = n_t;
            dt_closest = 3600.0;
            for (i=0; i<n_t; i++)
            {
                if (std::abs(t_set[i] - dt_stop) < dt_closest)
                {
                    dt_closest = std::abs(t_set[i] - dt_stop);
                    idx_set = i;
                }
            }
            if (idx_set >= n_t) // if thrust profile burned, resume circling surveillance
            {
                nav_state = SURVEILLANCE;
            }
            else
            {
                if (nav_state == STOP)
                {
                    // previously stopped, damped inertia, target acquisition
                    dt_stop = (t - t_stop) / dt1s;
                    acquire_target(q, u_saved, dt_stop, d_stop, r_nav);

                    // calculate thrust profile provided vector to target, r_nav
                    // t_set = (t_0, t_1, t_2, ..., t_nt)
                    // u_set = (u_0, u_1, u_2, ..., u_nt) where u_0 = u[t_0] setting
                    collocation.optimal_thrust(r_nav, t_set, n_t, u_set);
                }
                nav_state = STALK; // close the distance
            }
        }
        else {
            if (std::abs(d - d0) < 5){ // static target
                nav_state = STOP; // observe
                d_stop = d;
                t_stop = t;
            }
            else if (d > d0){ // assume carry forward on current trajectory
                nav_state = STALK;
            }
            else { // target approaching, update distance
                nav_state = STOP;
                d_stop = d;
                t_stop = t;
            }
        }
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


