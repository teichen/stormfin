#include <iostream>
#include <cassert>
using std::cerr;
using std::cout;
using std::endl;
#include <math.h>
#include <cmath>
#include <chrono>
#include "../Controller.h"

static int STOP = 0;
static int DIVE = 1;
static int SURFACE = 2;
static int SURVEILLANCE = 3;
static int STALK = 4;
static int COMMUNICATE = 5;

using namespace std;
using namespace std::chrono_literals; 

using SystemTimePoint = std::chrono::system_clock::time_point;

static auto dt1s = 1s;

static double dive_period = 50.0 * 60.0; // 5 min survey period
static double surface_period = 1.0 * 60.0; // 1 min GPS collection at surface
static double comms_off_period = 2.0 * 60.0; // 2 min acoustic signal silence, comms prioritized over surfacing
static double comms_on_period = 30.0; // 30 second acoustic communication period

using namespace std;

int main()
{
    // unit testing with run time asserts
    Controller controller;

    // TEST-0 : calculate navigation vector from damped inertia
    double q[4]; // q: quarternion data, quat.w(), quat.x(), quat.y(), quat.z()
    q[0] = 0.0; q[1] = 0.0; q[2] = 0.0; q[3] = 0.0;
    double u[3]; // u: last thrust configuration before damped inertia
    u[0] = 0.0; u[1] = 0.0; u[2] = 0.0;
    double dt = 2.0; // 2 seconds, dt: time difference since r_body was acquired (start of STOP until target lost)
    double d = 30.0; // 30 cm stop distance, d: distance at trigger of STOP state
    // r_nav: estimate vector to target
    double r_nav[3];
    controller.acquire_target(q, u, dt, d, r_nav);
    assert(r_nav[0] == 0.0);
    assert(r_nav[1] == 30.0);
    assert(r_nav[2] == 0.0);
    // TODO: off-axis drift test

    // TEST-1 : update navigation state
    int nav_state = SURFACE;
    double d0 = d;
    double d_stop = d;
    SystemTimePoint t0 = std::chrono::system_clock::now();
    SystemTimePoint t = t0;
    SystemTimePoint t_stop = t0;

    SystemTimePoint t_dive = t0;
    SystemTimePoint t_surface = t0;
    SystemTimePoint t_comms_off = t0;
    SystemTimePoint t_comms_on = t0;

    t = t0 + 2 * 60 * dt1s; // after 1 min should dive

    // thrust setpoints
    int n_t = 100;
    double t_set[n_t];
    double u_set[n_t * 3];
    int i;
    for (i=0; i<n_t; i++)
    {
        t_set[i] = 0.0;
        u_set[i*3 + 0] = 0.0;
        u_set[i*3 + 1] = 0.0;
        u_set[i*3 + 2] = 0.0;
    }

    int idx_set;
    double u_saved[3];
    for (i=0; i<3; i++)
    {
        u_saved[i] = u[i];
    } 

    controller.update_nav_state(nav_state, t, t_comms_off, t_comms_on,
                                 t_surface, t_dive, d, q,
                                 idx_set, t_set, n_t, u_set,
                                 u_saved, d_stop, t_stop, d0);
    assert(nav_state == DIVE);

    // TODO: acquire target, calculate optimal thrust, update nav_state

    return 0;
}
