#include "Controller.h"
#include <gperftools/profiler.h>
#include <stdlib.h>
#include <cstring>

using namespace std;

Controller::Controller()
{
    /* controller to run Kalman filter
    */
}

void Controller::process(double dt, double* x, double* s2, double* thrust, double* measurements)
{
    /* update best estimates forward in time with timestep dt
       (a) if no measurements processed, propagate with updated thrust
       (b) if measurements process, propagate and update with sensor data using extended Kalman filter
    */
    filter.process(dt, x, s2, thrust, measurements);    

    // update input mean state and covariance with the posterior estimates in the filter
    utilities.set_elements(filter.x_post, x, filter.n_s, 1);
    utilities.set_elements(filter.s2_post, s2, filter.n_s, 2);
}

Controller::~Controller()
{
}

