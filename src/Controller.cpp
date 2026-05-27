#include "Controller.h"
#include <gperftools/profiler.h>
#include <stdlib.h>
#include <cstring>

using namespace std;

static string filter_type = "on-board"; // on-board: ComplementaryFilter, off-board: KalmanFilter

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

Controller::~Controller()
{
}

