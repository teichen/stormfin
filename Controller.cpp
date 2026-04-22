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

void Controller::process(double dt, double* x_post, double* s2_post, double* thrust, double* measurements)
{
    /* update best estimates forward in time with timestep dt
       (a) if no measurements processed, propagate with updated thrust
       (b) if measurements process, propagate and update with sensor data using extended Kalman filter
    */
    filter.process(dt, x_post, s2_post, thrust, measurements);    
}

Controller::~Controller()
{
}

