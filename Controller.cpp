#include "Controller.h"
#include <gperftools/profiler.h>

#include <iostream>
#include <cstring>

using namespace std;

Controller::Controller()
{
    /* controller to run Kalman filter
    */
    filter.init_model(model);

}

void Controller::update_filter(double t, double* input_data)
{
    /* extended Kalman filter update
    */
    filter.propagate_update(t, input_data);    
}

Controller::~Controller()
{
}

