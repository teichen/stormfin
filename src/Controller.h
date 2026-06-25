// Controller.h
#ifndef _CONTROLLER
#define _CONTROLLER

#include "Utilities.h"
#include "KalmanFilter.h"
#include "ComplementaryFilter.h"
#include "Collocation.h"
#include "Sensors.h"

#include <iostream>
#include <chrono>

using SystemTimePoint = std::chrono::system_clock::time_point;

using namespace std;

class Controller
{
public:

    Controller();

    Utilities utilities;
    KalmanFilter ekf;
    ComplementaryFilter cf;
    Collocation collocation;

    void process_data(double, double*, double*, double*, double*);
    Sensors sensors; // utility functions, e.g. ref frame rotations
    void acquire_target(double*, double*, double, double, double*);
    void qrot_imu_data(double*, double*, double*, double*, double*, double*, double*);
    void update_nav_state(int&, SystemTimePoint, SystemTimePoint, SystemTimePoint, SystemTimePoint,
                          SystemTimePoint, double, double*,
                          int, double*, int, double*, double*, double, SystemTimePoint, double);
    ~Controller();

private:
};

#endif
