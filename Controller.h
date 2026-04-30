// Controller.h
#ifndef _CONTROLLER
#define _CONTROLLER

#include "Utilities.h"
#include "KalmanFilter.h"
#include "ComplementaryFilter.h"
#include "Collocation.h"

#include <iostream>

using namespace std;

class Controller
{
public:

    Controller();

    Utilities utilities;
    KalmanFilter ekf;
    ComplementaryFilter cf;
    Collocation collocation;

    void process(double, double*, double*, double*, double*);

    ~Controller();

private:
};

#endif
