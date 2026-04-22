// Controller.h
#ifndef _CONTROLLER
#define _CONTROLLER

#include "Filter.h"
#include "Collocation.h"

#include <iostream>

using namespace std;

class Controller
{
public:

    Controller();

    Filter filter;
    Collocation collocation;

    void process(double, double*, double*, double*, double*);

    ~Controller();

private:
};

#endif
