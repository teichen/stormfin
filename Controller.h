// Controller.h
#ifndef _CONTROLLER
#define _CONTROLLER

#include "LaminarModel.h"
#include "Filter.h"
#include "Collocation.h"

#include <iostream>

using namespace std;

class Controller
{
public:

    Controller();

    Filter filter;
    LaminarModel model;
    Collocation collocation;

    void update_filter(double, double*);

    ~Controller();

private:
};

#endif
