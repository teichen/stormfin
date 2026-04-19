// RungeKutta.h
#ifndef _RUNGEKUTTA
#define _RUNGEKUTTA

#include "Model.h"

#include <iostream>

using namespace std;

class RungeKutta
{
public:

    RungeKutta();
    void init_model(Model&);

    bool mem_test;

    int n;
    void initarrays();

    double* rate;

    void displacement(double*, double, double*);
    void propagate(double, double, double, double*);

    ~RungeKutta();

private:
};

#endif
