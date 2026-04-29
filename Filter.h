// Filter.h
#ifndef _FILTER
#define _FILTER

#include "LaminarModel.h"
#include "Utilities.h"

#include <iostream>

using namespace std;

class Filter
{
public:

    bool mem_test;

    Filter();

    Utilities utilities;
    LaminarModel model;

    int n_s;
    int n_m;
    double* x_post;
    double* x_prior;
    double* s2_post;
    double* s2_prior;
    double* linearized_rate;
    double* linearized_jacobian;

    int n_nonnan_z;
    int* nonnan_z_idx;

    void initialize_state();
    void initarrays();

    void process(double, double*, double*, double*, double*);

    void estimate_measurements(double*, double*);

    void update(double*);

    ~Filter();

private:
};

#endif
