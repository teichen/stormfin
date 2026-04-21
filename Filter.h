// Filter.h
#ifndef _FILTER
#define _FILTER

#include "Model.h"
#include "RungeKutta.h"
#include "Utilities.h"

#include <iostream>

using namespace std;

class Filter
{
public:

    bool mem_test;

    Filter();
    void init_model(Model&);

    RungeKutta propagator;
    Utilities utilities;

    int n;
    int n_in;
    double* x_post;
    double* x_prior;
    double* sig_post;
    double* sig_prior;
    double* jacobian;
    double* laplacian;

    double t0;

    void initialize_state();
    void initarrays();

    void propagate_update(double, double*);

    void calc_estimates(double*, double*);

    void set_prior(double*, double*);
    void set_posterior(double*, double*);
    void update(double*, double*);

    ~Filter();

private:
};

#endif
