// LaminarModel.h
#ifndef _LAMINARMODEL
#define _LAMINARMODEL

#include "Model.h"

#include <iostream>

using namespace std;

class LaminarModel : public Model
{
public:

    LaminarModel();

    bool mem_test;

    static int rate(double, const double[], double[], void*);
    static int jacobian(double, const double[], double*, double[], void*);

    void linearized_rate(double*, double*);
    void linearized_jacobian(double*, double*);

    void init_meas_noise(double*);
    void init_state(double*);
    void init_covariance(double*);

    void estimate_measurements(double*, double*);
    void measurement_jacobian(double*, double*);
    void linearized_measurement_jacobian(double*, double*);

    void initarrays();

    ~LaminarModel();

private:
};

#endif
