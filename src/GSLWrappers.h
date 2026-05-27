// GSLWrappers.h
#ifndef _GSLWRAPPERS
#define _GSLWRAPPERS

#include <gsl/gsl_blas.h>
#include <iostream>
#include "LaminarModel.h"

using namespace std;

class GSLWrappers
{
public:

    GSLWrappers();

    void matrix_exponential(double*, int, double*);
    void matrix_inv(double*, int, int, double*);
    void matrix_mult(double*, int, int, double*, int, int, double*, int, int);
    void ode_iv(LaminarModel&, double*, double*, int, double, double*);

    ~GSLWrappers();

private:
};

#endif
