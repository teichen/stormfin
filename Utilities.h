// Utilities.h
#ifndef _UTILITIES
#define _UTILITIES

#include <gsl/gsl_blas.h>
#include <iostream>
#include "LaminarModel.h"

using namespace std;

class Utilities
{
public:

    Utilities();

    void get_rows(double*, int, int, int*, int, double*);
    void get_cols(double*, int, int, int*, int, double*);
    void get_rows_cols(double*, int, int, int*, int, double*);
    void matrix_transpose(double*, int, int, double*);
    void matrix_exponential(double*, int, double*);
    void matrix_inv(double*, int, int, double*);
    void matrix_mult(double*, int, int, double*, int, int, double*, int, int);
    void ode_iv(LaminarModel&, double*, double*, int, double, double*);
    void set_elements(double*, double*, int, int);
    void unity(int, double*);

    ~Utilities();

private:
};

#endif
