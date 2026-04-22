// Model.h
#ifndef _MODEL
#define _MODEL

#include <iostream>

using namespace std;

class Model
{
public:

    Model();

    double* linearized_rate;
    double* linearized_jacobian;

    static int rate(double, const double[], double[], void*);
    // TODO: jacobian    

    ~Model();

private:
};

#endif
