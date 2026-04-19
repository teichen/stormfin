// Model.h
#ifndef _MODEL
#define _MODEL

#include <iostream>

using namespace std;

class Model
{
public:

    Model();
    
    int n_states;
    int n_inputs;

    double* linearized_rate;
    double* linearized_jacobian;
    double* linearized_laplacian;

    ~Model();

private:
};

#endif
