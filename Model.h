// Model.h
#ifndef _MODEL
#define _MODEL

#include <iostream>

using namespace std;

class Model
{
public:

    Model();

    int n;
    int n_in;

    static int rate(double, const double[], double[], void*);
    static int jacobian(double, const double[], double*, double[], void*);

    ~Model();

private:
};

#endif
