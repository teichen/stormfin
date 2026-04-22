#include "Model.h"
#include <stdlib.h>

using namespace std;

Model::Model()
{
}

int Model::rate(double t, const double x[], double f[], void *params)
{
    return 0;
}

int Model::jacobian(double t, const double x[], double *dfdx, double dfdt[], void *params)
{
    return 0;
}

Model::~Model()
{
}
