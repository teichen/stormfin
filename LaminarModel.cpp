#include "LaminarModel.h"
#include <stdlib.h>

using namespace std;

LaminarModel::LaminarModel()
{
    /* underactuated tethered underwater drone
       lakes and rivers, assume no turbulence (predictable current or no current)
    */
    n_states = 4;
    n_inputs = 3;

    mem_test = false;
    initarrays();

    int i,j;

    // linearized rate of change of state
    for (i=0; i<n_states; i++)
    {
        for (j=0; j<n_states; j++)
        {
            linearized_rate[i*n_states + j] = 0;
        }
    }

    for (i=0; i<(int)(n_states/2); i++)
    {
        linearized_rate[2*i*n_states + 2*(i+1)] = 1;
        linearized_rate[(2*i+1)*n_states + 2*i] = -1;
    }

    // linearized jacobian of input estimates
    // TODO: generalize jacobian, using linearized placeholder
    for (i=0; i<n_inputs; i++)
    {
        for (j=0; j<n_states; j++)
        {
            linearized_jacobian[i*n_states + j] = 0;
        }
    }
    for (i=0; i<n_inputs; i++)
    {
        linearized_jacobian[i*n_states + i] = 1.0;
    }

}

void LaminarModel::rate(double* x, double* r)
{
    int i,j;

    for (i=0; i<n_states; i++)
    {
        for (j=0; j<n_states; j++)
        {
            r[i*n_states + j] = 0;
        }
    }

    for (i=0; i<(int)(n_states/2); i++)
    {
        r[2*i*n_states + 2*(i+1)] = x[2*(i+1)];
        r[(2*i+1)*n_states + 2*i] = -x[2*i];
    }
}

void LaminarModel::map_inputs_states(double* x, double* f)
{
    /* map of states onto inputs
    */
    int i,j;

    for (i=0; i<n_inputs; i++)
    {
        for (j=0; j<n_states; j++)
        {
            f[i*n_states + j] = 0;
        }
    }
    for (i=0; i<n_inputs; i++)
    {
        // TODO: placeholder linear mapping
        f[i*n_states + i] = x[i];
    }
}

void LaminarModel::initarrays()
{
    linearized_rate = (double*) calloc (n_states * n_states, sizeof(double));
    linearized_jacobian = (double*) calloc (n_inputs * n_states, sizeof(double));

    mem_test = true;
}

LaminarModel::~LaminarModel()
{
    if(mem_test==true)
    {
    delete [] linearized_rate;
    delete [] linearized_jacobian;
    }
}

