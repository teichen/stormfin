#include "ComplementaryFilter.h"
#include <stdlib.h>
#include <cmath>

using namespace std;

static double alpha_att = 0.9; // stronger weight gyro data over accelerometer data
static double alpha_trans = 0.9; // stronger weight accelerometer data over GPS data

ComplementaryFilter::ComplementaryFilter()
{
    mem_test = false;

    n_s = model.n_s;
    n_m = model.n_m;

    initarrays();

    initialize_state();
}

void ComplementaryFilter::process(double dt, double* x, double* thrust, double* measurements)
{
    /* sensor fusion of gyro and accelerometer data for attitude (orientation)
       sensor fusion of accelerometer and GPS data for translation
       using a low pass and high pass filtering scheme
    */
    utilities.set_elements(x, x_prior, n_s, 1);

    // set prior with sensor data (assuming no noise)
    // TODO, if (!std::isnan(measurements[i]))

    // reduced model contains the attitude model to propagate for the low pass filter of gyrometer data
    // reduced model contains the translation model to propagate for the low pass filter of accelerometer data
    utilities.ode_iv(model, x, x_prior, n_s, dt, thrust);

    update(measurements);
}

void ComplementaryFilter::update(double* measurements)
{
    // TODO
}

void ComplementaryFilter::initialize_state()
{
    model.init_state(x_prior);

    int i, j;
    
    for (i=0; i<n_s; i++)
    {
        x_prior[i] = 0.0;
        x_post[i]  = 0.0;
    }
}

void ComplementaryFilter::initarrays()
{
    x_prior   = (double*) calloc (n_s, sizeof(double));
    x_post    = (double*) calloc (n_s, sizeof(double));

    mem_test  = true;
}

ComplementaryFilter::~ComplementaryFilter()
{
    if(mem_test==true)
    {
    delete [] x_prior;
    delete [] x_post;
    }
}
