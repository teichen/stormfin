#include "RungeKutta.h"

using namespace std;

RungeKutta::RungeKutta()
{
    mem_test = false;
}

void RungeKutta::init_model(Model& model)
{
    /* Args:
                model (Model) : model class
    */
    n = model.n_states;

    initarrays();

    int i,j;

    for (i=0; i<n; i++)
    {
        for (j=0; j<n; j++)
        {
            rate[i*n + j] = model.linearized_rate[i*n + j];
        }
    }
}

void RungeKutta::displacement(double* x, double dt, double* k)
{
    int i,j;

    for (i=0; i<n; i++)
    {
        for (j=0; j<n; j++)
        {
            k[i] += rate[i*n + j] * x[j] * dt;
        }
    }
}

void RungeKutta::propagate(double t0, double tf, double dt, double* x_prior)
{
    /* classic Runge-Kutta method (RK4)
       dy/dt = f(t,y) ; y(t0) = x_prior
       further assume autonomous, f(t,y) = f(y)
   
       Args:
                t0 (double)     : initial time
                tf (double)     : final time
                dt (double)     : time interval step size
                x_prior (array) : mean prior estimate
    */
    double t;

    double k1[n], k2[n], k3[n], k4[n];

    double x_transient[n];

    int i;

    t = t0;
    while (t < tf)
    {
        t += dt;

        displacement(x_prior, dt, k1);
        for (i=0; i<n; i++)
        {
            x_transient[i] = x_prior[i] + 0.5 * dt * k1[i];
        }
        displacement(x_transient, dt, k2);
        for (i=0; i<n; i++)
        {
            x_transient[i] = x_prior[i] + 0.5 * dt * k2[i];
        }
        displacement(x_transient, dt, k3);
        for (i=0; i<n; i++)
        {
            x_transient[i] = x_prior[i] + dt * k3[i];
        }
        displacement(x_transient, dt, k4);

        for (i=0; i<n; i++)
        {
            x_prior[i] += (1.0 / 6.0) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
        }
        t0 = t;
    }
}

void RungeKutta::initarrays()
{
    rate = (double*) calloc (n*n, sizeof(double));

    mem_test = true;
}

RungeKutta::~RungeKutta()
{
    if(mem_test==true)
    {
    delete [] rate;
    }
}
