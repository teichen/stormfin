#include <stdlib.h>
#include "Collocation.h"

using namespace std;

Collocation::Collocation()
{
    /* direct integration for sensor fault tolerance
       high thrust limit : Rodrigues re-orientation, thrust
       low thrust limit : minimize square of controll effort
    */
}

void Collocation::optimal_thrust(double* r, double* t, int n_t, double* u)
{
    /* provided a separation between nav and target for a fixed depth,
       calculate the optimal thrust as a function of time, t

       r = separation vector between nav and target
       u = thrust
       dr/dt = v
       dv/dt ~ u
       boundary conditions:
       (a) r(t=0) = r0
       (b) dr/dt(t=0) = v(t=0) = 0
       (c) r(t=T) = 0
       (d) v(t=T) = 0

       functional minimization, variational calculus using
       multiplier functions for system dynamics, take cost
       function to be square of control effort
    */
    int i;
    for (i=0; i<n_t; i++)
    {
        u[i*2 + 0] = 6.0 * r[0] * (2.0 * t[i] - 1.0); // optimal x thrust at time t[i]
        u[i*2 + 1] = 6.0 * r[1] * (2.0 * t[i] - 1.0); // optimal y thrust at time t[i]
    }
}

Collocation::~Collocation()
{
}

