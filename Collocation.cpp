#include <stdlib.h>
#include "Collocation.h"

using namespace std;

Collocation::Collocation()
{
    /* direct integration for sensor fault tolerance
       high thrust limit : Rodrigues re-orientation, thrust
       low thrust limit : minimize square of controll effort

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
}

void Collocation::optimal_thrust(double* r, double* u, double* t)
{
    /* provided a separation between nav and target for a fixed depth,
       calculate the optimal thrust as a function of time, t
    */
}

Collocation::~Collocation()
{
}

