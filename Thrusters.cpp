#include <stdlib.h>
#include "Thrusters.h"

using namespace std;

Thrusters::Thrusters()
{
    /* A2212 930KV brushless motors
       60mm blade propellers
       electronic speed controller (ESC), pulse width modulation (PWM) protocol with Arduino
       calibrate effective thrust (accounting for water drag)
    */
}

void Thrusters::thrust_to_pwm(double* u, double* pwm)
{
    /* provided a thrust, u [=] N, estimate a PWM (mapped) in microseconds
       u ~ c * PWM ** 2 where c is a coefficient of thrust
       pwm ~ 1500 microsec for warm up, 0 <= pwm <= 2000 microsec
    */
}

Thrusters::~Thrusters()
{
}

