#include <stdlib.h>
#include "Thrusters.h"
#include <cmath>

using namespace std;

static int c = 100;

Thrusters::Thrusters()
{
    /* A2212 930KV brushless motors
       60mm blade propellers
       electronic speed controller (ESC), pulse width modulation (PWM) protocol with Arduino
       calibrate effective thrust (accounting for water drag)
    */
}

void Thrusters::thrust_to_pwm(double* u, int* pwm)
{
    /* provided a thrust, u [=] N, estimate a PWM (mapped) in microseconds
       ~15N capacity of 60mm prop with A2212 930 KV motor
       u ~ c * PWM ** 2 where c is a coefficient of thrust
       pwm ~ 1500 microsec for warm up, 0 <= pwm <= 2000 microsec

       u[0], pwm[0]: left thruster
       u[1], pwm[1]: right thruster
       u[2], pwm[2]: vertical thruster
    */
    int i;
    for (i=0; i<3; i++)
    {
        pwm[i] = static_cast<int>(std::floor(u[i] * c));
    }
}

Thrusters::~Thrusters()
{
}

