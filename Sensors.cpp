#include <iostream>
#include "Sensors.h"

using namespace std;

Sensors::Sensors()
{
    /* DFRobot IP68 6m UART ultrasonic sensor, ~30ft depth, 20ft distance capacity
       VIN-5V, SDA-SDA, SLC-SCL, GND-GND
       echoTime
       distance = echoTime * 0.0343 / 2.
       depth = height - (echoTime * 0.0343 / 2.)

       BNO055 strapdown IMU (MEMS)
       16bit gyro, 14bit accelerometer
       quaternions, q, at 100 Hz
       angular velocity, omega, at 100 Hz
       acceleration, a [=] m / s**2, at 100 Hz
       gravity, g [=] m / s**2, without movement, at 100 Hz

       GLONASS + GPS PA1616D - 99 channel with 10 Hz update

    */
}

// TODO: body to inertial (NAV) frame rotation

Sensors::~Sensors()
{
}

