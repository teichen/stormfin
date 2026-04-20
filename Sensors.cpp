#include <stdlib.h>
#include "Sensors.h"

using namespace std;

Sensors::Sensors()
{
    /* DFRobot IP68 6m UART ultrasonic sensor, ~30ft depth, 20ft distance capacity
       TTL Serial (UART) protocol with Arduino
       VIN-5V, SDA-SDA, SLC-SCL, GND-GND
       echoTime
       distance = echoTime * 0.0343 / 2.

       BNO055 strapdown IMU (MEMS)
       16bit gyro, 14bit accelerometer
       inter-integrated circuit (I2C) protocol with Arduino
       SDA serial data, SCL serial clock, 115200 baud
       quaternions, q, at 100 Hz
       angular velocity, omega, at 100 Hz
       acceleration, a [=] m / s**2, at 100 Hz
       gravity, g [=] m / s**2, without movement, at 100 Hz

       GLONASS + GPS PA1616D - 99 channel with 10 Hz update
       TTL Serial (UART) protocol with Arduino
       9600 baud
       VIN-5V, RX-7, TX-8, GND-GND
       fix, latitude, longitude, speed
    */
    mem_test = false;
    initarrays();

}

void Sensors::body_to_nav(double* q, double* r_body, double* r_nav)
{
    // body to inertial (NAV) frame rotation
    m[0] = 1.0 - 2.0 * (q[2] * q[2]) - 2.0 * (q[3] * q[3]);
    m[1] = 2.0 * (q[1] * q[2] - q[0] * q[3]);
    m[2] = 2.0 * (q[1] * q[3] + q[0] * q[2]);
    m[3] = 2.0 * (q[1] * q[2] + q[0] * q[3]);
    m[4] = 1.0 - 2.0 * (q[1] * q[1]) - 2.0 * (q[3] * q[3]);
    m[5] = 2.0 * (q[2] * q[3] - q[0] * q[1]);
    m[6] = 2.0 * (q[1] * q[3] - q[0] * q[2]);
    m[7] = 2.0 * (q[2] * q[3] + q[0] * q[1]);
    m[8] = 1.0 - 2.0 * (q[1] * q[1]) - 2.0 * (q[2] * q[2]);

    int i,j;
    for (i=0; i<3; i++)
    {
        r_nav[i] = 0.0;
        for (j=0; j<3; j++)
        {
            r_nav[i] += m[i*3 + j] * r_body[j];
        }
    }
}

void Sensors::initarrays()
{
    m    = (double*) calloc (9, sizeof(double));
    mem_test  = true;
}

Sensors::~Sensors()
{
    if(mem_test==true)
    {
    delete [] m;
    }
}

