#include <stdlib.h>
#include "Sensors.h"

using namespace std;

Sensors::Sensors()
{
    /* DFRobot IP68 6m UART ultrasonic sensor, ~30ft depth, 20ft distance capacity
       TTL Serial (UART) protocol with Arduino
       VIN-5V, D2 (white), D3 (yellow), GND-GND
       transducer is engineered for the density and sound speed of water
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

void Sensors::set_qrot(double* q)
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
}


void Sensors::qrot_pure(double* q, double* a)
{
    // pure quarternion rotation, a = q x a x q^-1
    double a_rot[4];

    // q = (q[0], qv) where qv = q[1:3]
    // a = (a[0], av) where av = a[1:3]
    // q x a = (q[0] * a[0] - qv dot av, q[0] * av + a[0] * qv + qv cross av)
    double qa[4];
    qa[0] = q[0] * a[0] - (q[1] * a[1] + q[2] * a[2] + q[3] * a[3]);
    qa[1] = q[0] * a[1] + a[0] * q[1] + (q[2] * a[3] - q[3] * a[2]);
    qa[2] = q[0] * a[2] + a[0] * q[2] + (q[3] * a[1] - q[1] * a[3]);
    qa[3] = q[0] * a[3] + a[0] * q[3] + (q[1] * a[2] - q[2] * a[1]);

    // q* = q^-1 = (q[0], -v)
    a_rot[0] = qa[0] * q[0] + (qa[1] * q[1] + qa[2] * q[2] + qa[3] * q[3]);
    a_rot[1] = -qa[0] * q[1] + q[0] * qa[1] - (qa[2] * q[3] - qa[3] * q[2]);
    a_rot[2] = -qa[0] * q[2] + q[0] * qa[2] - (qa[3] * q[1] - qa[1] * q[3]);
    a_rot[3] = -qa[0] * q[3] + q[0] * qa[3] - (qa[1] * q[2] - qa[2] * q[1]);

    int i;
    for (i=0; i<4; i++)
    {
        a[i] = a_rot[i];
    }
}

void Sensors::body_to_nav(double* q, double* r_body, double* r_nav)
{
    set_qrot(q);

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

void Sensors::ultrasonic_distance(double distance)
{
    /* scale the distance and use drift in orientation to predict center of mass
    */
    distance /= 10; // units of cm
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

