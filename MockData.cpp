#include <stdlib.h>
#include "MockData.h"

using namespace std;

MockData::MockData()
{
}

void MockData::request_data(void)
{
    // TODO: swap in quat data
    q[0] = 0.0; // quat.w();
    q[1] = 0.0; // quat.x();
    q[2] = 0.0; // quat.y();
    q[3] = 0.0; // quat.z();

    // TODO: swap in gyro_data, [=] radians / s
    omega_body[0] = 0.0;
    omega_body[1] = 1.0; // gyro_data[0];
    omega_body[2] = 0.0; // gyro_data[1];
    omega_body[3] = 0.0; // gyro_data[2];

    mag_body[0] = 0.0;
    mag_body[1] = 0.1; // mag_data[0]
    mag_body[2] = 0.1; // mag_data[1]
    mag_body[3] = 0.1; // mag_data[2]

    // tether to GPS buoy should minimize tilt, use linear acceleration (no gravity)
    // to avoid interpreting tilt as xy movement
    // TODO: swap in accel_data, [=] m / s**2
    a_body[0] = 0.0;
    a_body[1] = 0.0; // accel_data[0]
    a_body[2] = 0.5; // accel_data[1]
    a_body[3] = 0.0; // accel_data[2]

    // TODO: get GPS data
    latitude = 0.0;
    longitude = 0.0;
    speed = 0.0;
    gps_angle = 0.0;
}

MockData::~MockData()
{
}

