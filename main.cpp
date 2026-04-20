#include <iostream>
#include "Controller.h"
#include "Sensors.h"
#include "Drivers.h"
#include <gperftools/profiler.h>

/*
TODO: automate main.cpp->arduino main.ino.cpp

e.g.
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//Set the delay between fresh samples
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
*/

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
// Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

using namespace std;

int main()
{
    ProfilerStart("/tmp/prof.out"); // memory profiler

    Sensors sensors; // utility functions, e.g. ref frame rotations
    Drivers drivers; // utility functions, e.g. effective thrust inputs
    Controller controller; // sensor fusion, estimation, navigation

    /* while operational,
       (1) process sensor data
       (2) determine operating state
           (a) ROV manual
           (b) circling surveilance
           (c) target acquisition - stalking
           (d) target acquisition - abandon to surface
       (2) run estimation and navigation routines
       (3) adjust thrust
    */

    /*
    sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);    
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    imu::Quaternion quat = bno.getQuat();
    */
    double q[4];
    q[0] = 0.0;
    q[1] = 0.0;
    q[2] = 0.0;
    q[3] = 0.0;

    q[0] = 0.0; // quat.w();
    q[1] = 0.0; // quat.x();
    q[2] = 0.0; // quat.y();
    q[3] = 0.0; // quat.z();

    double r_body[3];
    r_body[0] = 1.0;
    r_body[1] = 0.0;
    r_body[2] = 0.0;
    double r_nav[3];
    r_nav[0] = 0.0;
    r_nav[1] = 0.0;
    r_nav[2] = 0.0;
    sensors.body_to_nav(q, r_body, r_nav);

    ProfilerStop();

    return 0;
}
