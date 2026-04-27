#include <iostream>
#include <cmath>
#include <cstdlib>
#include "Controller.h"
#include "Sensors.h"
#include "Thrusters.h"
#include <gperftools/profiler.h>
#include <chrono>

#define COM 0x55

/*
TODO: automate main.cpp->arduino main.ino.cpp

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

//Set the delay between fresh samples
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

SoftwareSerial ultrasonicSerial(2, 3); // IP68 UART ultrasonic
unsigned char ultrasonic_data[4];

Servo ESC;
*/
static int STOP = 0;
static int DIVE = 1;
static int SURFACE = 2;
static int SURVEILLANCE = 3;
static int STALK = 4;

static int n_states = 21; // TODO: DRY
static int n_measurements = 8;
static int n_thrusters = 3;

static int MI_OMEGA_X = 0; // IMU gyro
static int MI_OMEGA_Y = 1;
static int MI_OMEGA_Z = 2;
static int MI_A_X = 3; // IMU accel
static int MI_A_Y = 4;
static int MI_A_Z = 5;
static int MI_X = 6; // GPS
static int MI_Y = 7;

using namespace std;

int main()
{
    using namespace std::chrono_literals;
    auto dt1s = 1s;
    ProfilerStart("/tmp/prof.out"); // memory profiler

    Sensors sensors; // utility functions, e.g. ref frame rotations
    Thrusters thrusters; // utility functions, e.g. effective thrust inputs
    Controller controller; // sensor fusion, estimation, navigation

    int nav_state = DIVE;
    auto t0 = std::chrono::system_clock::now();

    double x[n_states];
    double s2[n_states * n_states];
    double z[n_measurements];

    int i,j;
    for (i=0; i<n_states; i++)
    {
        x[i] = 0.0; // TODO: FIX: pull in from model
        for (j=0; j<n_states; j++)
        {
            s2[i*n_states + j] = 0.0;
        }
    }
    /*
    // arduino setup()
    Serial.begin(115200); // BNO055

    sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
    uint8_t system, gyro, accel, mag = 0;

    ESC.attach(9, 1000, 2000); // ESC connected to PIN 9
    */

    double q[4];
    double r_body[3];
    double r_nav[3];

    double omega_body[4];
    double omega_nav[4];
    double a_body[4];
    double a_nav[4];

    double latitude, longitude;

    double d0 = 0.0; // previous distance [=] cm, placeholder
    double d = 0.0; // current distance    

    int pwm[3];
    double u[3]; // thrust: L forward/reverse, R forward/reverse, Vertical (dive/surface)
    pwm[0] = 0.0;
    pwm[1] = 0.0;
    pwm[2] = 0.0;
    u[0] = 0.0;
    u[1] = 0.0;
    u[2] = 0.0;

    // arduino loop()
    while (true) {
        /*
           (1) process sensor data
           (2) determine operating state
               (a) ROV manual
               (b) circling surveilance
               (c) target acquisition - stalking
               (d) target acquisition - abandon to surface
           (2) run estimation and navigation routines
           (3) adjust thrust

        // first, the BNO055
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
        bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
        bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
        bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
        bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

        bno.getCalibration(&system, &gyro, &accel, &mag);

        imu::Quaternion quat = bno.getQuat();
        imu::Vector<3> gyro_data = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        imu::Vector<3> accel_data = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
        */
        // TODO: swap in quat data
        q[0] = 0.0; // quat.w();
        q[1] = 0.0; // quat.x();
        q[2] = 0.0; // quat.y();
        q[3] = 0.0; // quat.z();
        r_body[0] = 1.0;
        r_body[1] = 0.0;
        r_body[2] = 0.0;
        r_nav[0] = 0.0;
        r_nav[1] = 0.0;
        r_nav[2] = 0.0;
        sensors.body_to_nav(q, r_body, r_nav);

        /* body frame sufficient for stabilization, navigation frame needed for
           fusion with GPS and fault tolerance
        */
        // TODO: swap in gyro_data, [=] radians / s
        omega_body[0] = 0.0;
        omega_body[1] = 1.0; // gyro_data[0];
        omega_body[2] = 0.0; // gyro_data[1];
        omega_body[3] = 0.0; // gyro_data[2];
        for (i=0; i<4; i++)
        {
            omega_nav[i] = omega_body[i];
        }
        sensors.qrot_pure(q, omega_nav);

        // tether to GPS buoy should minimize tilt, use linear acceleration (no gravity)
        // to avoid interpreting tilt as xy movement
        // TODO: swap in accel_data, [=] m / s**2
        a_body[0] = 0.0;
        a_body[1] = 0.0; // accel_data[0]
        a_body[2] = 0.5; // accel_data[1]
        a_body[3] = 0.0; // accel_data[2]
        for (i=0; i<4; i++)
        {
            a_nav[i] = a_body[i];
        }
        sensors.qrot_pure(q, a_nav);

        // TODO: get GPS data
        latitude = 0.0;
        longitude = 0.0;

        z[MI_OMEGA_X] = omega_nav[1];
        z[MI_OMEGA_Y] = omega_nav[2];
        z[MI_OMEGA_Z] = omega_nav[3];
        z[MI_A_X] = a_nav[1];
        z[MI_A_Y] = a_nav[2];
        z[MI_A_Z] = a_nav[3];
        z[MI_X] = longitude;
        z[MI_Y] = latitude;

        /*
        // next, the IP68 UART ultrasonic sensor
        ultrasonicSerial.begin(115200); // IP68 UART ultrasonic sensor

        // first, for the IP68 UART ultrasonic sensor
        ultrasonicSerial.write(COM);
        if (ultrasonicSerial.available() > 0) {
            delay(4);
            // Look for the start byte (typically 0xFF)
            if (ultrasonicSerial.read() == 0xFF) {
                ultrasonic_data[0] = 0xFF;
                for (int i = 1; i < 4; i++) {
                    ultrasonic_data[i] = ultrasonicSerial.read();
                }
                // Checksum validation: (Data[0] + Data[1] + Data[2]) & 0x00FF
                if (((ultrasonic_data[0] + ultrasonic_data[1] + ultrasonic_data[2]) & 0xFF) == ultrasonic_data[3]) {
                    d = (ultrasonic_data[1] << 8) + ultrasonic_data[2]; // [=]mm, Combine High and Low bytes
                }
            }
        }
        */
        sensors.ultrasonic_distance(d);

        // use extended Kalman filter to get best estimate for d
        // handle IMU, GPS, ultrasonic data as measurements with nonzero uncertainty
        auto t = std::chrono::system_clock::now();
        std::chrono::duration<double> dt = t - t0;

        // TODO: fill z with sensor data
        controller.process(dt/dt1s, x, s2, u, z);

        if (d > 500){ // 600cm (20ft) operating limit
            // resume circling surveillance
            nav_state = SURVEILLANCE;
        }
        else {
            if (d < 30){
                // abandon
                nav_state = SURFACE;
            }
            else if (d > 200){
                nav_state = STALK; // close the distance
            }
            else {
                if (std::abs(d - d0) < 5){
                    nav_state = STOP; // observe
                }
                else if (d > d0){
                    nav_state = STALK;
                }
                else {
                    nav_state = STOP;
                }
            }
        }

        if (nav_state == DIVE){
            u[0] = 0.0;
            u[1] = 0.0;
            u[2] = -1.0; // well below 15N capacity
        }
        else if (nav_state == SURFACE) {
            u[0] = 0.0;
            u[1] = 0.0;
            u[2] = 1.0;
        }
        else if (nav_state == SURVEILLANCE) {
            u[0] = 0.1;
            u[1] = 2.0 * u[0]; // circle
            u[2] = 0.0;
        }
        else if (nav_state == STOP) {
            u[0] = 0.0;
            u[1] = 0.0;
            u[2] = 0.0;
        }
        else if (nav_state == STALK) {
            u[0] = (d - d0) / (dt/dt1s) / 60 * 0.2; // account for drag, 1N ~ 60cm/s
            u[1] = u[0];
            u[2] = 0.0;
        }

        thrusters.thrust_to_pwm(u, pwm);
        // ESC.writeMicroseconds(pwm);
        // delay(100);

        d0 = d;
        t0 = t;
        break; // for testing
    }

    ProfilerStop();

    return 0;
}
