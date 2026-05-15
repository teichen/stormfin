#include <iostream>
#include <cmath>
#include <cstdlib>
#include "RunGNC.h"
#include "Controller.h"
#include "Collocation.h"
#include "LaminarModel.h"
#include "Sensors.h"
#include "Thrusters.h"
#include "DataStore.h"
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
#include <SPI.h>
#include <SD.h>

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

const int chipSelect = 10;
string data_string = "";

RunGNC run_gnc; // run methods
Sensors sensors; // utility functions, e.g. ref frame rotations
Thrusters thrusters; // utility functions, e.g. effective thrust inputs
Controller controller; // sensor fusion, estimation, navigation
DataStore datastore; // csv formatting
LaminarModel model;
Collocation coll;

using namespace std::chrono_literals;
auto dt1s = 1s;
double dt = 0.0;
double dt_stop = dt;

int nav_state = DIVE;
auto t0 = std::chrono::system_clock::now();
auto t = t0;
auto t_stop = t0;

int i,j;

double q[4];
double r_nav[3];

double omega_body[4];
double omega_nav[4];
double mag_body[4];
double mag_nav[4];
double a_body[4];
double a_nav[4];

double latitude, longitude, speed, gps_angle;

double d0 = 0.0; // previous distance [=] cm, placeholder
double d = d0; // current distance    
double d_stop = d;

int pwm[3];
double u[3]; // thrust: L forward/reverse, R forward/reverse, Vertical (dive/surface)
double u_saved[3];
double dt_closest = 3600.0;

using namespace std;

int main()
{
    ProfilerStart("/tmp/prof.out"); // memory profiler

    double x[model.n_s];
    double s2[model.n_s * model.n_s];
    double z[model.n_m];

    // thrust setpoints
    int n_t = 100;
    double t_set[n_t];
    double u_set[n_t * 3];
    for (i=0; i<n_t; i++)
    {
        t_set[i] = 0.0;
        u_set[i*3 + 0] = 0.0;
        u_set[i*3 + 1] = 0.0;
        u_set[i*3 + 2] = 0.0;
    }

    int idx_set;
   
    model.init_state(x);
    model.init_covariance(s2);
    /*
    // arduino setup()
    Serial.begin(115200); // BNO055

    sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
    uint8_t system, gyro, accel, mag = 0;

    ESC.attach(9, 1000, 2000); // ESC connected to PIN 9
    */

    pwm[0] = 0.0;
    pwm[1] = 0.0;
    pwm[2] = 0.0;
    u[0] = 0.0;
    u[1] = 0.0;
    u[2] = 0.0;
    u_saved[0] = u[0];
    u_saved[1] = u[1];
    u_saved[2] = u[2];

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
        imu::Vector<3> mag_data = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
        */

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

        /* body frame sufficient for stabilization, navigation frame needed for
           fusion with GPS and fault tolerance
        */
        run_gnc.qrot_imu_data(q, omega_body, mag_body, a_body, omega_nav, mag_nav, a_nav);

        // TODO: get GPS data
        latitude = 0.0;
        longitude = 0.0;
        speed = 0.0;
        gps_angle = 0.0;

        z[model.mi_omega_x] = omega_nav[1];
        z[model.mi_omega_y] = omega_nav[2];
        z[model.mi_omega_z] = omega_nav[3];
        z[model.mi_a_x] = a_nav[1];
        z[model.mi_a_y] = a_nav[2];
        z[model.mi_a_z] = a_nav[3];
        z[model.mi_x] = longitude;
        z[model.mi_y] = latitude;
        z[model.mi_v_x] = speed * std::cos(gps_angle);
        z[model.mi_v_y] = speed * std::sin(gps_angle);
        z[model.mi_b_x] = mag_nav[1];
        z[model.mi_b_y] = mag_nav[2];
        z[model.mi_b_z] = mag_nav[3];

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
        dt = (t - t0) / dt1s;

        controller.process(dt, x, s2, u, z);

        if (d < 30){
            // abandon
            nav_state = SURFACE;
        }
        else if (d > 500){ // 600cm (20ft) operating limit, lost the target
            // do we have target thrust from our collocation?
            idx_set = n_t;
            dt_closest = 3600.0;
            for (i=0; i<n_t; i++)
            {
                if (std::abs(t_set[i] - dt_stop) < dt_closest)
                {
                    dt_closest = std::abs(t_set[i] - dt_stop);
                    idx_set = i;
                }
            }
            if (idx_set >= n_t) // if thrust profile burned, resume circling surveillance
            {
                nav_state = SURVEILLANCE;
            }
            else
            {
                if (nav_state == STOP)
                {
                    // previously stopped, damped inertia, target acquisition
                    dt_stop = (t - t_stop) / dt1s;
                    run_gnc.acquire_target(q, u_saved, dt_stop, d_stop, r_nav);

                    // calculate thrust profile provided vector to target, r_nav
                    // t_set = (t_0, t_1, t_2, ..., t_nt)
                    // u_set = (u_0, u_1, u_2, ..., u_nt) where u_0 = u[t_0] setting
                    coll.optimal_thrust(r_nav, t_set, n_t, u_set);
                }
                nav_state = STALK; // close the distance
            }
        }
        else {
            if (std::abs(d - d0) < 5){ // static target
                nav_state = STOP; // observe
                d_stop = d;
                t_stop = t;
            }
            else if (d > d0){ // assume carry forward on current trajectory
                nav_state = STALK;
            }
            else { // target approaching, update distance
                nav_state = STOP;
                d_stop = d;
                t_stop = t;
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
            // save off last thrust configuration
            u_saved[0] = u[0];
            u_saved[1] = u[1];
            u_saved[2] = u[2];
            u[0] = 0.0;
            u[1] = 0.0;
            u[2] = 0.0;
        }
        else if (nav_state == STALK) {
            u[0] = u_set[idx_set * 3 + 0];
            u[1] = u_set[idx_set * 3 + 1];
            u[2] = u_set[idx_set * 3 + 2];
        }

        thrusters.thrust_to_pwm(u, pwm);
        // ESC.writeMicroseconds(pwm);
        // delay(100);

        d0 = d;
        t0 = t;

        /* TODO: data logging
        File dataFile = SD.open("datalog.txt", FILE_WRITE);

        // if the file is available, write to it:
        if (dataFile) {
            dataFile.println(dataString);
            dataFile.close();
            // print to the serial port too:
            Serial.println(dataString);
        }
        // if the file isn't open, pop up an error:
        else {
            Serial.println("error opening datalog.txt");
        }
        */

        break; // for testing
    }

    ProfilerStop();

    return 0;
}
