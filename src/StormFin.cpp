#include <iostream>
#include <cmath>
#include <cstdlib>
#include "RunGNC.h"
#include "MockData.h"
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
static int COMMUNICATE = 5;

const int chipSelect = 10;
string data_string = "";

RunGNC run_gnc; // run methods
Sensors sensors; // utility functions, e.g. ref frame rotations
Thrusters thrusters; // utility functions, e.g. effective thrust inputs
Controller controller; // sensor fusion, estimation, navigation
DataStore datastore; // csv formatting
LaminarModel model;
Collocation coll;
MockData mock_data;

using namespace std::chrono_literals;
auto dt1s = 1s;
double dt = 0.0;
double dt_stop = dt;

int nav_state = DIVE;
auto t0 = std::chrono::system_clock::now();
auto t = t0;
auto t_stop = t0;

int i,j;

double r_nav[3];
double omega_nav[4];
double mag_nav[4];
double a_nav[4];

double d0 = 0.0; // previous distance [=] cm, placeholder
double d_stop = d0; // stop distance

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
        */
        mock_data.request_data(nav_state);

        run_gnc.qrot_imu_data(mock_data.q, mock_data.omega_body, mock_data.mag_body, mock_data.a_body,
                              omega_nav, mag_nav, a_nav);

        z[model.mi_omega_x] = omega_nav[1];
        z[model.mi_omega_y] = omega_nav[2];
        z[model.mi_omega_z] = omega_nav[3];
        z[model.mi_a_x] = a_nav[1];
        z[model.mi_a_y] = a_nav[2];
        z[model.mi_a_z] = a_nav[3];
        z[model.mi_x] = mock_data.longitude;
        z[model.mi_y] = mock_data.latitude;
        z[model.mi_v_x] = mock_data.speed * std::cos(mock_data.gps_angle);
        z[model.mi_v_y] = mock_data.speed * std::sin(mock_data.gps_angle);
        z[model.mi_b_x] = mag_nav[1];
        z[model.mi_b_y] = mag_nav[2];
        z[model.mi_b_z] = mag_nav[3];

        // use extended Kalman filter to get best estimate for d
        // handle IMU, GPS, ultrasonic data as measurements with nonzero uncertainty
        auto t = std::chrono::system_clock::now();
        dt = (t - t0) / dt1s;

        controller.process(dt, x, s2, u, z);

        // TODO: nav_state = COMMUNICATE temporarily during events such as collision, target loss

        if (mock_data.d < 30){
            // abandon
            nav_state = SURFACE;
        }
        else if (mock_data.d > 500){ // 600cm (20ft) operating limit, lost the target
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
                    run_gnc.acquire_target(mock_data.q, u_saved, dt_stop, d_stop, r_nav);

                    // calculate thrust profile provided vector to target, r_nav
                    // t_set = (t_0, t_1, t_2, ..., t_nt)
                    // u_set = (u_0, u_1, u_2, ..., u_nt) where u_0 = u[t_0] setting
                    coll.optimal_thrust(r_nav, t_set, n_t, u_set);
                }
                nav_state = STALK; // close the distance
            }
        }
        else {
            if (std::abs(mock_data.d - d0) < 5){ // static target
                nav_state = STOP; // observe
                d_stop = mock_data.d;
                t_stop = t;
            }
            else if (mock_data.d > d0){ // assume carry forward on current trajectory
                nav_state = STALK;
            }
            else { // target approaching, update distance
                nav_state = STOP;
                d_stop = mock_data.d;
                t_stop = t;
            }
        }

        thrusters.thrust_state(nav_state, u_set, idx_set, u);
        if (nav_state == STOP){
            // save off last thrust configuration
            u_saved[0] = u[0];
            u_saved[1] = u[1];
            u_saved[2] = u[2];
        }

        thrusters.thrust_to_pwm(u, pwm);
        // ESC.writeMicroseconds(pwm);
        // delay(100);

        d0 = mock_data.d;
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
