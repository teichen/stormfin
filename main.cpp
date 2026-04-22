#include <iostream>
#include "Controller.h"
#include "Sensors.h"
#include "Thrusters.h"
#include <gperftools/profiler.h>

#define COM 0x55

/*
TODO: automate main.cpp->arduino main.ino.cpp

e.g.
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

using namespace std;

int main()
{
    ProfilerStart("/tmp/prof.out"); // memory profiler

    Sensors sensors; // utility functions, e.g. ref frame rotations
    Thrusters thrusters; // utility functions, e.g. effective thrust inputs
    Controller controller; // sensor fusion, estimation, navigation

    // arduino setup()
    // Serial.begin(115200); // BNO055
    
    /*
    // first, the BNO055
    sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

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
    imu::Vector<3> gyro_data = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    */
    double q[4];

    // TODO: swap in quat data
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

    /* body frame sufficient for stabilization, navigation frame needed for
       fusion with GPS and fault tolerance
    */
    double omega_body[4];
    // TODO: swap in gyro_data
    omega_body[0] = 0.0;
    omega_body[1] = 1.0; // gyro_data[0];
    omega_body[2] = 0.0; // gyro_data[1];
    omega_body[3] = 0.0; // gyro_data[2];
    double omega_nav[4];
    int i;
    for (i=0; i<4; i++)
    {
        omega_nav[i] = omega_body[i];
    }
    sensors.qrot_pure(q, omega_nav);

    /*
    // next, the IP68 UART ultrasonic sensor
    ultrasonicSerial.begin(115200); // IP68 UART ultrasonic sensor

    // arduino loop()
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
                distance = (ultrasonic_data[1] << 8) + ultrasonic_data[2]; // Combine High and Low bytes
            }
        }
    }
    */
    double distance = 1.0; // [=] mm, placeholder
    sensors.ultrasonic_distance(distance);

    /* now for the motor
    int Speed;
    ESC.attach(9, 1000, 2000); // ESC connected to PIN 9
    */

    int pwm = 0;
    double u[2];
    u[0] = 1.0; // placeholder
    u[1] = 0.0; // placeholder
    thrusters.thrust_to_pwm(u, pwm);
    // ESC.writeMicroseconds(pwm);
    // delay(100);

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

    ProfilerStop();

    return 0;
}
