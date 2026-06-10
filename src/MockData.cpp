#include <stdlib.h>
#include "MockData.h"

static int STOP = 0;
static int DIVE = 1;
static int SURFACE = 2;
static int SURVEILLANCE = 3;
static int STALK = 4;
static int COMMUNICATE = 5;

using namespace std;

MockData::MockData()
{
    /*
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
}

void MockData::request_data(int nav_state)
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

    if (nav_state == SURFACE)
    {
        // GPS available
    }
    else
    {
        // GPS-denied conditions
    }
    // TODO: get GPS data
    latitude = 0.0;
    longitude = 0.0;
    speed = 0.0;
    gps_angle = 0.0;

    if (nav_state != COMMUNICATE) // forward facing narrow beam only when not communicating
    {
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
    }
    else
    {
        // acoustic modem communication halt forward facing narrow beam
    }
    d = 0.0; // placeholder, current distance    
    sensors.ultrasonic_distance(d);
}

MockData::~MockData()
{
}

