// MockData.h
#ifndef _MOCKDATA
#define _MOCKDATA
#include "Sensors.h"

using namespace std;

class MockData
{
public:

    MockData();

    Sensors sensors;

    void request_data(void);
    double d;
    double q[4];
    double omega_body[4];
    double mag_body[4];
    double a_body[4];
    double latitude, longitude, speed, gps_angle;

    ~MockData();

private:
};

#endif
