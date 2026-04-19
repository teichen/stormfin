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

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

*/

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

    ProfilerStop();

    return 0;
}
