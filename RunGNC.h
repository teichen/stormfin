// RunGNC.h
#ifndef _RUNGNC
#define _RUNGNC
#include "Sensors.h"

using namespace std;

class RunGNC
{
public:

    RunGNC();

    Sensors sensors; // utility functions, e.g. ref frame rotations
    void acquire_target(double*, double*, double, double, double*);
    void qrot_imu_data(double*, double*, double*, double*, double*, double*, double*);

    ~RunGNC();

private:
};

#endif
