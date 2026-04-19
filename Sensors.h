// Sensors.h
#ifndef _SENSORS
#define _SENSORS

using namespace std;

class Sensors
{
public:

    bool mem_test;
    Sensors();

    double* m;

    void initarrays();

    void propagate_update(double, double*);
    void body_to_nav(double*, double*, double*);

    ~Sensors();

private:
};

#endif
