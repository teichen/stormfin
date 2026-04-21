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

    void body_to_nav(double*, double*, double*);
    void set_qrot(double*);
    void qrot_pure(double*, double*);

    ~Sensors();

private:
};

#endif
