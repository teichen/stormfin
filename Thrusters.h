// Thrusters.h
#ifndef _THRUSTERS
#define _THRUSTERS

using namespace std;

class Thrusters
{
public:

    Thrusters();

    void thrust_to_pwm(double*, double*);

    ~Thrusters();

private:
};

#endif
