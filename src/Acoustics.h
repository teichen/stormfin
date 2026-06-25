// Acoustics.h
#ifndef _ACOUSTICS
#define _ACOUSTICS

#include <iostream>

static double fc = 35.0; // carrier freq
static double f1 = 36.0; // mark freq
static double f0 = 34.0; // space freq

using namespace std;

class Acoustics
{
public:

    Acoustics();

    void goertzel_dtft(double*, double*, int, double = f0, double = fc, double = f1);

    ~Acoustics();

private:
};

#endif
