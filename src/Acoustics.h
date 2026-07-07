// Acoustics.h
#ifndef _ACOUSTICS
#define _ACOUSTICS

#include <iostream>

static double fc = 35.0; // carrier freq
static double f1 = 36.0; // mark freq
static double f0 = 34.0; // space freq
static double f_sampling = 96.0; // provides a 48kHz Nyquist frequency

using namespace std;

class Acoustics
{
public:

    Acoustics();

    void goertzel_dtft(double*, double*, int, double = f0, double = fc, double = f1, double = f_sampling);

    ~Acoustics();

private:
};

#endif
