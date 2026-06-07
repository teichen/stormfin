// Acoustics.h
#ifndef _ACOUSTICS
#define _ACOUSTICS

#include <iostream>

using namespace std;

class Acoustics
{
public:

    Acoustics();

    void goertzel_dtft(double*, double*, int);

    ~Acoustics();

private:
};

#endif
