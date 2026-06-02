#include "Acoustics.h"
#include <stdlib.h>

static double fc = 35.0; // carrier freq
static double f1 = 36.0; // mark freq
static double f0 = 34.0; // space freq
const double PI = 3.14159265358979323846;

using namespace std;

Acoustics::Acoustics()
{
}

void Acoustics::fft(double* x, double* y, int n)
{
    /* Goertzel DTFT
       second order infinite impulse response filter
       double x[2 * N]: Re(0), Im(0), Re(1), Im(1)...
       filter out noise and isolate one of three frequencies:
       fc : carrier (central) 35kHz
       f1 : mark (1-bit) 36kHz
       f0 : space (0-bit) 34kHz
    */
    int i;
    for (i=0; i<(2*n); i++)
    {
        y[i] = x[i] / n;
    }
    // for frequency shift keying we can ignore the phase
    double bc, b1, b0;
    bc = 2.0 * std::cos(2.0 * PI * fc * 1000.0);
    b1 = 2.0 * std::cos(2.0 * PI * f1 * 1000.0);
    c0 = 2.0 * std::cos(2.0 * PI * f0 * 1000.0);

    // TODO: recursive solution for carrier, mark, and space
}

Acoustics::~Acoustics()
{
}
