#include "Acoustics.h"
#include <stdlib.h>
#include <cmath>

static double fc = 35.0; // carrier freq
static double f1 = 36.0; // mark freq
static double f0 = 34.0; // space freq
const double PI = 3.14159265358979323846;

using namespace std;

Acoustics::Acoustics()
{
    /* acoustic signal processing for telemetry and command
    */
}

void Acoustics::goertzel_dtft(double* x, double* y, int n)
{
    /* Goertzel DTFT for demodulation
       second order infinite impulse response filter
       Sysel and Rajmic 2012 (56); EURASIP Journal on Advances in Signal Processing
       3N * 3 operations for 3 frequencies (FFT would require Nlog2(N))

       double x[2 * N]: Re(0), Im(0), Re(1), Im(1)...
       filter out noise and isolate one of three frequencies:
       fc : carrier (central) 35kHz
       f1 : mark (1-bit) 36kHz
       f0 : space (0-bit) 34kHz
    */
    int i,j,k;
    for (i=0; i<(2*n); i++) // complex input
    {
        x[i] = x[i] / n; // scaled for dtft
    }
    // for frequency shift keying we can ignore the phase
    double bc, b1, b0;
    bc = 2.0 * std::cos(2.0 * PI * fc * 1000.0);
    b1 = 2.0 * std::cos(2.0 * PI * f1 * 1000.0);
    b0 = 2.0 * std::cos(2.0 * PI * f0 * 1000.0);

    // recursive solution for carrier, mark, and space
    // save complex multiplier for the final output at end
    // recursion defined in modified state space
    // dft: h_k = sum_{n=0}^{N-1}\, H_n * w^{-k(N-n)} * u(n) where u is step function
    // y_k(n=N) = h_k, w = exp(2*pi*i/N), B = 2*cos(2*pi*k/N)
    // original: y_k(n) - B * y_k(n-1) + y_k(n-2) = H_n - w^{k} * H_{n-1}
    // modified: output y_k(n) = s(n) - w^{k} * s(n-1), s(n) = H_n + B * s(n-1) - s(n-2)
    // indexing: 0 - f0 real
    //           1 - f0 imag
    //           2 - fc real
    //           3 - fc imag
    //           4 - f1 real
    //           5 - f1 imag
    double b[3], f[3];
    b[0] = b0; b[1] = bc; b[2] = b1;
    f[0] = f0; f[1] = fc; f[2] = f1;

    double s0[6], s1[6], s2[6];
    for (j=0; j<6; j++)
    {
        s0[j] = 0.0;
        s1[j] = 0.0;
        s2[j] = 0.0;
    }
    for (k=0; k<3; k++)
    {
        for (i=0; i<(n-1); i++)
        {
            for (j=0; j<2; j++)
            {
                s0[2*k + j] = x[2*i + j] + b[k] * s1[2*k + j] - s2[2*k + j];
                s2[2*k + j] = s1[2*k + j];
                s1[2*k + j] = s0[2*k + j];
            }
        }
        for (j=0; j<2; j++)
        {
            s0[2*k + j] = b[k] * s1[2*k + j] - s2[2*k + j];
            if (j==0)
            {
                y[2*k + j] = s0[2*k + j] - s1[2*k + j] * std::cos(2.0 * PI * f[k] * 1000.0);
            }
            else if (j==1)
            {
                y[2*k + j] = s0[2*k + j] - s1[2*k + j] * std::sin(2.0 * PI * f[k] * 1000.0);
            }
        }
    }
}

Acoustics::~Acoustics()
{
}
