#include <iostream>
#include <cassert>
using std::cerr;
using std::cout;
using std::endl;
#include <math.h>
#include <cmath>
#include "../Controller.h"
#include "../Sensors.h"

using namespace std;

const double PI = 3.14159265358979323846;

void set_quarternion(double* q, double psi, double theta, double phi){
    /* quarternion, q, must be normalized
       psi   : yaw
       theta : pitch
       phi   : roll
    */ 
    q[0] = std::cos(0.5 * psi) * std::cos(0.5 * theta) * std::cos(0.5 * phi)
         + std::sin(0.5 * psi) * std::sin(0.5 * theta) * std::sin(0.5 * phi);
    q[1] = std::cos(0.5 * psi) * std::cos(0.5 * theta) * std::sin(0.5 * phi)
         - std::sin(0.5 * psi) * std::sin(0.5 * theta) * std::cos(0.5 * phi);
    q[2] = std::cos(0.5 * psi) * std::sin(0.5 * theta) * std::cos(0.5 * phi)
         + std::sin(0.5 * psi) * std::cos(0.5 * theta) * std::sin(0.5 * phi);
    q[3] = std::sin(0.5 * psi) * std::cos(0.5 * theta) * std::cos(0.5 * phi)
         - std::cos(0.5 * psi) * std::sin(0.5 * theta) * std::sin(0.5 * phi);
}

int main()
{
    // unit testing with run time asserts

    // TEST-0 : test the quarternion rotation matrix
    Sensors sensors;
    double psi   = 0.0; // yaw
    double theta = 0.0; // pitch
    double phi   = 0.0; // roll

    double q[4];
    set_quarternion(q, psi, theta, phi);

    double r_body[3];
    r_body[0] = 1.0;
    r_body[1] = 0.0;
    r_body[2] = 0.0;
    double r_nav[3];
    r_nav[0] = 0.0;
    r_nav[1] = 0.0;
    r_nav[2] = 0.0;
    sensors.body_to_nav(q, r_body, r_nav);

    int i;
    for (i=0; i<3; i++)
    {
        assert(r_nav[i] == r_body[i]);
    }

    psi = 0.5 * PI; // unit x -> unit y (rot about z)
    set_quarternion(q, psi, theta, phi);
    sensors.body_to_nav(q, r_body, r_nav);

    assert(std::abs(r_nav[0]) < 1.0e-10);
    assert(r_nav[1] == 1.0);
    assert(std::abs(r_nav[2]) < 1.0e-10);

    // controller tests:

    return 0;
}
