#include <iostream>
#include <cassert>
using std::cerr;
using std::cout;
using std::endl;
#include <math.h>
#include <cmath>
#include "../Utilities.h"
#include "../GSLWrappers.h"
#include "../LaminarModel.h"

using namespace std;

const double PI = 3.14159265358979323846;

int main()
{
    Utilities utilities;
    GSLWrappers gsl;

    // TEST-0 : simple matrix transpose
    double a[4];
    a[0] = 1.0;
    a[1] = 2.0;
    a[2] = 3.0;
    a[3] = 4.0;
    double a_trans[4];
    utilities.matrix_transpose(a, 2, 2, a_trans);
    assert(a_trans[0] == 1.0);
    assert(a_trans[1] == 3.0);
    assert(a_trans[2] == 2.0);
    assert(a_trans[3] == 4.0);

    // TEST-1 : simple matrix inversion
    double a_inv[4];
    gsl.matrix_inv(a, 2, 2, a_inv);
    double a_det = a[0] * a[3] - a[1] * a[2];
    
    assert(std::abs(a_inv[0] - (double)(a[3] / a_det)) < 1.0e-10);
    assert(std::abs(a_inv[1] - (double)(-a[1] / a_det)) < 1.0e-10);
    assert(std::abs(a_inv[2] - (double)(-a[2] / a_det)) < 1.0e-10);
    assert(std::abs(a_inv[3] - (double)(a[0] / a_det)) < 1.0e-10);

    // TEST-2 : simple matrix exponential
    double b[9];
    b[0] = 0.0;
    b[1] = 1.0;
    b[2] = 2.0;
    b[3] = 0.0;
    b[4] = 0.0;
    b[5] = 3.0;
    b[6] = 0.0;
    b[7] = 0.0;
    b[8] = 0.0;
    double b_exp[9];
    gsl.matrix_exponential(b, 3, b_exp);

    assert(b_exp[0] == 1.0);
    assert(std::abs(b_exp[1] - b[1]) < 1.0e-10);
    assert(std::abs(b_exp[2] - (double)(b[2] + 0.5 * b[1] * b[5])) < 1.0e-10);
    assert(b_exp[3] == 0.0);
    assert(b_exp[4] == 1.0);
    assert(std::abs(b_exp[5] - b[5]) < 1.0e-10);
    assert(b_exp[6] == 0.0);
    assert(b_exp[7] == 0.0);
    assert(b_exp[8] == 1.0);

    // TEST-3 : simple matrix multiply
    double c[4];
    c[0] = 1.0;
    c[1] = 2.0;
    c[2] = 2.0;
    c[3] = 3.0;
    double d[4];
    gsl.matrix_mult(a, 2, 2, c, 2, 2, d, 2 , 2);

    assert(std::abs(d[0] - (double)(a[0] * c[0] + a[1] * c[2])) < 1.0e-10);
    assert(std::abs(d[1] - (double)(a[0] * c[1] + a[1] * c[3])) < 1.0e-10);
    assert(std::abs(d[2] - (double)(a[2] * c[0] + a[3] * c[2])) < 1.0e-10);
    assert(std::abs(d[3] - (double)(a[2] * c[1] + a[3] * c[3])) < 1.0e-10);

    // TEST-4 : ode_iv integrate model state dynamics
    LaminarModel model;

    double x0[model.n_s];
    double x[model.n_s];
    double u[model.n_u];
    int i;
    for (i=0; i<model.n_u; i++)
    {
        u[i] = 0.0;
    }    

    model.init_state(x0);
    x0[model.si_omega_x] = PI; // [=] rad / s
    utilities.set_elements(x0, x, model.n_s, 1);
    double dt = 1.0; // [=] s

    for (i=0; i<model.n_s; i++)
    {
        if (i == model.si_omega_x)
        {
            assert(x[i] == PI); // angular velocity
        }
        else
        {
            assert(x[i] == 0.0);
        }
    }

    gsl.ode_iv(model, x0, x, model.n_s, dt, u);

    for (i=0; i<model.n_s; i++)
    {
        if ((i == model.si_theta_x) or (i == model.si_omega_x))
        {
            assert(std::abs(x[i] - PI) < 0.1); // angular displacement integrated over 1s
        }
        else
        {
            assert(std::abs(x[i]) < 1.0e-5);
        }
    }

    return 0;
}
