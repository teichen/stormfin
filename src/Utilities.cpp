#include "Utilities.h"
#include <stdlib.h>

using namespace std;

Utilities::Utilities()
{
}

void Utilities::matrix_mult(double* a, int n_a0, int n_a1, double* b, int n_b0, int n_b1, double* c, int n_c0, int n_c1)
{
    /* matrix multiplication, C = AB
    */
    int i,j,k;
    for (i=0; i<n_c0; i++) // n_a0 = n_c0
    {
        for (j=0; j<n_c1; j++) // n_b1 = n_c1
        {
            c[i*n_b1 + j] = 0.0;
            for (k=0; k<n_a1; k++)
            {
                c[i*n_b1 + j] += a[i*n_a1 + k] * b[k*n_b1 + j];
            }
        }
    }
}

void Utilities::ode_iv(LaminarModel& model, double* x0, double* x, int n_x, double dt, double* u)
{
    /* integrate an ode rate function, f
    */
    size_t size_x = n_x;

    double h = 0.01; // step size
    double x_err[n_x];

    double t0 = 0.0;
    double t = t0;

    double f[n_x];
    double dfdx[n_x * n_x];
    double dfdt[n_x];

    double k1[n_x];
    double k2[n_x];
    double k3[n_x];
    double k4[n_x];

    double x1[n_x];
    double x2[n_x];
    double x3[n_x];

    int i;
    int param = 0;
    int status;
    
    set_elements(x0, x, n_x, 1);
    while (t < dt)
    {
        // RK4 
        status = model.rate(t, x, f, u);
        status = model.jacobian(t, x, dfdx, dfdt, &param);
        for (i=0; i<n_x; i++)
        {
            k1[i] = f[i] * h;
            x1[i] = x[i] + 0.5 * k1[i];
        }
        status = model.rate((double)(t + 0.5 * h), x1, f, u);
        status = model.jacobian((double)(t + 0.5 * h), x1, dfdx, dfdt, &param);
        for (i=0; i<n_x; i++)
        {
            k2[i] = f[i] * h;
            x2[i] = x[i] + 0.5 * k2[i];
        }
        status = model.rate((double)(t + 0.5 * h), x2, f, u);
        status = model.jacobian((double)(t + 0.5 * h), x2, dfdx, dfdt, &param);
        for (i=0; i<n_x; i++)
        {
            k3[i] = f[i] * h;
            x3[i] = x[i] + k3[i];
        }
        status = model.rate((double)(t + h), x3, f, u);
        status = model.jacobian((double)(t + h), x3, dfdx, dfdt, &param);
        for (i=0; i<n_x; i++)
        {
            k4[i] = f[i] * h;
        }

        t += h;
        for (i=0; i<n_x; i++)
        {
            x[i] += (1.0 / 6.0) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
        }
    }
}

void Utilities::print_mat(double* a, int n_a0, int n_a1)
{
    std::string row;
    int i,j;
    for (i=0; i<n_a0; i++)
    {
        row = "";
        for (j=0; j<n_a1; j++)
        {
            row += std::to_string(a[i*n_a1 + j]);
            row += " ";
        }
        cout << row << endl;
    }
}

void Utilities::get_cols(double* a, int n_a0, int n_a1, int* idx, int n_idx, double* a_trunc)
{
    int i,j;
    for (i=0; i<n_a0; i++)
    {
        for (j=0; j<n_idx; j++)
        {
            a_trunc[i * n_idx + j] = a[i * n_a1 + idx[j]];
        }
    }
}

void Utilities::get_rows(double* a, int n_a0, int n_a1, int* idx, int n_idx, double* a_trunc)
{
    int i,j;
    for (i=0; i<n_idx; i++)
    {
        for (j=0; j<n_a1; j++)
        {
            a_trunc[i * n_a1 + j] = a[idx[i] * n_a1 + j];
        }
    }
}

void Utilities::get_rows_cols(double* a, int n_a0, int n_a1, int* idx, int n_idx, double* a_trunc)
{
    int i,j;
    for (i=0; i<n_idx; i++)
    {
        for (j=0; j<n_idx; j++)
        {
            a_trunc[i*n_idx + j] = a[idx[i] * n_a1 + idx[j]];
        }
    }
}

void Utilities::matrix_transpose(double* a, int n_a0, int n_a1, double* a_transpose)
{
    int i,j;
    for (i=0; i<n_a0; i++)
    {
        for (j=0; j<n_a1; j++)
        {
            a_transpose[j * n_a0 + i] = a[i * n_a1 + j];
        }
    }
}

void Utilities::set_elements(double* a, double* b, int n, int dim)
{
    /* a -> b
    */
    int i,j;
    if (dim == 1)
    {
        for (i=0; i<n; i++)
        {
            b[i] = a[i];
        }
    }
    else
    {
        for (i=0; i<n; i++)
        {
            for (j=0; j<n; j++)
            {
                b[i*n + j] = a[i*n + j];
            }
        }
    }
}

void Utilities::unity(int n, double* a)
{
    int i,j;
    for (i=0; i<n; i++)
    {
        for (j=0; j<n; j++)
        {
            if (i == j)
            {
                a[i*n + j] = 1.0;
            }
            else
            {
                a[i*n + j] = 0.0;
            }
        }
    }
}

Utilities::~Utilities()
{
}
