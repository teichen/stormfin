#include "Utilities.h"
#include <stdlib.h>

using namespace std;

Utilities::Utilities()
{
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
