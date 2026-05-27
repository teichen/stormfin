// ComplementaryFilter.h
#ifndef _COMPLEMENTARYFILTER
#define _COMPLEMENTARYFILTER

#include "LaminarModel.h"
#include "Utilities.h"

#include <iostream>

using namespace std;

class ComplementaryFilter
{
public:

    bool mem_test;

    ComplementaryFilter();

    Utilities utilities;
    LaminarModel model;

    int n_s;
    int n_m;
    double* x_post;
    double* x_prior;

    void initialize_state();
    void initarrays();

    void process(double, double*, double*, double*);

    void update(double*);

    ~ComplementaryFilter();

private:
};

#endif
