// LaminarModel.h
#ifndef _LAMINARMODEL
#define _LAMINARMODEL

#include "Model.h"

#include <iostream>

using namespace std;

class LaminarModel : public Model
{
public:

    LaminarModel();

    bool mem_test;

    void rate(double*, double*);
    void map_inputs_states(double*, double*);

    void initarrays();

    ~LaminarModel();

private:
};

#endif
