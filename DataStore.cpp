#include <stdlib.h>
#include "DataStore.h"

using namespace std;

DataStore::DataStore()
{
    /* write out data for specific events with the following format:
       measurements (sensor data)
       thrusters (thrust inputs)
       mean state estimate

       storing 32 doubles coming in at 100Hz and 64MB capacity we have approx. 43 min capacity
    */

    buffer.reserve(1000);
}

void DataStore::event_store(double* z, double* u, double* x)
{
    buffer.emplace_back()
}

DataStore::~DataStore()
{
}

