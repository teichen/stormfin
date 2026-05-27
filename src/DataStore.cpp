#include <stdlib.h>
#include "DataStore.h"
#include <string>

using namespace std;

static int n_states = 21;
static int n_measurements = 13;
static int n_thrusters = 3;

// all states and measurements in the NAV frame

DataStore::DataStore()
{
    /* write out data for specific events with the following format:
       measurements (sensor data)
       thrusters (thrust inputs)
       mean state estimate

       storing 37 doubles coming in at 100Hz and 64MB capacity we have approx. 43 min capacity
    */
    std::string csv_header[37] = {"MI_OMEGA_X", "MI_OMEGA_Y", "MI_OMEGA_Z", "MI_A_X", "MI_A_Y", "MI_A_Z", "MI_X",
                                  "MI_Y", "MI_V_X", "MI_V_Y", "MI_B_X", "MI_B_Y", "MI_B_Z",
                                  "UI_L", "UI_R", "UI_V", "SI_THETA_X", "SI_THETA_Y", "SI_THETA_Z",
                                  "SI_OMEGA_X", "SI_OMEGA_Y", "SI_OMEGA_Z", "SI_X", "SI_Y", "SI_Z", "SI_V_X",
                                  "SI_V_Y", "SI_V_Z", "SI_A_X", "SI_A_Y", "SI_A_Z", "SI_EPSILON_X", "SI_EPSILON_Y",
                                  "SI_EPSILON_Z", "SI_BETA_X", "SI_BETA_Y", "SI_BETA_Z"};

    buffer.reserve(1000);
}

void DataStore::event_store(double* z, double* u, double* x)
{
    buffer.emplace_back(z, u, x, n_measurements, n_thrusters, n_states);
}

DataStore::~DataStore()
{
}

