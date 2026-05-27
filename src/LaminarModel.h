// LaminarModel.h
#ifndef _LAMINARMODEL
#define _LAMINARMODEL

#include <iostream>

using namespace std;

class LaminarModel
{
public:

    LaminarModel();

    bool mem_test;
    int n_s, n_m, n_u;
    int si_theta_x, si_theta_y, si_theta_z;
    int si_omega_x, si_omega_y, si_omega_z;
    int si_x, si_y, si_z;
    int si_v_x, si_v_y, si_v_z;
    int si_a_x, si_a_y, si_a_z;
    int si_epsilon_x, si_epsilon_y, si_epsilon_z;
    int si_beta_x, si_beta_y, si_beta_z;
    int mi_omega_x, mi_omega_y, mi_omega_z;
    int mi_a_x, mi_a_y, mi_a_z;
    int mi_x, mi_y, mi_v_x, mi_v_y;
    int mi_b_x, mi_b_y, mi_b_z;
    int ui_l, ui_r, ui_v;

    static int rate(double, const double[], double[], void*);
    static int jacobian(double, const double[], double*, double[], void*);

    void linearized_rate(double*, double*);
    void linearized_jacobian(double*, double*);

    void init_meas_noise(double*);
    void init_state(double*);
    void init_covariance(double*);

    void estimate_measurements(double*, double*);
    void measurement_jacobian(double*, double*);
    void linearized_measurement_jacobian(double*, double*);

    void initarrays();

    ~LaminarModel();

private:
};

#endif
