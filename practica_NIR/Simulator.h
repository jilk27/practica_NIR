#pragma once
#define _USE_MATH_DEFINES
#include "VecMath.h"
#include "SINS.h"

struct ImuMeasurement {
    int timestamp;   
    Vec accelerometer;
    Vec gyroscope;
};

class Simulator {
public:
    float a_x = 0.0f;
    float a_y = 0.0f;
    float a_z = 0.0f;

    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;  
    float yaw_deg = 0.0f;

    Mat C_b_n_true;

    float w_x = 0.0f;
    float w_y = 0.0f;
    float w_z = 0.0f;

    double t = 0.0;   

    ImuMeasurement getMeasurement(double dt) {
        ImuMeasurement imu;

        t += dt;
        imu.timestamp = static_cast<int>(t * 1000.0);


        Vec a_signal(a_x, a_y, a_z);

        Vec w_body(w_x, w_y, w_z); 

        Mat Omega = skew(w_body);

        Mat I_plus;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                I_plus[i][j] = (i == j ? 1.0 : 0.0) + Omega[i][j] * dt;

        C_b_n_true = C_b_n_true * I_plus;

        C_b_n_true.orthonormalize(); // (Метод Грама-Шмидта)
        
        Mat C_n_b = T(C_b_n_true);
        Vec g_n(0.0, 0.0, SINS::G_ACCEL);

        Vec g_b = C_n_b * g_n;


        imu.accelerometer = a_signal + g_b;


        double L = 55.0 * SINS::DEG2RAD;

        Vec omega_ie_n(
            SINS::OMEGA_EARTH * std::cos(L),
            0.0,
            -SINS::OMEGA_EARTH * std::sin(L)
        );

        imu.gyroscope = w_body + C_n_b * omega_ie_n;
        return imu;
    }
};