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
    float signal_amplitude = 2.0f;   // амплитуда ускорения, м/с^2
    float frequency = 0.2f;   // частота сигнала, Гц

    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;   // тангаж, градусы
    float yaw_deg = 0.0f;

    double t = 0.0;   

    ImuMeasurement getMeasurement(double dt, const Mat& C_b_n_sins) {
        ImuMeasurement imu;

        t += dt;
        imu.timestamp = static_cast<int>(t * 1000.0);


        double a_signal = signal_amplitude * std::sin(2.0 * M_PI * frequency * t);

        Vec g_n(0.0, 0.0, SINS::G_ACCEL);

        //матрица перехода NED -> Body
        double r = roll_deg * SINS::DEG2RAD;
        double p = pitch_deg * SINS::DEG2RAD;
        double y = yaw_deg * SINS::DEG2RAD;

        Mat C_b_n = Mat::fromEuler(r, p, y);
        
        Mat C_n_b = T(C_b_n);

        Vec g_b = C_n_b * g_n;

        imu.accelerometer[0] = a_signal + g_b[0];
        imu.accelerometer[1] = g_b[1];
        imu.accelerometer[2] = g_b[2];


        double L = 55.0 * SINS::DEG2RAD;

        Vec omega_ie_n(
            SINS::OMEGA_EARTH * std::cos(L),
            0.0,
            -SINS::OMEGA_EARTH * std::sin(L)
        );
        Mat C_n_b_gyro = T(C_b_n_sins);
        Vec w_body(1.0, 0.0, 0.0);

        imu.gyroscope = w_body + C_n_b * omega_ie_n;;
        return imu;
    }
};