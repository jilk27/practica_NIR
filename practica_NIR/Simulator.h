#pragma once
#define _USE_MATH_DEFINES
#include "VecMath.h"
#include "SINS.h"

class Simulator {
public:
    float signal_amplitude = 2.0f;   // амплитуда ускорения, м/с^2
    float frequency = 0.2f;   // частота сигнала, Гц

    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;   // тангаж, градусы
    float yaw_deg = 0.0f;

    double t = 0.0;   

    Vec getAccel(double dt) {
        double a_signal = signal_amplitude * std::sin(2.0 * M_PI * frequency * t);

        Vec g_n(0.0, 0.0, SINS::G_ACCEL);

        //матрица перехода NED -> Body
        double r = roll_deg * SINS::DEG2RAD;
        double p = pitch_deg * SINS::DEG2RAD;
        double y = yaw_deg * SINS::DEG2RAD;

        Mat C_b_n = Mat::fromEuler(r, p, y);
        
        Mat C_n_b = T(C_b_n);

        Vec g_b = C_n_b * g_n;

        Vec acc_b;
        acc_b[0] = a_signal + g_b[0];
        acc_b[1] = g_b[1];
        acc_b[2] = g_b[2];

        t += dt;
        return acc_b;
    }

    //объект неподвижен, поэтому измеряем только вращение земли
    Vec getGyro(const Mat& C_b_n) {
        double L = 55.0 * SINS::DEG2RAD;

        Vec omega_ie_n(
            SINS::OMEGA_EARTH * std::cos(L),
            0.0,
            -SINS::OMEGA_EARTH * std::sin(L)
        );
        Mat C_n_b = T(C_b_n);
        Vec w_body(1.0, 0.0, 0.0);
        return w_body + C_n_b * omega_ie_n;
    }
};