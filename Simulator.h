#pragma once
#define _USE_MATH_DEFINES
#include "VecMath.h"
#include "SINS.h"

class Simulator {
public:
    double signal_amplitude = 2.0;   // амплитуда ускорения, м/с^2
    double frequency = 0.2;   // частота сигнала, Гц
    double pitch_deg = 0.0;   // тангаж, градусы

    double t = 0.0;   

    Vec getAccel(double dt) {
        double a_signal = signal_amplitude * std::sin(2.0 * M_PI * frequency * t);

        Vec g_n(0.0, 0.0, SINS::G_ACCEL);

        //матрица перехода NED -> Body
        double theta = pitch_deg * SINS::DEG2RAD;
        Mat C_n_b;
        C_n_b[0][0] = std::cos(theta);  C_n_b[0][1] = 0; C_n_b[0][2] = -std::sin(theta);
        C_n_b[1][0] = 0;                C_n_b[1][1] = 1; C_n_b[1][2] = 0;
        C_n_b[2][0] = std::sin(theta);  C_n_b[2][1] = 0; C_n_b[2][2] = std::cos(theta);

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
        return C_n_b * omega_ie_n;
    }
};