#pragma once
#define _USE_MATH_DEFINES
#include "VecMath.h"
#include "SINS.h"
#include <random>

struct ImuMeasurement {
    int timestamp;   
    Vec accelerometer;
    Vec gyroscope;
};

struct ImuError {
    Vec bias;
    Mat H;
    double noise;

    ImuError() : bias(0, 0, 0), noise(0.0) {}
};

class Simulator {
public:
    float a_x = 0.0f;
    float a_y = 0.0f;
    float a_z = 0.0f;

    Mat C_b_n_true;

    float w_x = 0.0f;
    float w_y = 0.0f;
    float w_z = 0.0f;

    double t = 0.0;

    ImuError accel_errors;
    ImuError gyro_errors;

    std::mt19937 gen;
    std::normal_distribution<double> dist;

    Simulator() : gen(std::random_device{}()), dist(0.0, 1.0) {}

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

        Vec true_acc = a_signal + g_b;


        double L = 55.0 * SINS::DEG2RAD;

        Vec omega_ie_n(
            SINS::OMEGA_EARTH * std::cos(L),
            0.0,
            -SINS::OMEGA_EARTH * std::sin(L)
        );

        Vec true_gyro = w_body + C_n_b * omega_ie_n;

        

        Vec meas_acc = accel_errors.bias + accel_errors.H * true_acc;
        Vec meas_gyro = gyro_errors.bias + gyro_errors.H * true_gyro;

        double sigma_acc = std::sqrt(accel_errors.noise);
        double sigma_gyro = std::sqrt(gyro_errors.noise);

        meas_acc[0] += dist(gen) * sigma_acc;
        meas_acc[1] += dist(gen) * sigma_acc;
        meas_acc[2] += dist(gen) * sigma_acc;

        meas_gyro[0] += dist(gen) * sigma_gyro;
        meas_gyro[1] += dist(gen) * sigma_gyro;
        meas_gyro[2] += dist(gen) * sigma_gyro;

        //imu.accelerometer = true_acc;
        //imu.gyroscope = true_gyro;

        imu.accelerometer = meas_acc;
        imu.gyroscope = meas_gyro;

        return imu;
    }
};