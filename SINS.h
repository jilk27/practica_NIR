#pragma once
#include "VecMath.h"

class SINS {
public:
    static constexpr double OMEGA_EARTH = 7.292115e-5;
    static constexpr double G_ACCEL = 9.81;
    static constexpr double R_EARTH = 6378137.0;
    static constexpr double DEG2RAD = 3.14159265358979 / 180.0;

    double lat;       // широта
    double lon;       // долгота
    double height;    // высота 
    Vec   V;         // скорость в навигационной системе: [Vn, Ve, Vd]
    Mat   C_b_n;     // матрица перехода

    SINS(double lat_deg = 55.0, double lon_deg = 37.0, double h = 100.0)
        : lat(lat_deg* DEG2RAD),
        lon(lon_deg* DEG2RAD),
        height(h),
        V(0, 0, 0),
        C_b_n()
    {
    }

    //шаг интегрирования
    void update(const Vec& acc_b, const Vec& gyro_b, double dt) {

        Vec omega_ie_n(
            OMEGA_EARTH * std::cos(lat),
            0.0,
            -OMEGA_EARTH * std::sin(lat)
        );

        
        Mat C_n_b = T(C_b_n);
        Vec omega_ie_b = C_n_b * omega_ie_n;
        
        Vec omega_nb_b = gyro_b - omega_ie_b;


        //обновление матрицы ориентации
        Mat Omega_nb_b = skew(omega_nb_b);
        Mat I_plus = Omega_nb_b;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                I_plus[i][j] = (i == j ? 1.0 : 0.0) + Omega_nb_b[i][j] * dt;

        C_b_n = C_b_n * I_plus;
        orthonormalize();


        Vec acc_n = C_b_n * acc_b;


        Vec g_n(0.0, 0.0, G_ACCEL);
        Vec a_n = acc_n - g_n;

        //интегрирование скорости
        V = V + dt * a_n;

        //интегрирование координат
        lat += (V[0] / R_EARTH) * dt;
        lon += (V[1] / (R_EARTH * std::cos(lat))) * dt;
        height -= V[2] * dt;
    }

    // ортонормализация матрицы (метод Грама-Шмидта)
    void orthonormalize() {
        Vec c0(C_b_n[0][0], C_b_n[1][0], C_b_n[2][0]);
        Vec c1(C_b_n[0][1], C_b_n[1][1], C_b_n[2][1]);
        Vec c2;

        c0 = (1.0 / norm(c0)) * c0;

        double dot = c0[0] * c1[0] + c0[1] * c1[1] + c0[2] * c1[2];
        c1 = c1 - dot * c0;
        c1 = (1.0 / norm(c1)) * c1;

        c2[0] = c0[1] * c1[2] - c0[2] * c1[1];
        c2[1] = c0[2] * c1[0] - c0[0] * c1[2];
        c2[2] = c0[0] * c1[1] - c0[1] * c1[0];

        C_b_n[0][0] = c0[0]; C_b_n[1][0] = c0[1]; C_b_n[2][0] = c0[2];
        C_b_n[0][1] = c1[0]; C_b_n[1][1] = c1[1]; C_b_n[2][1] = c1[2];
        C_b_n[0][2] = c2[0]; C_b_n[1][2] = c2[1]; C_b_n[2][2] = c2[2];
    }

};