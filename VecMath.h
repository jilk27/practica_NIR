#pragma once
#include <iostream>
#include <cmath>

//создал структуру вектора со своим конструктором
//ниже прописал операторы для этого вектора 
struct Vec {
    double data[3];

    Vec(double x = 0.0, double y = 0.0, double z = 0.0) : data{ x, y, z } {}

    double& operator[](int i) { return data[i]; }
    double operator[](int i) const { return data[i]; }

};

inline Vec operator+(const Vec& a, const Vec& b) {
    Vec r;
    for (int i = 0; i < 3; i++) { r[i] = a[i] + b[i]; }
    return r;
}

inline Vec operator-(const Vec& a, const Vec& b) {
    Vec r;
    for (int i = 0; i < 3; i++) { r[i] = a[i] - b[i]; }
    return r;
}

inline Vec operator*(double a, const Vec& b) {
    Vec r;
    for (int i = 0; i < 3; i++) { r[i] = a * b[i]; }
    return r;
}

inline double dot(const Vec& a, const Vec& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline Vec cross(const Vec& a, const Vec& b) {
    return Vec(
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    );
}

inline double norm(const Vec& v) { return std::sqrt(dot(v, v)); }

inline Vec normalize(const Vec& v) {
    double n = norm(v);
    if (n < 1e-15) return Vec(0, 0, 0);
    return Vec(v[0] / n, v[1] / n, v[2] / n);
}


//создал структуру матриц, в нем прописал метод, возвращающий матрицу поворота по трём углам Эйлера и обратный метод
//также прописал операторы для матриц
struct Mat {
    double data[3][3] = {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    };

    double* operator[](int i) { return data[i]; }
    const double* operator[](int i) const { return data[i]; }

    static Mat fromEuler(double roll, double pitch, double yaw) {
        const double sr = std::sin(roll), cr = std::cos(roll);
        const double sp = std::sin(pitch), cp = std::cos(pitch);
        const double sy = std::sin(yaw), cy = std::cos(yaw);

        Mat m;
        m[0][0] = cy * cp;
        m[0][1] = -sy * cr + cy * sp * sr;
        m[0][2] = sy * sr + cy * sp * cr;

        m[1][0] = sy * cp;
        m[1][1] = cy * cr + sy * sp * sr;
        m[1][2] = -cy * sr + sy * sp * cr;

        m[2][0] = -sp;
        m[2][1] = cp * sr;
        m[2][2] = cp * cr;

        return m;
    }

    static void toEuler(const Mat& m, double& roll, double& pitch, double& yaw) {

        double cos_pitch = std::sqrt(m[2][1] * m[2][1] + m[2][2] * m[2][2]);

        pitch = std::atan2(-m[2][0], cos_pitch);

        // Проверка на «карданов замок», cos(pitch) близок к нулю 
        if (cos_pitch > 1e-6) {
            roll = std::atan2(m[2][1], m[2][2]);
            yaw = std::atan2(m[1][0], m[0][0]);
        }
        else {
            roll = 0.0;
            yaw = std::atan2(-m[0][1], m[1][1]);
        }

    }

    void orthonormalize() {
        Vec c0(data[0][0], data[1][0], data[2][0]);
        Vec c1(data[0][1], data[1][1], data[2][1]);

        c0 = normalize(c0);
        c1 = normalize(c1 - dot(c0, c1) * c0);
        Vec c2 = cross(c0, c1);

        data[0][0] = c0[0]; data[1][0] = c0[1]; data[2][0] = c0[2];
        data[0][1] = c1[0]; data[1][1] = c1[1]; data[2][1] = c1[2];
        data[0][2] = c2[0]; data[1][2] = c2[1]; data[2][2] = c2[2];
    }
};

inline Mat operator+(const Mat& A, const Mat& B) {
    Mat R;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R[i][j] = A[i][j] + B[i][j];
    return R;
}

inline Mat operator-(const Mat& A, const Mat& B) {
    Mat R;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R[i][j] = A[i][j] - B[i][j];
    return R;
}

inline Mat operator*(const Mat& A, const Mat& B) {
    Mat R;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0.0;
            for (int k = 0; k < 3; ++k) {
                sum += A[i][k] * B[k][j];
            }
            R[i][j] = sum;
        }
    }
    return R;
}

inline Mat operator*(double s, const Mat& A) {
    Mat R;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R[i][j] = s * A[i][j];
    return R;
}

//для уравнения Пуассона
inline Mat skew(const Vec& v) {
    Mat S;
    S[0][0] = 0;      S[0][1] = -v[2];  S[0][2] = v[1];
    S[1][0] = v[2];   S[1][1] = 0;     S[1][2] = -v[0];
    S[2][0] = -v[1];   S[2][1] = v[0];  S[2][2] = 0;
    return S;
}

inline Vec operator*(const Mat& A, const Vec& v) {
    Vec r;
    for (int i = 0; i < 3; ++i) {
        double sum = 0.0;
        for (int k = 0; k < 3; ++k) {
            sum += A[i][k] * v[k];
        }
        r[i] = sum;
    }
    return r;
}

inline Mat T(const Mat& A) {
    Mat R;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R[j][i] = A[i][j];
        }
    }
    return R;
}
