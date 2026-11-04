#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>

/*создал структуру вектора со своим конструктором
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
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0]
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




struct ImuMeasurement {
    int timestamp;
    Vec accelerometer ;
    Vec gyroscope;
};

struct NavigationState {
    int timestamp;
    Vec velocity;
    Vec eulerAnguler;
    Mat matrix;
    Vec coordinate;
};

int main() {
    // ============ ТЕСТ 1: Матрица * вектор ============
    Mat I; // единичная
    Vec v(1, 2, 3);
    Vec r = I * v;
    std::cout << "I*v = [" << r[0] << ", " << r[1] << ", " << r[2] << "]\n";
    // Ожидание: [1, 2, 3]

    // ============ ТЕСТ 2: Поворот на 90° вокруг Z ============
    Mat Rz = Mat::fromEuler(0, 0, M_PI / 2);
    Vec ex(1, 0, 0);          // вектор вдоль X
    Vec ex_rot = Rz * ex;
    std::cout << "Rz(90)*ex = [" << ex_rot[0] << ", " << ex_rot[1] << ", " << ex_rot[2] << "]\n";
    // Ожидание: X должен перейти в Y: [~0, ~1, 0]

    // ============ ТЕСТ 3: Обратимость Эйлера ============
    double r0 = 0.3, p0 = -0.4, y0 = 0.5;
    Mat R = Mat::fromEuler(r0, p0, y0);
    double r_new, p, y;
    Mat::toEuler(R, r_new, p, y);
    std::cout << "in : " << r0 << "  " << p0 << "  " << y0 << '\n';
    std::cout << "out: " << r_new << "  " << p << "  " << y << '\n';
    // Ожидание: числа должны совпасть

    // ============ ТЕСТ 4: skew ============
    Vec w(1, 2, 3);
    Mat S = skew(w);
    Vec w_cross_v = S * v;              // S*v должно дать w × v
    Vec w_cross_v2 = cross(w, v);       // а это — то же самое напрямую
    std::cout << "skew test: [" << w_cross_v[0] << ", " << w_cross_v[1] << ", " << w_cross_v[2] << "]\n";
    std::cout << "cross test:[" << w_cross_v2[0] << ", " << w_cross_v2[1] << ", " << w_cross_v2[2] << "]\n";
    // Ожидание: оба вектора одинаковые

    // ============ ТЕСТ 5: Транспонирование ============
    Mat Rt = T(R);
    Mat prod = R * Rt;                  // R * R^T должно быть единичной матрицей
    std::cout << "R*R^T =\n";
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) std::cout << prod[i][j] << "  ";
        std::cout << '\n';
    }
    // Ожидание: единичная матрица с точностью до 1e-15

    // ============ ТЕСТ 6: Ортонормализация ============
    Mat M = Mat::fromEuler(0.1, 0.2, 0.3);
    // Слегка «испортим» матрицу
    M[0][0] += 1e-3;
    M.orthonormalize();
    // Проверим, что det = 1 и строки ортогональны
    std::cout << "После ортонормализации:\n";
    std::cout << "dot(c0, c1) = " << dot(Vec(M[0][0], M[1][0], M[2][0]),
        Vec(M[0][1], M[1][1], M[2][1])) << '\n';
    // Ожидание: ~0
}*/
