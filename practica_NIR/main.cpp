#define _USE_MATH_DEFINES
#include "imgui.h"
#include "implot.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>
#include <vector>
#include <cmath>
#include "SINS.h"
#include "Simulator.h"
#include "VecMath.h"

int main() {
    // --- инициализация GLFW + OpenGL ---
    glfwInit();
    GLFWwindow* window = glfwCreateWindow(1280, 720, "SINS Simulator", nullptr, nullptr);
    glfwMakeContextCurrent(window);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();

    //делает белый фон
    ImGui::StyleColorsLight();
    ImPlot::StyleColorsLight();

    ImGuiIO& io = ImGui::GetIO();
    io.Fonts->AddFontFromFileTTF("C:/Windows/Fonts/arial.ttf", 18.0f, NULL, io.Fonts->GetGlyphRangesCyrillic());

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    Simulator sim;
    SINS sins;

    float amplitude = 2.0f;
    float frequency = 0.2f;

    std::vector<double> times, acc_x_raw, acc_y_raw, acc_z_raw, acc_z_ideal;
    std::vector<double> gyro_x_raw, gyro_y_raw, gyro_z_raw;
    double t = 0.0;
    const double dt = 0.01;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Управление");
        ImGui::SliderFloat("Ускорение x", &sim.a_x, 0.0f, 10.0f);
        ImGui::SliderFloat("Ускорение y", &sim.a_y, 0.0f, 10.0f);
        ImGui::SliderFloat("Ускорение z", &sim.a_z, 0.0f, 10.0f);
        ImGui::SliderFloat("Угловая скорость вокруг x", &sim.w_x, -2.0f, 2.0f);
        ImGui::SliderFloat("Угловая скорость вокруг y", &sim.w_y, -2.0f, 2.0f);
        ImGui::SliderFloat("Угловая скорость вокруг z", &sim.w_z, -2.0f, 2.0f);
        ImGui::End();

        t += dt;
        // 1 симулятор выдает структуру с сырыми данными
        ImuMeasurement imu = sim.getMeasurement(dt);

        // 2 SINS обрабатывает данные
        sins.update(imu.accelerometer, imu.gyroscope, dt);



        ImGui::Begin("Навигационное решение SINS");

        ImGui::Text("Широта: %.6f град", sins.lat * 180.0 / M_PI);
        ImGui::Text("Долгота: %.6f град", sins.lon * 180.0 / M_PI);
        ImGui::Text("Высота: %.2f м", sins.height);

        ImGui::Text("Скорость Vn (Север): %.4f м/с", sins.V[0]);
        ImGui::Text("Скорость Ve (Восток): %.4f м/с", sins.V[1]);
        ImGui::Text("Скорость Vd (Вниз): %.4f м/с", sins.V[2]);

        double sins_roll, sins_pitch, sins_yaw;
        Mat::toEuler(sins.C_b_n, sins_roll, sins_pitch, sins_yaw);
        ImGui::Text("Крен (SINS): %.2f град", sins_roll * 180.0 / M_PI);
        ImGui::Text("Тангаж (SINS): %.2f град", sins_pitch * 180.0 / M_PI);
        ImGui::Text("Рыскание (SINS): %.2f град", sins_yaw * 180.0 / M_PI);

        ImGui::End();

        ImGui::Begin("Реальные данные из simulator");

        double true_roll, true_pitch, true_yaw;
        Mat::toEuler(sim.C_b_n_true, true_roll, true_pitch, true_yaw);
        ImGui::Text("Крен (SINS): %.2f град", sins_roll * 180.0 / M_PI);
        ImGui::Text("Тангаж (SINS): %.2f град", sins_pitch * 180.0 / M_PI);
        ImGui::Text("Рыскание (SINS): %.2f град", sins_yaw * 180.0 / M_PI);
        ImGui::End();



        times.push_back(t);

        // Выводим на график сырые данные из структуры
        acc_x_raw.push_back(imu.accelerometer[0]);
        acc_y_raw.push_back(imu.accelerometer[1]);
        acc_z_raw.push_back(imu.accelerometer[2]);
        //данные для гироскопа
        gyro_x_raw.push_back(imu.gyroscope[0]);
        gyro_y_raw.push_back(imu.gyroscope[1]);
        gyro_z_raw.push_back(imu.gyroscope[2]);

        if (times.size() > 1000) {
            times.erase(times.begin());
            acc_x_raw.erase(acc_x_raw.begin());
            acc_y_raw.erase(acc_y_raw.begin());
            acc_z_raw.erase(acc_z_raw.begin());
            
            gyro_x_raw.erase(gyro_x_raw.begin());
            gyro_y_raw.erase(gyro_y_raw.begin());
            gyro_z_raw.erase(gyro_z_raw.begin());
        }



        ImGui::Begin("Модель ошибок датчиков");

        // --- Ошибки акселерометра ---
        if (ImGui::CollapsingHeader("Акселерометр")) {
            // Bias
            ImGui::InputDouble("Bias X (акс)", &sim.accel_errors.bias[0], 0.01, 0.1, "%.4f");
            ImGui::InputDouble("Bias Y (акс)", &sim.accel_errors.bias[1], 0.01, 0.1, "%.4f");
            ImGui::InputDouble("Bias Z (акс)", &sim.accel_errors.bias[2], 0.01, 0.1, "%.4f");

            // Дисперсия шума
            ImGui::InputDouble("Дисперсия шума (акс)", &sim.accel_errors.noise, 0.0001, 0.001, "%.6f");

            // Матрица H (для простоты зададим только диагональные элементы - масштабные коэффициенты)
            ImGui::Text("Матрица H (акс):");
            if (ImGui::BeginTable("AccelHTable", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_SizingStretchSame)) {
                for (int i = 0; i < 3; i++) {
                    ImGui::TableNextRow();
                    for (int j = 0; j < 3; j++) {
                        ImGui::TableSetColumnIndex(j);
                        ImGui::PushID(i * 3 + j);
                        ImGui::SetNextItemWidth(-FLT_MIN);
                        ImGui::InputDouble("##val", &sim.accel_errors.H[i][j], 0.0001, 0.001, "%.4f");
                        ImGui::PopID();
                    }
                }
                ImGui::EndTable();
            }
        }

        // --- Ошибки гироскопа ---
        if (ImGui::CollapsingHeader("Гироскоп")) {
            ImGui::InputDouble("Bias X (гиро)", &sim.gyro_errors.bias[0], 0.0001, 0.001, "%.6f");
            ImGui::InputDouble("Bias Y (гиро)", &sim.gyro_errors.bias[1], 0.0001, 0.001, "%.6f");
            ImGui::InputDouble("Bias Z (гиро)", &sim.gyro_errors.bias[2], 0.0001, 0.001, "%.6f");

            ImGui::InputDouble("Дисперсия шума (гиро)", &sim.gyro_errors.noise, 0.000001, 0.00001, "%.8f");

            ImGui::Text("Матрица H (гиро):");
            if (ImGui::BeginTable("GyroHTable", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_SizingStretchSame)) {
                for (int i = 0; i < 3; i++) {
                    ImGui::TableNextRow();
                    for (int j = 0; j < 3; j++) {
                        ImGui::TableSetColumnIndex(j);
                        ImGui::PushID(i * 3 + j + 10);
                        ImGui::SetNextItemWidth(-FLT_MIN);
                        ImGui::InputDouble("##val", &sim.gyro_errors.H[i][j], 0.0001, 0.001, "%.6f");
                        ImGui::PopID();
                    }
                }
                ImGui::EndTable();
            }
        }

        // --- Кнопка сброса ---
        if (ImGui::Button("Сбросить ошибки")) {
            sim.accel_errors = ImuError();
            sim.gyro_errors = ImuError();
        }

        ImGui::SameLine();

        if (ImGui::Button("ПОЛНЫЙ СБРОС (Начать заново)")) {
            t = 0.0;

            sim = Simulator();
            sins = SINS();

            times.clear();
            acc_x_raw.clear();
            acc_y_raw.clear();
            acc_z_raw.clear();
            gyro_x_raw.clear();
            gyro_y_raw.clear();
            gyro_z_raw.clear();
        }

        ImGui::End();

        //переключение между графиками акс и гиро
        ImGui::Begin("Графики датчиков"); 
        if (ImGui::BeginTabBar("MyTabBar")) {

            //АКСЕЛЕРОМЕТР
            if (ImGui::BeginTabItem("Акселерометры")) {
                if (ImPlot::BeginPlot("##acc_x", ImVec2(-1, 300))) {
                    ImPlot::SetupAxes("t (с)", "Ускорение X (м/с²)");
                    if (!times.empty()) {
                        ImPlot::SetupAxesLimits(times.front(), times.back(), -15, 15, ImPlotCond_Always);
                        ImPlot::PlotLine("Данные акселерометра на X", times.data(), acc_x_raw.data(), times.size());
                    }
                    else {
                        ImPlot::SetupAxesLimits(0.0, 10.0, -15, 15, ImPlotCond_Always);
                    }
                    ImPlot::EndPlot();
                }
                // Рисуем график Y
                if (ImPlot::BeginPlot("##acc_y", ImVec2(-1, 300))) {
                    ImPlot::SetupAxes("t (с)", "Ускорение Y (м/с²)");
                    if (!times.empty()) {
                        ImPlot::SetupAxesLimits(times.front(), times.back(), -15, 15, ImPlotCond_Always);
                        ImPlot::PlotLine("Данные акселерометра на Y", times.data(), acc_y_raw.data(), times.size());
                    }
                    else ImPlot::SetupAxesLimits(0.0, 10.0, -15, 15, ImPlotCond_Always);
                    ImPlot::EndPlot();
                }
                // Рисуем график Z
                if (ImPlot::BeginPlot("##acc_z", ImVec2(-1, 300))) {
                    ImPlot::SetupAxes("t (с)", "Ускорение Z (м/с²)");
                    if (!times.empty()) {
                        ImPlot::SetupAxesLimits(times.front(), times.back(), -15, 15, ImPlotCond_Always);
                        ImPlot::PlotLine("Данные акселерометра на Z", times.data(), acc_z_raw.data(), times.size());
                    }
                    else ImPlot::SetupAxesLimits(0.0, 10.0, -15, 15, ImPlotCond_Always);
                    ImPlot::EndPlot();

                }
                ImGui::EndTabItem();
            }

            // ГИРОСКОП
            if (ImGui::BeginTabItem("Гироскопы")) {
                // График X
                if (ImPlot::BeginPlot("##gyro_x", ImVec2(-1, 300))) {
                    ImPlot::SetupAxes("t (с)", "Угловая скорость X (рад/с)");
                    if (!times.empty()) {
                        ImPlot::SetupAxesLimits(times.front(), times.back(), -0.001, 0.001, ImPlotCond_Always);
                        ImPlot::PlotLine("Гироскоп X", times.data(), gyro_x_raw.data(), times.size());
                    }
                    else ImPlot::SetupAxesLimits(0.0, 10.0, -0.001, 0.001, ImPlotCond_Always);
                    ImPlot::EndPlot();
                }
                // График Y
                if (ImPlot::BeginPlot("##gyro_y", ImVec2(-1, 300))) {
                    ImPlot::SetupAxes("t (с)", "Угловая скорость Y (рад/с)");
                    if (!times.empty()) {
                        ImPlot::SetupAxesLimits(times.front(), times.back(), -0.001, 0.001, ImPlotCond_Always);
                        ImPlot::PlotLine("Гироскоп Y", times.data(), gyro_y_raw.data(), times.size());
                    }
                    else ImPlot::SetupAxesLimits(0.0, 10.0, -0.001, 0.001, ImPlotCond_Always);
                    ImPlot::EndPlot();
                }
                // График Z
                if (ImPlot::BeginPlot("##gyro_z", ImVec2(-1, 300))) {
                    ImPlot::SetupAxes("t (с)", "Угловая скорость Z (рад/с)");
                    if (!times.empty()) {
                        ImPlot::SetupAxesLimits(times.front(), times.back(), -0.001, 0.001, ImPlotCond_Always);
                        ImPlot::PlotLine("Гироскоп Z", times.data(), gyro_z_raw.data(), times.size());
                    }
                    else ImPlot::SetupAxesLimits(0.0, 10.0, -0.001, 0.001, ImPlotCond_Always);
                    ImPlot::EndPlot();
                }
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
        ImGui::End(); 

        ImGui::Render();
        int w, h;
        glfwGetFramebufferSize(window, &w, &h);
        glViewport(0, 0, w, h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}