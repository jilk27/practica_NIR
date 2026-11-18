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
        ImGui::SliderFloat("Амплитуда", &sim.signal_amplitude, 0.0f, 10.0f);
        ImGui::SliderFloat("Угловая скорость вокруг x", &sim.w_x, -2.0f, 2.0f);
        ImGui::SliderFloat("Угловая скорость вокруг y", &sim.w_y, -2.0f, 2.0f);
        ImGui::SliderFloat("Угловая скорость вокруг z", &sim.w_z, -2.0f, 2.0f);
        ImGui::End();

        t += dt;
        // 1 симулятор выдает структуру с сырыми данными
        ImuMeasurement imu = sim.getMeasurement(dt, sins.C_b_n);

        // 2 SINS обрабатывает данные
        sins.update(imu.accelerometer, imu.gyroscope, dt);

        times.push_back(t);

        // Выводим на график сырые данные из структуры
        acc_x_raw.push_back(imu.accelerometer[0]);
        acc_y_raw.push_back(imu.accelerometer[1]);
        acc_z_raw.push_back(imu.accelerometer[2]);
        //данные для гироскопа;
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

        //переключение между графиками акс и гиро
        ImGui::Begin("Графики датчиков"); 
        if (ImGui::BeginTabBar("MyTabBar")) {

            //АКСЕЛЕРОМЕТР
            if (ImGui::BeginTabItem("Акселерометры")) {
                if (ImPlot::BeginPlot("##acc_x", ImVec2(-1, 300))) {
                    ImPlot::SetupAxes("t (с)", "Ускорение X (м/с²)");
                    ImPlot::SetupAxesLimits(times.front(), times.back(), -15, 15, ImPlotCond_Always);
                    ImPlot::PlotLine("Сырые данные X", times.data(), acc_x_raw.data(), times.size());
                    ImPlot::EndPlot();
                }
                // Рисуем график Y
                if (ImPlot::BeginPlot("##acc_y", ImVec2(-1, 300))) {
                    ImPlot::SetupAxes("t (с)", "Ускорение Y (м/с²)");
                    ImPlot::SetupAxesLimits(times.front(), times.back(), -15, 15, ImPlotCond_Always);
                    ImPlot::PlotLine("Сырые данные Y", times.data(), acc_y_raw.data(), times.size());
                    ImPlot::EndPlot();
                }
                // Рисуем график Z
                if (ImPlot::BeginPlot("##acc_z", ImVec2(-1, 300))) {
                    ImPlot::SetupAxes("t (с)", "Ускорение Z (м/с²)");
                    ImPlot::SetupAxesLimits(times.front(), times.back(), -15, 15, ImPlotCond_Always);
                    ImPlot::PlotLine("Сырые данные Z", times.data(), acc_z_raw.data(), times.size());
                    ImPlot::EndPlot();
                }
                ImGui::EndTabItem();
            }

            // ГИРОСКОП
            if (ImGui::BeginTabItem("Гироскопы")) {
                // График X
                if (ImPlot::BeginPlot("##gyro_x", ImVec2(-1, 300))) {
                    ImPlot::SetupAxes("t (с)", "Угловая скорость X (рад/с)");
                    ImPlot::SetupAxesLimits(times.front(), times.back(), -0.001, 0.001, ImPlotCond_Always);
                    ImPlot::PlotLine("Гироскоп X", times.data(), gyro_x_raw.data(), times.size());
                    ImPlot::EndPlot();
                }
                // График Y
                if (ImPlot::BeginPlot("##gyro_y", ImVec2(-1, 300))) {
                    ImPlot::SetupAxes("t (с)", "Угловая скорость Y (рад/с)");
                    ImPlot::SetupAxesLimits(times.front(), times.back(), -0.001, 0.001, ImPlotCond_Always);
                    ImPlot::PlotLine("Гироскоп Y", times.data(), gyro_y_raw.data(), times.size());
                    ImPlot::EndPlot();
                }
                // График Z
                if (ImPlot::BeginPlot("##gyro_z", ImVec2(-1, 300))) {
                    ImPlot::SetupAxes("t (с)", "Угловая скорость Z (рад/с)");
                    ImPlot::SetupAxesLimits(times.front(), times.back(), -0.001, 0.001, ImPlotCond_Always);
                    ImPlot::PlotLine("Гироскоп Z", times.data(), gyro_z_raw.data(), times.size());
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