#define _USE_MATH_DEFINES
#include "imgui.h"
#include "implot.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>
#include <vector>
#include <cmath>
#include "SINS.h"

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

    float amplitude = 2.0f;
    float frequency = 0.2f;
    float pitch_deg = 0.0f;

    std::vector<double> times, acc_z, signal_clean;
    double t = 0.0;
    const double dt = 0.001;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Управление");
        ImGui::SliderFloat("Амплитуда", &amplitude, 0.0f, 10.0f);
        ImGui::SliderFloat("Частота", &frequency, 0.0f, 5.0f);
        ImGui::SliderFloat("Тангаж", &pitch_deg, -90.0f, 90.0f);
        ImGui::End();

        // --- генерация новых данных ---
        t += dt;
        double a_signal = amplitude * std::sin(2.0 * M_PI * frequency * t);
        double g_b_z = SINS::G_ACCEL * std::cos(pitch_deg * SINS::DEG2RAD);

        times.push_back(t);
        acc_z.push_back(a_signal + g_b_z);      // с гравитацией
        signal_clean.push_back(a_signal + g_b_z); // идеальный сигнал

        if (times.size() > 2000) {
            times.erase(times.begin());
            acc_z.erase(acc_z.begin());
            signal_clean.erase(signal_clean.begin());
        }

        // --- окно с графиком ---
        ImGui::Begin("График показаний акселерометра");
        if (ImPlot::BeginPlot("##acc", ImVec2(-1, -1))) {
            ImPlot::SetupAxes("t (с)", "Ускорение (м/с²)");
            ImPlot::SetupAxesLimits(times.front(), times.back(), -15, 15, ImPlotCond_Always);
            ImPlot::PlotLine("Сырые данные (с g)", times.data(), acc_z.data(), times.size());
            ImPlot::PlotLine("Идеальный сигнал", times.data(), signal_clean.data(), times.size());
            ImPlot::EndPlot();
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

    // --- очистка ---
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}