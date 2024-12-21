#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib> // For random initialization
#include <iomanip> // For formatted output
#include "Renderer.h" // Assuming Renderer library is available for visualization

using namespace std;

// Constants
const double nu = 0.1; // Thermal diffusivity
float dt = 0.001; // Time step
const double x_min = 0.0, x_max = 2.0; // Domain in x-direction
const double y_min = 0.0, y_max = 4.0; // Domain in y-direction

bool play = false;
glm::f64 time_cumulative;

// Data structure to represent the temperature grid
class TemperatureGrid {
public:
    TemperatureGrid(int rows, int cols)
        : m_rows(rows), m_cols(cols), data(rows * cols, 0.0) {}

    double& operator()(int i, int j) {
        return data[i * m_cols + j];
    }

    double operator()(int i, int j) const {
        return data[i * m_cols + j];
    }

    int rows() const { return m_rows; }
    int cols() const { return m_cols; }

private:
    int m_rows, m_cols;
    std::vector<double> data;
};

class ExplicitSimulation:public Scene {
public:
    ExplicitSimulation()
    : m(16), n(16), dx((1.0) / (16 + 1)), dy((1.0) / (16 + 1)),
      T(16, 16), image(16 * 16, 0){
        initializeGrid();
    }

    /*
    void simulate(Renderer &renderer, int timeSteps) {
        for (int t = 0; t < timeSteps; ++t) {
            step();
            onDraw(renderer);
        }

    }*/

private:
    int m, n = 16;
    double dx, dy;
    TemperatureGrid T;
    std::vector<float> image;

    void initializeGrid() {
        time_cumulative = 0.0;

        for (int i = 0; i < m; ++i) {
            for (int j = 0; j < n; ++j) {
                T(i, j) = (rand() % 100) / 100.0 - 0.5; // Random values between -0.5 and 0.5
            }
        }
    }

    void simulateStep() override {
            if (play) {
                time_cumulative += dt;
                expliciteuler();
            }

    }

    void onDraw(Renderer &renderer) override {

        renderer.drawImage(image, m, n); // Render the image
    }

    void onGUI() override {
        ImGui::Checkbox("Play", &play);
        ImGui::SliderFloat("Time step", &dt, 0.001f, 0.1f);

    }

    void expliciteuler() {
        TemperatureGrid T_new(m, n);

        for (int i = 0; i < m; ++i) {
            for (int j = 0; j < n; ++j) {
                double Txx = 0.0, Tyy = 0.0;

                // Compute second derivatives in x and y directions (handling boundaries)
                if (i > 0) Txx += T(i - 1, j);
                if (i < m - 1) Txx += T(i + 1, j);
                Txx -= 2 * T(i, j);
                Txx /= (dx * dx);

                if (j > 0) Tyy += T(i, j - 1);
                if (j < n - 1) Tyy += T(i, j + 1);
                Tyy -= 2 * T(i, j);
                Tyy /= (dy * dy);

                // Update temperature
                T_new(i, j) = T(i, j) + nu * dt * (Txx + Tyy);
            }
        }

        T = T_new; // Update the temperature field
        for (int i = 0; i < m; ++i) {
            for (int j = 0; j < n; ++j) {
                // Normalize temperature to a value between 0 and 255 for visualization
                double normalized = (T(i, j) + 0.5) * 255.0;
                image[i * n + j] = static_cast<uint8_t>(std::min(std::max(normalized, 0.0), 255.0));
            }
        }

    }
};

