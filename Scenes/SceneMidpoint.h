#pragma once
#include "Scene.h"
#include <array>
#include <iostream>
#include "sim/Spring.h"


struct SceneMidpoint : public Scene
{
    std::vector<Point> points;
    std::vector<Spring> springs;
    time_step_t time_step;

    auto initialize_values() -> void
    {
        auto point1 = Point{
            .position = {0.0, 0.0, 0.0},
            .velocity = {-1.0, 0.0, 0.0},
            .force = {0.0, 0.0, 0.0},
            .mass = 10.0,
            .damping = 0.0,
            .fixed = false,
        };
        auto point2 = Point{
            .position = {0.0, 2.0, 0.0},
            .velocity = {1.0, 0.0, 0.0},
            .force = {0.0, 0.0, 0.0},
            .mass = 10.0,
            .damping = 0.0,
            .fixed = false,
        };
        points = {point1, point2};
        springs = {Spring{
            .point1 = 0,
            .point2 = 1,
            .stiffness = 40.0,
            .initialLength = 1.0,
        }};
        time_step = 0.005;
    }

    /// @brief Initialize the scene. Gets called every time the scene is switched to.
    virtual auto init() -> void override
    {
        initialize_values();
    };
    /// @brief Simulate a step in the scene. Gets called every frame before onDraw.
    ///
    /// This is where you should update the physics of the scene.
    virtual auto simulateStep() -> void override {
        midpoint_one_step(points, springs, time_step);
    };
    /// @brief Draw the scene. Gets called every frame after simulateStep.
    ///
    /// This is where you should call the Renderer draw functions.
    virtual auto onDraw(Renderer &renderer) -> void override
    {
        renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));

        for (auto &point : points)
        {
            renderer.drawSphere(point.position, 0.05);
        }

        for (auto &spring : springs) {
            vec3 pos1 = points[spring.point1].position;
            vec3 pos2 = points[spring.point2].position;
            renderer.drawLine(pos1, pos2, {0.6, 0.3, 0.3});
        }
    };
    /// @brief Define the GUI for the scene. Gets called every frame after onDraw.
    virtual auto onGUI() -> void override {

        ImGui::SliderFloat("Time step", &time_step, 0.0001, 0.5);

    };
    virtual ~SceneMidpoint() override = default;
};