#pragma once
#include "Scene.h"
#include <array>
#include <iostream>
#include "sim/Spring.h"

struct SceneEuler : public Scene
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
        };
        auto point2 = Point{
            .position = {0.0, 2.0, 0.0},
            .velocity = {1.0, 0.0, 0.0},
            .force = {0.0, 0.0, 0.0},
            .mass = 10.0,
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

    virtual auto init() -> void override
    {
        initialize_values();
    };
    virtual auto simulateStep() -> void override
    {
        euler_one_step(points, springs, time_step, 0.0f);
    };
    virtual auto onDraw(Renderer &renderer) -> void override
    {
        renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));

        for (auto &point : points)
        {
            renderer.drawSphere(point.position, 0.05);
        }

        for (auto &spring : springs)
        {
            vec3 pos1 = points[spring.point1].position;
            vec3 pos2 = points[spring.point2].position;
            renderer.drawLine(pos1, pos2, {0.6, 0.3, 0.3});
        }
    };
    virtual auto onGUI() -> void override {

        ImGui::SliderFloat("Time step", &time_step, 0.001, 0.1);
    };
    virtual ~SceneEuler() override = default;
};