#pragma once
#include "Scene.h"
#include <array>
#include <iostream>
#include "sim/Spring.h"

struct SceneSingleStep : public Scene
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
        time_step = 0.1;
    }

    virtual auto init() -> void override
    {
        initialize_values();
        euler_one_step(points, springs, time_step, 0.0f);

        std::cout << "Euler:\n"
                  << "point 1:\n"
                  << points.at(0) << "\n"
                  << "point 2:\n"
                  << points.at(1) << "\n";

        initialize_values();
        midpoint_one_step(points, springs, time_step, 0.0f);

        std::cout << "\nMidpoint:\n"
                  << "point 1:\n"
                  << points.at(0) << "\n"
                  << "point 2:\n"
                  << points.at(1) << "\n";
    };

    virtual auto simulateStep() -> void override {};

    virtual auto onDraw(Renderer &renderer) -> void override
    {
        renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));

        for (auto &point : points)
        {
            renderer.drawSphere(point.position, 0.05);
        }
    };
    
    virtual auto onGUI() -> void override {};
    virtual ~SceneSingleStep() override = default;
};