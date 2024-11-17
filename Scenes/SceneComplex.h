#pragma once
#include "Scene.h"
#include <array>
#include <iostream>
#include "sim/Spring.h"

struct Aabb {
    vec3 min;
    vec3 max;
};

auto enforce_bbox(Aabb bb, std::vector<Point>& points) {
    // if we went out of a face, we flip our velocity and position in that distance
    // and pretend it is business as usual
    for (auto &point: points) {
        vec3 pos = point.position;
        vec3 vel = point.velocity;
        if (pos.x > bb.max.x) {
            point.position.x +=  pos.x - bb.max.x;
            point.velocity.x = -vel.x;
        } else if (pos.x < bb.min.x) {
            point.position.x += pos.x - bb.min.x;
            point.velocity.x = -vel.x;
        }
        if (pos.y > bb.max.y) {
            point.position.y += pos.y - bb.max.y;
            point.velocity.y = -vel.y;
        } else if (pos.y < bb.min.y) {
            point.position.y += pos.y - bb.min.y;
            point.velocity.y = -vel.y;
        }
        if (pos.z > bb.max.z) {
            point.position.z += pos.z - bb.max.z;
            point.velocity.z = -vel.z;
        } else if (pos.y < bb.min.z) {
            point.position.z += pos.z - bb.min.z;
            point.velocity.z = -vel.z;
        }
    }
}


struct SceneComplex : public Scene
{
    int method;
    std::vector<Point> points;
    std::vector<Spring> springs;
    time_step_t time_step;
    Aabb bounding_box;
    bool gravity;

    auto initialize_values() -> void
    {
        bounding_box = Aabb {
            .min = {-2.5, -2.5, -2.5},
            .max = {2.5, 2.5, 2.5},
        };
        points = {
            Point{
                .position = {0.0, 0.0, 0.0},
                .velocity = {-1.0, 0.0, 0.0},
                .force = {0.0, 0.0, 0.0},
                .mass = 10.0,
                .damping = 0.0,
                .fixed = false,
            },
            Point{
                .position = {0.0, 2.0, 0.0},
                .velocity = {1.0, 0.0, 0.0},
                .force = {0.0, 0.0, 0.0},
                .mass = 10.0,
                .damping = 0.0,
                .fixed = false,
            },
            Point{
                .position = {0.0, 2.0, 0.0},
                .velocity = {1.0, 0.0, 0.0},
                .force = {0.0, 0.0, 0.0},
                .mass = 10.0,
                .damping = 0.0,
                .fixed = false,
            },
            Point{
                .position = {0.0, 2.0, 0.0},
                .velocity = {1.0, 0.0, 0.0},
                .force = {0.0, 0.0, 0.0},
                .mass = 10.0,
                .damping = 0.0,
                .fixed = false,
            },
            Point{
                .position = {0.0, 2.0, 0.0},
                .velocity = {1.0, 0.0, 0.0},
                .force = {0.0, 0.0, 0.0},
                .mass = 10.0,
                .damping = 0.0,
                .fixed = false,
            },
            Point{
                .position = {0.0, 2.0, 0.0},
                .velocity = {1.0, 0.0, 0.0},
                .force = {0.0, 0.0, 0.0},
                .mass = 10.0,
                .damping = 0.0,
                .fixed = false,
            },
            Point{
                .position = {0.0, 2.0, 0.0},
                .velocity = {1.0, 0.0, 0.0},
                .force = {0.0, 0.0, 0.0},
                .mass = 10.0,
                .damping = 0.0,
                .fixed = false,
            },
            Point{
                .position = {0.0, 2.0, 0.0},
                .velocity = {1.0, 0.0, 0.0},
                .force = {0.0, 0.0, 0.0},
                .mass = 10.0,
                .damping = 0.0,
                .fixed = false,
            },
            Point{
                .position = {0.0, 2.0, 0.0},
                .velocity = {1.0, 0.0, 0.0},
                .force = {0.0, 0.0, 0.0},
                .mass = 10.0,
                .damping = 0.0,
                .fixed = false,
            },
        };
        springs = {
            Spring{
                .point1 = 0,
                .point2 = 1,
                .stiffness = 40.0,
                .initialLength = 1.0,
            },
            Spring{
                .point1 = 0,
                .point2 = 1,
                .stiffness = 40.0,
                .initialLength = 1.0,
            },
            Spring{
                .point1 = 0,
                .point2 = 1,
                .stiffness = 40.0,
                .initialLength = 1.0,
            },
            Spring{
                .point1 = 0,
                .point2 = 1,
                .stiffness = 40.0,
                .initialLength = 1.0,
            },
            Spring{
                .point1 = 0,
                .point2 = 1,
                .stiffness = 40.0,
                .initialLength = 1.0,
            },
            Spring{
                .point1 = 0,
                .point2 = 1,
                .stiffness = 40.0,
                .initialLength = 1.0,
            },
            Spring{
                .point1 = 0,
                .point2 = 1,
                .stiffness = 40.0,
                .initialLength = 1.0,
            },
            Spring{
                .point1 = 0,
                .point2 = 1,
                .stiffness = 40.0,
                .initialLength = 1.0,
            },
            Spring{
                .point1 = 0,
                .point2 = 1,
                .stiffness = 40.0,
                .initialLength = 1.0,
            },
            Spring{
                .point1 = 0,
                .point2 = 1,
                .stiffness = 40.0,
                .initialLength = 1.0,
            },
        };
        time_step = 0.005;
    }

    virtual auto init() -> void override
    {
        initialize_values();
    };

    virtual auto simulateStep() -> void override
    {
        if (method == 0)
        {
            euler_one_step(points, springs, time_step);
        }
        else if (method == 1)
        {
            midpoint_one_step(points, springs, time_step);
        }
        else
        {
        }
        enforce_bbox(bounding_box, points);
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

    virtual auto onGUI() -> void override
    {
        ImGui::SliderFloat("Time step", &time_step, 0.0001, 0.5);
        ImGui::Combo("Integration Method", &method, "Euler\0Midpoint\0", -1);
    };
    virtual ~SceneComplex() override = default;
};