#pragma once

#include "Scene.h"
#include "sim/RigidBody.h"
#include "sim/glm_print.h"
#include <array>
#include <iostream>

using vec3 = glm::vec3;
using vec4 = glm::vec4;
using mat3x3 = glm::mat3x3;
using f32 = float;
using f64 = double;
using time_step_t = f32;
using Quaternion = glm::quat;

struct RigidScene4 : public Scene
{
    std::vector<RigidBody> rigid_bodies;
    std::vector<Force> forces;
    time_step_t time_step;
    f32 gravity;
    f64 time_cumulative;

    bool play = false;

    auto initialize_values() -> void
    {
        auto const extent = vec3{1.0f, 0.6f, 0.5f};
        auto const x_cm = vec3{0.0f};
        auto const x_cm2 = vec3{2.0f, 0.0f, 0.0f};
        auto const x_cm3 = vec3{0.0f, -2.0f, 0.0f};
        auto const x_cm4 = vec3{1.0f, -1.0f, 0.3f};

        auto const orientation = Quaternion{vec3{0.0f, 0.0f, 0.5f * glm::pi<f32>()}};
        auto const orientation2 = Quaternion{vec3{0.0f, 0.0f * glm::pi<f32>(), 0.25f * glm::pi<f32>()}};

        f32 const mass = 8.0f;

        RigidBody floor = RigidBody::new_still(
            vec3{1000.0f, 1000.0f, 100.0f},
            vec3{0.0f, 0.0f, -52.5f},
            Quaternion{vec3{0.0f}},
            1000000000.0f);
        floor.fixed = true;
        floor.visible = false;

        rigid_bodies = {
            RigidBody::new_still(extent, x_cm, orientation, mass),
            RigidBody::new_still(extent, x_cm2, orientation2, mass),
            RigidBody::new_still(extent, x_cm3, orientation, mass),
            RigidBody::new_still(extent, x_cm4, orientation, mass),
            floor,
            };

        time_step = 0.01f;
        gravity = 0.0f;
        time_cumulative = 0.0;

        rigid_bodies.at(1).velocity_lin = vec3 {-0.25f, 0.0f, 0.0f};

        forces = {
            Force{
                vec3{-0.3f, 0.5f, -0.25f},
                vec3{0.0f, 1.0f, 1.0f},
            },
            Force{
                vec3{0.3f, 0.5f, 0.25f},
                vec3{1.0f, 1.0f, 0.0f},
            }
            };
    }

    virtual auto init() -> void override
    {
        initialize_values();
    };

    virtual auto simulateStep() -> void override
    {
        if (play)
        {
            time_cumulative += time_step;
            euler_one_step_collisions(rigid_bodies, forces, time_step, gravity);
        }
        if (time_cumulative > 1.0)
        {
            forces = {};
        }
    };

    virtual auto onDraw(Renderer &renderer) -> void override
    {
        renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));
        for (auto const &body : rigid_bodies)
        {
            draw_rigidbody(renderer, body);
        }
        for (auto const &force : forces)
        {
            draw_force(renderer, force);
        }
    };

    virtual auto onGUI() -> void override
    {
        ImGui::Checkbox("Play", &play);
        ImGui::SliderFloat("Time step", &time_step, 0.001, 0.1);
        ImGui::SliderFloat("Gravity: ", &gravity, 0.0, 20.0);
    };
    virtual ~RigidScene4() override = default;
};
