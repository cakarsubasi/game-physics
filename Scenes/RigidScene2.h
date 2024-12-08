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

struct RigidScene2 : public Scene
{
    std::vector<RigidBody> rigid_bodies;
    std::vector<Force> forces;
    time_step_t time_step;
    f64 time_cumulative;

    bool play = false;

    auto initialize_values() -> void
    {
        auto const extent = vec3{1.0f, 0.6f, 0.5f};
        auto const x_cm = vec3{0.0f};

        auto const orientation = Quaternion{vec3{0.0f, 0.0f, 0.5f * glm::pi<f32>()}};

        std::cout << "orientation(r): " << orientation << "\n";

        f32 const mass = 2.0f;

        rigid_bodies = {
            RigidBody::new_still(extent, x_cm, orientation, mass)};

        time_step = 0.01f;
        time_cumulative = 0.0;

        forces = {
            Force{
                vec3{0.3f, 0.5f, 0.25f},
                vec3{1.0f, 1.0f, 0.0f},
            }};
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
            euler_one_step(rigid_bodies, forces, time_step, 0.0);
        }
        if (time_cumulative > 2.0) {
            forces.clear();
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
    };
    virtual ~RigidScene2() override = default;
};
