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
using time_step_t = f32;
using Quaternion = glm::quat;

struct RigidScene3 : public Scene
{
    std::vector<RigidBody> rigid_bodies;
    std::vector<Force> forces;
    time_step_t time_step;

    bool play = false;

    auto initialize_values() -> void
    {
        auto const extent = vec3{1.0f, 0.6f, 0.5f};
        auto const x_cm = vec3{0.0f};
        auto const x_cm2 = vec3{2.0f, 0.0f, 0.0f};

        auto const orientation = Quaternion{vec3{0.0f, 0.0f, 0.5f * glm::pi<f32>()}};
        auto const orientation2 = Quaternion{vec3{0.0f, 0.0f * glm::pi<f32>(), 0.25f * glm::pi<f32>()}};

        std::cout << "orientation(r): " << orientation << "\n";

        f32 const mass = 2.0f;

        rigid_bodies = {
            RigidBody::new_still(extent, x_cm, orientation, mass),
            RigidBody::new_still(extent, x_cm2, orientation2, mass),
            };

        time_step = 0.01f;

        rigid_bodies.at(1).velocity_lin = vec3 {-0.25f, 0.0f, 0.0f};

        forces = {
            //Force{
            //    vec3{0.3f, 0.5f, 0.25f},
            //    vec3{1.0f, 1.0f, 0.0f},
            //}
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
            euler_one_step_collisions(rigid_bodies, forces, time_step, 0.0);
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
    virtual ~RigidScene3() override = default;
};
