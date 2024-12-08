#pragma once

#include "Scene.h"
// #include "sim/Quaternion.h"
#include "sim/RigidBody.h"
#include "sim/glm_print.h"
// #include <glm/gtx/quaternion.hpp>
#include <array>
#include <iostream>

// #define DEBUG

using vec3 = glm::vec3;
using vec4 = glm::vec4;
using mat3x3 = glm::mat3x3;
using f32 = float;
using time_step_t = f32;
using Quaternion = glm::quat;

struct RigidSceneSingleStep : public Scene
{
    std::vector<RigidBody> rigid_bodies;
    std::vector<Force> forces;
    time_step_t time_step;

    Force visual_force;

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

        time_step = 2.0f;

        forces = {
            Force{
                vec3{0.3f, 0.5f, 0.25f},
                vec3{1.0f, 1.0f, 0.0f},
            }};

        visual_force = forces.at(0);
    }

    virtual auto init() -> void override
    {
        initialize_values();
        auto const poi = vec3{-0.3f, -0.5f, -0.25f};
        euler_one_step(rigid_bodies, forces, time_step, 0.0f);

        auto &rb = rigid_bodies.at(0);
        std::cout << "pos: " << rb.position_of(poi) << "\n"
                  << "vel: " << rb.velocity_of(poi) << "\n";
    };

    virtual auto simulateStep() -> void override
    {
        forces.clear();
        if (play)
        {
            euler_one_step(rigid_bodies, forces, 0.01 / (165.0f / 60.0f), 0.0);
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
        draw_force(renderer, visual_force);
    };

    virtual auto onGUI() -> void override
    {
        ImGui::Checkbox("Play", &play);
    };
    virtual ~RigidSceneSingleStep() override = default;
};
