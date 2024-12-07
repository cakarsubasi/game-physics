#pragma once

#include "Scene.h"
// #include "sim/Quaternion.h"
#include "sim/RigidBody.h"
#include "sim/glm_print.h"
//#include <glm/gtx/quaternion.hpp>
#include <array>
#include <iostream>

//#define DEBUG

using vec3 = glm::vec3;
using vec4 = glm::vec4;
/// @brief  position in local space
using local3 = glm::vec3;
/// @brief  position in global space
using global3 = glm::vec3;
using mat3x3 = glm::mat3x3;
using f32 = float;
using time_step_t = f32;
using Quaternion = glm::quat;


auto box_inertia0(vec3 extent, float mass) -> mat3x3
{
    auto id = glm::zero<mat3x3>();
    // z-up need to check if this is correct
    float h = extent.z;
    float w = extent.x;
    float d = extent.y;
    id[0][0] = (h * h + d * d) * mass / 12.0f;
    id[1][1] = (h * h + w * w) * mass / 12.0f;
    id[2][2] = (w * w + d * d) * mass / 12.0f;
    return id;
}


auto draw_rigidbody(Renderer &renderer, RigidBody const &body) -> void
{
    vec3 scale = body.extent;

    vec3 x_cm = body.center_of_mass;
    Quaternion r = body.orientation;
    renderer.drawCube(x_cm, r, scale);
}

auto draw_force(Renderer &renderer, Force const &force) -> void
{
    // TODO
}

auto total_force(std::vector<Force> const &forces) -> vec3
{
    auto total = vec3{0.0f};
    for (auto const &force : forces)
    {
        total += force.strength;
    }
    return total;
}

auto euler_one_step(std::vector<RigidBody> &bodies, std::vector<Force> const &forces, time_step_t time_step, f32 gravity) -> void
{
    vec3 force_pos = total_force(forces);

    // calculate individual torques (q)
    for (auto &body : bodies)
    {
        body.clear_torque();
        for (auto const &force : forces)
        {
            body.add_torque(force);
        }
#ifdef DEBUG
        auto q0 = body.torque; // -0.25 0.25 -0.2
        std::cout << "q0: " << q0 << "\n";
#endif
    }

    for (auto &body : bodies)
    {
#ifdef DEBUG
        auto x_cm = body.center_of_mass; // 0
        auto v_cm = body.velocity_lin;   // 0
        auto r_0 = body.orientation;     // (0.707, 0, 0, 0.707)
        auto L_0 = body.angular_moment;  // 0

        std::cout << "x_cm: " << x_cm << "\n"
                  << "v_cm: " << v_cm << "\n"
                  << "r_0: " << r_0 << "\n"
                  << "L_0: " << L_0 << "\n";
#endif

        // euler step 1
        body.center_of_mass += time_step * body.velocity_lin;
        body.velocity_lin += time_step * force_pos / body.mass;

        // update variables
        body.update_r(time_step);
        body.update_L(time_step);
        body.update_I();
        body.update_w();
        // Euler step 2 is not performed here, but rather
        // when we draw the rigidbody since each point corresponds to a corner of our bbox

#ifdef DEBUG
        auto x_cm1 = body.center_of_mass; // 0
        auto v_cm1 = body.velocity_lin;   // (1, 1, 0)
        auto r_1 = body.orientation;    // same as r0
        auto L_1 = body.angular_moment; // -0.5 0.5 -0.4
        std::cout << "x_cm1: " << x_cm1 << "\n"
                  << "v_cm1: " << v_cm1 << "\n"
                  << "r_1: " << r_1 << "\n"
                  << "L_1: " << L_1 << "\n";
#endif
    }
}

struct RigidSceneSingleStep : public Scene
{
    std::vector<RigidBody> rigid_bodies;
    std::vector<Force> forces;
    time_step_t time_step;

    bool play = false;

    auto initialize_values() -> void
    {
        auto const extent = vec3{1.0f, 0.6f, 0.5f};
        auto const x_cm = vec3{0.0f};

        auto const orientation = Quaternion{vec3{0.0f, 0.0f, 0.5f * glm::pi<f32>()}};

        std::cout << "orientation(r): " << orientation << "\n";

        f32 const mass = 2.0f;
        auto inertia_inv = glm::inverse(box_inertia0(extent, mass));

        rigid_bodies = {
            RigidBody::new_still(extent, x_cm, orientation, mass, inertia_inv)};

        time_step = 2.0f;

        forces = {
            Force{
                vec3{0.3f, 0.5f, 0.25f},
                vec3{1.0f, 1.0f, 0.0f},
            }};
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

    virtual auto simulateStep() -> void override {
        forces.clear();
        if (play) {
            euler_one_step(rigid_bodies, forces, 0.01, 0.0);   
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

    virtual auto onGUI() -> void override {
        ImGui::Checkbox("Play", &play);
    };
    virtual ~RigidSceneSingleStep() override = default;
};
