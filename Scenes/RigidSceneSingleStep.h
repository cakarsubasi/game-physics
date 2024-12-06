
#pragma once
#include "Scene.h"
#include <array>
#include <iostream>

using vec3 = glm::vec3;
/// @brief  position in local space
using local3 = glm::vec3;
/// @brief  position in global space
using global3 = glm::vec3;
using mat3x3 = glm::mat3x3;
using f32 = float;
using scalar = f32;
using time_step_t = f32;
using quaternion = glm::vec4;


auto quaternion_to_rotation(quaternion r) -> mat3x3 {
    // TODO
    auto id = glm::identity<mat3x3>();
    return id;
}

auto euler_to_quaternion(vec3 euler) -> quaternion {
    // TODO
    return quaternion { 0.0f };
}

struct Force {
    vec3 point;
    vec3 strength;
};

constexpr auto box_inertia(Aabb extent, f32 mass) -> mat3x3 {
    // are we using the correct extent values?
    auto const w = extent.max.x - extent.min.x;
    auto const h = extent.max.y - extent.min.y;
    auto const d = extent.max.z - extent.min.z;
    f32 const c = 1.0f / 12.0f;
    return mat3x3 { 
        vec3 { c * mass * (h*h + d*d), 0.0f, 0.0f },
        vec3 { 0.0f, c * mass * (w*w + h*h), 0.0f },
        vec3 { 0.0f, 0.0f, c * mass * (w*w + d*d) },
     };
}

auto draw_box(Renderer& renderer, Aabb extent, quaternion orientation) -> void {
    // TODO
}

// we are cheating a bit here, since the bounding
// box will be in local space
struct Aabb {
    vec3 min;
    vec3 max;
};

struct RigidBody {
    Aabb extent;

    global3 center_of_mass; // x_cm
    vec3 velocity_lin;      // v_cm

    quaternion orientation; // r
    vec3 velocity_ang;      // w

    f32 mass;               // M
    mat3x3 inertia_0;       // I_0^-1
    mat3x3 inertia;        // Rot_r I_0^-1 Rot_r^T 

    local3 torque;          // q - cleared
    vec3 angular_moment;    // L

    RigidBody() = delete;

    RigidBody(Aabb extent,
              global3 center_of_mass, 
              vec3 linear_velocity,
              quaternion orientation,
              vec3 velocity_rot,
              f32 mass,
              mat3x3 inertia_inv) :
              extent { extent },
              center_of_mass { center_of_mass },
              velocity_lin { linear_velocity },
              orientation { orientation },
              velocity_ang { velocity_rot },
              mass { mass },
              inertia_0 { inertia_inv }
             {}

    static auto new_still(Aabb extent, global3 center_of_mass, quaternion orientation, f32 mass, mat3x3 inertia_inv) -> RigidBody {
        auto zero = vec3 { 0.0f };
        return RigidBody { extent, center_of_mass, zero, orientation, zero, mass, inertia_inv };
    }

    auto inline clear_torque() -> void {
        torque = vec3 { 0.0f };
    }

    auto inline add_torque(Force &const force) -> void {
        vec3 x_i = force.point - center_of_mass;
        torque += glm::cross(x_i, force.strength);
    }

    auto inline update_r(time_step_t time_step) -> void {
        quaternion inter = glm::vec4 { 0.0f, velocity_ang } * orientation;
        orientation += inter * 0.5f * time_step; 
    }

    auto inline update_L(time_step_t time_step) -> void {
        angular_moment += time_step * torque;
    }

    auto inline update_I() -> void {
        mat3x3 rot = quaternion_to_rotation(orientation);
        mat3x3 rot_transpose = glm::transpose(rot);
        // TODO: check if this is correct matmul
        inertia = rot * inertia_0 * rot_transpose;
    }

    auto inline update_w() -> void {
        velocity_ang = inertia * angular_moment;
    }
};


auto total_force(std::vector<Force> &const forces) -> vec3 {
    auto total = vec3 {0.0f};
    for (auto &const force: forces) {
        total += force.strength;
    }
    return total;
}

auto euler_one_step(std::vector<RigidBody> &bodies, std::vector<Force> &const forces, time_step_t time_step, f32 gravity) -> void
{
    vec3 force_pos = total_force(forces);

    // calculate individual torques (q)
    for (auto &body : bodies)
    {
        body.clear_torque();
        for (auto &const force : forces) {
            body.add_torque(force);
        }
    }

    // euler step1
    // calculate x_cm and v_cm

    for (auto &body: bodies) {
        // euler step 1
        body.center_of_mass += time_step * body.velocity_lin;
        body.velocity_lin += time_step * force_pos / body.mass;
        // update variables
        body.update_r(time_step);
        body.update_L(time_step);
        body.update_I();
        body.update_w();
        
        // Euler step 2 is not performed here, but rather
        // when we draw the rigidbody since each point corresponds to 
    }

}

struct RigidSceneSingleStep : public Scene
{
    std::vector<RigidBody> rigid_bodies;
    std::vector<Force> forces;
    time_step_t time_step;

    auto initialize_values() -> void
    {
        auto const extent1 = vec3 { 1.0f, 0.6f, 0.5f };
        auto const extent2 = Aabb { -extent1, extent1 };
        auto const c_o_m = vec3 { 0.0f };
        auto const orientation = quaternion { 0.0f, 0.0f, 0.5f * glm::pi<f32>(), 1.0f };
        f32 const mass = 2.0f;
        auto inertia_inv = glm::inverse(box_inertia(extent2, mass));
        
        rigid_bodies = {
            RigidBody::new_still(extent2, c_o_m, orientation, mass, inertia_inv)
        };

        time_step = 0.5;
        // TODO

    }

    virtual auto init() -> void override
    {
        initialize_values();
    };

    virtual auto simulateStep() -> void override {};

    virtual auto onDraw(Renderer &renderer) -> void override
    {
        renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));
        for (auto const &body: rigid_bodies) {
            // TODO
        }
        for (auto const &force: forces) {
            // TODO
        }
    };
    
    virtual auto onGUI() -> void override {};
    virtual ~RigidSceneSingleStep() override = default;
};