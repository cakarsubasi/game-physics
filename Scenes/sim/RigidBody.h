#pragma once

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <vector>

#include "glm_print.h"
#include "Renderer.h"
#include "../CollisionDetection.h"

using vec3 = glm::vec3;
using f32 = float;
using mat3x3 = glm::mat3x3;
using mat4x4 = glm::mat4x4;
using Quaternion = glm::quat;
using time_step_t = float;

struct Force
{
    vec3 point;
    vec3 strength;
};

inline auto quaternion_to_rotation(Quaternion r) -> mat3x3
{
    return glm::toMat3(r);
}

auto box_inertia0(vec3 extent, float mass) -> mat3x3;

struct RigidBody
{
    vec3 extent;

    vec3 center_of_mass; // x_cm
    vec3 velocity_lin;   // v_cm

    Quaternion orientation; // r
    vec3 velocity_ang;      // w

    /// M
    f32 mass;
    /// I_0^-1
    mat3x3 inertia_0_inv; 
    /// Rot_r I_0^-1 Rot_r^T
    mat3x3 inertia_inv;   
    /// q - cleared
    vec3 torque;         
    /// L
    vec3 angular_moment;

    bool visible;
    bool fixed;

    RigidBody() = delete;

    RigidBody(vec3 extent,
              vec3 center_of_mass,
              vec3 linear_velocity,
              Quaternion orientation,
              vec3 velocity_rot,
              f32 mass) : extent{extent},
                          center_of_mass{center_of_mass},
                          velocity_lin{linear_velocity},
                          orientation{orientation},
                          velocity_ang{velocity_rot},
                          mass{mass},
                          inertia_0_inv{glm::inverse(box_inertia0(extent, mass))},
                          angular_moment{vec3{0.0f}},
                          visible {true},
                          fixed {false}
    {
    }

    static auto new_still(vec3 extent, vec3 center_of_mass, Quaternion orientation, f32 mass) -> RigidBody
    {
        auto zero = vec3{0.0f};
        return RigidBody{extent, center_of_mass, zero, orientation, zero, mass};
    }

    auto inline clear_torque() -> void
    {
        torque = vec3{0.0f};
    }

    auto inline add_torque(Force const &force) -> void
    {
        vec3 x_i = force.point - center_of_mass;
        torque += glm::cross(x_i, force.strength);
    }

    auto inline update_linear(vec3 total_force, time_step_t time_step) -> void { 
        center_of_mass += time_step * velocity_lin;
        velocity_lin += time_step * total_force / mass;
        if (fixed) {
            velocity_lin = vec3 {0.0f};
        }
    }

    auto inline update_r(time_step_t time_step) -> void
    {
        Quaternion inter = Quaternion{};
        inter.w = 0.0f;
        inter.x = velocity_ang.x;
        inter.y = velocity_ang.y;
        inter.z = velocity_ang.z;
        inter = inter * orientation;
        orientation += (inter * 0.5f * time_step);
        orientation = glm::normalize(orientation);
    }

    auto inline update_L(time_step_t time_step) -> void
    {
        angular_moment += time_step * torque;
    }

    auto inline update_I() -> void
    {
        mat3x3 rot = quaternion_to_rotation(orientation);
        mat3x3 rot_transpose = glm::transpose(rot);
        inertia_inv = rot * inertia_0_inv * rot_transpose;
    }

    auto inline update_w() -> void
    {
        velocity_ang = inertia_inv * angular_moment;
        if (fixed) {
            velocity_ang = vec3 { 0.0f };
        }
    }

    // convert position in local coordinates to global coordinates
    auto inline position_of(vec3 local) -> vec3
    {
        return center_of_mass + glm::rotate(orientation, local);
    }

    // get the global velocity of given local position
    auto inline velocity_of(vec3 local) const -> vec3
    {
        return velocity_lin + glm::cross(velocity_ang, local);
    }
};

auto euler_one_step(std::vector<RigidBody> &bodies, std::vector<Force> const &forces, time_step_t time_step, f32 gravity) -> void;

auto euler_one_step_collisions(std::vector<RigidBody> &bodies, std::vector<Force> const &forces, time_step_t time_step, f32 gravity) -> void;

auto draw_rigidbody(Renderer &renderer, RigidBody const &body) -> void;

auto draw_force(Renderer &renderer, Force const &force) -> void;