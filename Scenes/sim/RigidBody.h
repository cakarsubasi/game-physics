#pragma once

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

using vec3 = glm::vec3;
using f32 = float;
using mat3x3 = glm::mat3x3;
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

struct RigidBody
{
    vec3 extent;

    vec3 center_of_mass; // x_cm
    vec3 velocity_lin;      // v_cm

    Quaternion orientation; // r
    vec3 velocity_ang;      // w

    f32 mass;             // M
    mat3x3 inertia_0_inv; // I_0^-1
    mat3x3 inertia_inv;   // Rot_r I_0^-1 Rot_r^T

    vec3 torque;       // q - cleared
    vec3 angular_moment; // L

    RigidBody() = delete;

    RigidBody(vec3 extent,
              vec3 center_of_mass,
              vec3 linear_velocity,
              Quaternion orientation,
              vec3 velocity_rot,
              f32 mass,
              mat3x3 inertia_inv) : extent{extent},
                                    center_of_mass{center_of_mass},
                                    velocity_lin{linear_velocity},
                                    orientation{orientation},
                                    velocity_ang{velocity_rot},
                                    mass{mass},
                                    inertia_0_inv{inertia_inv},
                                    angular_moment{vec3{0.0f}}
    {
    }

    static auto new_still(vec3 extent, vec3 center_of_mass, Quaternion orientation, f32 mass, mat3x3 inertia_inv) -> RigidBody
    {
        auto zero = vec3{0.0f};
        return RigidBody{extent, center_of_mass, zero, orientation, zero, mass, inertia_inv};
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

    auto inline update_r(time_step_t time_step) -> void
    {
        Quaternion inter = Quaternion {};
        inter.w = 0.0f;
        inter[1] = velocity_ang[0];
        inter[2] = velocity_ang[1];
        inter[3] = velocity_ang[2];
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
        // TODO: check if this is correct matmul
        inertia_inv = rot * inertia_0_inv * rot_transpose;
    }

    auto inline update_w() -> void
    {
        velocity_ang = inertia_inv * angular_moment;
    }

    // convert position in local coordinates to global coordinates
    auto inline position_of(vec3 local) -> vec3
    {
        return center_of_mass + glm::rotate(orientation, local);
        // return center_of_mass + orientation.rotate(local);
    }

    // get the global velocity of given local position
    auto inline velocity_of(vec3 local) const -> vec3
    {
        return velocity_lin + glm::cross(velocity_ang, local);
    }
};