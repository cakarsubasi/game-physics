#include "RigidBody.h"
#include "glm_print.h"

// #define DEBUG

auto box_inertia0(vec3 extent, float mass) -> mat3x3
{
    auto id = glm::zero<mat3x3>();
    // z-up need to check if this is correct
    float h = extent.x;
    float w = extent.y;
    float d = extent.z;
    id[0][0] = (h * h + d * d) * mass / 12.0f;
    id[1][1] = (h * h + w * w) * mass / 12.0f;
    id[2][2] = (w * w + d * d) * mass / 12.0f;
    return id;
}

auto euler_one_step(std::vector<RigidBody> &bodies, std::vector<Force> const &forces, time_step_t time_step, f32 gravity) -> void
{
    auto total_force = vec3{0.0f};
    for (auto const &force : forces)
    {
        total_force += force.strength;
    }

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
        body.velocity_lin += time_step * total_force / body.mass;

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
        auto r_1 = body.orientation;      // same as r0
        auto L_1 = body.angular_moment;   // -0.5 0.5 -0.4

        auto I_0 = body.inertia_0_inv;
        auto rot = quaternion_to_rotation(r_1);
        auto inertia_inv = body.inertia_inv;
        auto w = body.velocity_ang;

        std::cout << "x_cm1: " << x_cm1 << "\n"
                  << "v_cm1: " << v_cm1 << "\n"
                  << "r_1: " << r_1 << "\n"
                  << "L_1: " << L_1 << "\n"
                  << "rot: " << rot << "\n"
                  << "I-0: " << I_0 << "\n"
                  << "I-1: " << inertia_inv << "\n"
                  << "w:   " << w << "\n";
#endif
    }

}

auto calculate_impulse(
    vec3 normal,
    vec3 velocity_rel,
    f32 elasticity,
    f32 mass_a,
    f32 mass_b,
    mat3x3 inertia_a,
    mat3x3 inertia_b,
    vec3 x_a,
    vec3 x_b)
    -> f32
{
    f32 top = -(1.0f + elasticity) * glm::dot(velocity_rel, normal);
    vec3 bot1 = glm::cross(inertia_a * glm::cross(x_a, normal), x_a);
    vec3 bot2 = glm::cross(inertia_b * glm::cross(x_b, normal), x_b);
    f32 bot = 1.0f / mass_a + 1.0f / mass_b + glm::dot(bot1 + bot2, normal);
    f32 impulse = top / bot;
    assert(impulse >= 0.0f);
    return impulse;
}

auto rigidbody_to_world_transform(RigidBody const &rb) -> mat4x4
{
    mat4x4 rotationMatrix = glm::toMat4(rb.orientation);
    mat4x4 scaleMatrix = glm::scale(mat4x4(1), rb.extent);
    mat4x4 translationMatrix = glm::translate(mat4x4(1), rb.center_of_mass);
    return translationMatrix * rotationMatrix * scaleMatrix;
}

auto euler_one_step_collisions(std::vector<RigidBody> &bodies, std::vector<Force> const &forces, time_step_t time_step, f32 gravity) -> void
{
    std::vector<mat4x4> transforms;
    for (auto const &rb : bodies)
    {
        transforms.push_back(rigidbody_to_world_transform(rb));
    }

    for (size_t i = 0; i < bodies.size(); i++)
    {
        auto &rb1mat = transforms.at(i);
        for (size_t j = i + 1; j < bodies.size(); j++)
        {
            auto &rb2mat = transforms.at(j);
            CollisionInfo info = collisionTools::checkCollisionSAT(rb1mat, rb2mat);
            if (info.isColliding)
            {
                auto &rb1 = bodies.at(i);
                auto &rb2 = bodies.at(j);
                vec3 velocity_rel = rb1.velocity_lin - rb2.velocity_lin;
                // separating case
                if (glm::dot(velocity_rel, info.normalWorld) > 0.0f) {
                    continue;
                }
                // find the impulse
                vec3 x_a = info.collisionPointWorld - rb1.center_of_mass;
                vec3 x_b = info.collisionPointWorld - rb2.center_of_mass;
                f32 impulse = calculate_impulse(
                    info.normalWorld, 
                    velocity_rel, 
                    0.1, // might wish to change this
                    rb1.mass, 
                    rb2.mass, 
                    rb1.inertia_0_inv, 
                    rb2.inertia_0_inv, 
                    x_a, 
                    x_b);
                // add the impulse in the correct directions
                rb1.velocity_lin += impulse * info.normalWorld / rb1.mass;
                rb2.velocity_lin -= impulse * info.normalWorld / rb2.mass;

                rb1.angular_moment += glm::cross(x_a, impulse * info.normalWorld);
                rb2.angular_moment -= glm::cross(x_b, impulse * info.normalWorld);
            }
        }
    }

    euler_one_step(bodies, forces, time_step, gravity);
}

auto draw_rigidbody(Renderer &renderer, RigidBody const &body) -> void
{
    vec3 scale = body.extent;
    vec3 x_cm = body.center_of_mass;
    Quaternion r = body.orientation;
    renderer.drawCube(x_cm, r, scale);

    // linear velocity
    renderer.drawLine(x_cm, x_cm + body.velocity_lin, vec3{0.2, 0.8, 0.2});
    // angular moment
    renderer.drawLine(x_cm, x_cm + body.angular_moment, vec3{0.2, 0.2, 0.8});
}

auto draw_force(Renderer &renderer, Force const &force) -> void
{
    renderer.drawLine(force.point, force.point + force.strength, vec3{0.8, 0.2, 0.2});
}
