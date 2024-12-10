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
    Force initial_force;
    Force user_force;
    time_step_t time_step;
    f64 time_cumulative;

    glm::mat4 cameraMatrix = glm::mat4(1);
    glm::vec3 fwd = glm::vec3(1, 0, 0);
    glm::vec3 right = glm::vec3(0, 1, 0);
    glm::vec3 up = glm::vec3(0, 0, 1);

    glm::vec2 mouse_pos;

    bool play = false;

    auto initialize_values() -> void
    {
        auto const extent = vec3{1.0f, 0.6f, 0.5f};
        auto const x_cm = vec3{0.0f};

        auto const orientation = Quaternion{vec3{0.0f, 0.0f, 0.5f * glm::pi<f32>()}};

        f32 const mass = 2.0f;

        forces = {};
        rigid_bodies = {
            RigidBody::new_still(extent, x_cm, orientation, mass)};

        time_step = 0.01f;
        time_cumulative = 0.0;
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
            //std::cout << "user force:" << user_force.strength << "\n";
            forces = {initial_force, user_force};
            euler_one_step(rigid_bodies, forces, time_step, 0.0);
        }

        if (time_cumulative > 2.0)
        {
            initial_force = Force{
                vec3{0.0f},
                vec3{0.0f},
            };
        }
    };

    virtual auto onDraw(Renderer &renderer) -> void override
    {
        cameraMatrix = renderer.camera.viewMatrix;
        fwd = inverse(cameraMatrix) * glm::vec4(0, 0, 1, 0);
        right = inverse(cameraMatrix) * glm::vec4(1, 0, 0, 0);
        up = inverse(cameraMatrix) * glm::vec4(0, 1, 0, 0);

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

        if (ImGui::IsMouseReleased(ImGuiMouseButton_Left))
        {
            auto current_pos = ImGui::GetMouseDragDelta(ImGuiMouseButton_Left);
            glm::vec2 drag;
            drag.x = current_pos.x; // - current_pos;
            drag.y = current_pos.y; // - current_pos;
            std::cout << drag.x << "\n";
            // auto drag = ImGui::GetMouseDragDelta(1);
            if (true)
            {
                auto dx = drag.x * right;
                auto dy = -drag.y * up;
                user_force.point = vec3{0.0f, 0.0f, 0.0f};
                user_force.strength += (dx + dy) / 1000.0f;
            }
        } else {
            //user_force.strength *= 0.9;
        }
    };
    virtual ~RigidScene2() override = default;
};
