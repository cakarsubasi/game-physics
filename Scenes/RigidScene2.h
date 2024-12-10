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
    f64 time_user = 0.0;
    f32 time_user_timeout = 2.0;

    glm::mat4 cameraMatrix = glm::mat4(1);
    glm::mat4 projectionMatrix = glm::mat4(1);
    glm::vec3 fwd = glm::vec3(1, 0, 0);
    glm::vec3 right = glm::vec3(0, 1, 0);
    glm::vec3 up = glm::vec3(0, 0, 1);
    f32 fov = 0;
    glm::vec3 camera_pos = vec3 {0, 0, 0};
    glm::vec2 wh = glm::vec2(1920.0, 1080.0);

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

        initial_force = 
            Force{
                vec3{0.3f, 0.5f, 0.25f},
                vec3{1.0f, 1.0f, 0.0f},
            };
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
            time_user += time_step;
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
        if (time_user > time_user_timeout) {
            user_force = Force {
                vec3{0.0f},
                vec3{0.0f},
            };
        }
    };

    virtual auto onDraw(Renderer &renderer) -> void override
    {
        cameraMatrix = renderer.camera.viewMatrix;
        wh.x = renderer.camera.width;
        wh.y = renderer.camera.height;
        camera_pos = renderer.camera.position;
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
        ImGui::SliderFloat("User force timeout", &time_user_timeout, 0.001, 20.0);

        if (ImGui::IsMouseReleased(ImGuiMouseButton_Right))
        {
            time_user = 0.0f;
            auto current_delta = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right);
            auto current_pos = ImGui::GetMousePos();
            glm::vec2 drag;
            glm::vec2 pos;
            
            drag.x = current_delta.x; // - current_pos;
            drag.y = current_delta.y; // - current_pos;
            pos.x = current_pos.x;
            pos.y = current_pos.y;
            // auto drag = ImGui::GetMouseDragDelta(1);
            if (true)
            {
                auto dx = drag.x * right;
                auto dy = -drag.y * up;
                auto x = pos.x / wh.x * 2.0f - 1.0f;
                auto y = -pos.y / wh.y * 2.0f + 1.0f;
                // probably need aspect ratio here but forget it
                //std::cout << "right: " << right << "\n";
                //std::cout << "up:    " << up << "\n";
                //std::cout << "fwd:   " << fwd << "\n";
                //std::cout << "x     :" << x << "\n";
                //std::cout << "y     :" << y << "\n";
                //std::cout << "fov   :" << fov << "\n";
                //std::cout << "pos   :" << camera_pos << "\n";
                //std::cout << "ray:  "  << ray << "\n";
                auto d = glm::length(camera_pos);
                user_force.point = camera_pos - fwd * d + (x * right * d * 0.75f) + (y * up * d * 0.4f);
                user_force.strength = (dx + dy) / 600.0f;
            }
        } else {
            //user_force.strength *= 0.9;
        }
    };
    virtual ~RigidScene2() override = default;
};
