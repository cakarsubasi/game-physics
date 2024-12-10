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

struct RigidScene2 : public Scene {
  std::vector<RigidBody> rigid_bodies;
  std::vector<Force> forces;
  time_step_t time_step;
  f64 time_cumulative;

  // Add initial force and add user force
  Force initial_force;
  Force user_force;

  // for mouse input we need camera info
  glm::mat4 cameraMatrix = glm::mat4(1);
  glm::vec3 fwd = glm::vec3(1, 0, 0);
  glm::vec3 right = glm::vec3(0, 1, 0);
  glm::vec3 up = glm::vec3(0, 0, 1);

  bool play = false;

  auto initialize_values() -> void {
    auto const extent = vec3{1.0f, 0.6f, 0.5f};
    auto const x_cm = vec3{0.0f};

    auto const orientation =
        Quaternion{vec3{0.0f, 0.0f, 0.5f * glm::pi<f32>()}};

    f32 const mass = 2.0f;

    rigid_bodies = {RigidBody::new_still(extent, x_cm, orientation, mass)};

    time_step = 0.01f;
    time_cumulative = 0.0;

    // Initialize forces
    initial_force = Force{vec3{0.3f, 0.5f, 0.25f}, vec3{1.0f, 1.0f, 0.0f}};
    user_force = Force{vec3{0.0f}, vec3{0.0f}};
    forces = {initial_force, user_force};
  }

  virtual auto init() -> void override { initialize_values(); };

  virtual auto simulateStep() -> void override {
    if (play) {
      time_cumulative += time_step;
      euler_one_step(rigid_bodies, forces, time_step, 0.0);
      if (ImGui::IsMouseReleased(ImGuiMouseButton_Right)) {
        auto drag = ImGui::GetMouseDragDelta(1);
        if (!(drag.x == 0 && drag.y == 0)) {
        }
        auto dx = drag.x * right;
        auto dy = -drag.y * up;
        // user force will always be at the 1th position
        forces.at(1).strength += (dx + dy) * 0.02f;
      }
    }

    if (time_cumulative > 2.0) {
      // if we do forces.clear() here, the vector will have size 0
      // that will trigger index out of bounds on line 75

      forces.at(0).point = vec3{0.0f};
      forces.at(0).strength = vec3{0.0f};
      forces.at(1).strength = vec3{0.0f};
      forces.at(1).point = vec3{0.0f};
      // we need to reset time here, bc if we dont we can only change force for
      // the first 2 seconds
      time_cumulative = 0.0;
    }
  };

  virtual auto onDraw(Renderer &renderer) -> void override {

    // capture camera info
    cameraMatrix = renderer.camera.viewMatrix;
    fwd = inverse(cameraMatrix) * glm::vec4(0, 0, 1, 0);
    right = inverse(cameraMatrix) * glm::vec4(1, 0, 0, 0);
    up = inverse(cameraMatrix) * glm::vec4(0, 1, 0, 0);

    renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));
    for (auto const &body : rigid_bodies) {
      draw_rigidbody(renderer, body);
    }
    for (auto const &force : forces) {
      draw_force(renderer, force);
    }
  };

  virtual auto onGUI() -> void override {
    ImGui::Checkbox("Play", &play);
    ImGui::SliderFloat("Time step", &time_step, 0.001, 0.1);
  };
  virtual ~RigidScene2() override = default;
};
