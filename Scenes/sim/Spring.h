#pragma once
#include <iostream>
#include <glm/glm.hpp>
#include <vector>

#include "glm_print.h"

using vec3 = glm::vec3;

struct Point
{
    vec3 position;
    vec3 velocity;
    vec3 force;
    vec3 scratch;
    vec3 scratch2;
    float mass;
    float damping;
    bool fixed;

    auto clear_force(void) -> void;
};

auto operator<<(std::ostream &os, const Point &point) -> std::ostream &;

using index_t = size_t;
using time_step_t = float;

struct Spring
{
    index_t point1;
    index_t point2;
    float stiffness;
    float initialLength;
    glm::vec3 force;

    auto compute_elastic_forces(std::vector<Point> const &points) -> void;
    auto compute_elastic_forces_midpoint(std::vector<Point> const &points) -> void;
    auto add_to_end_points(std::vector<Point> &points) -> void;
};

auto integrate_positions_euler(std::vector<Point> &points, time_step_t time_step) -> void;
auto integrate_velocities_euler(std::vector<Point> &points, time_step_t time_step) -> void;

auto integrate_midpoint_1(std::vector<Point> &points, time_step_t time_step) -> void;
auto integrate_midpoint_2(std::vector<Point> &points, time_step_t time_step) -> void;

auto euler_one_step(std::vector<Point> &points, std::vector<Spring> &springs, time_step_t time_step) -> void;
auto midpoint_one_step(std::vector<Point> &points, std::vector<Spring> &springs, time_step_t time_step) -> void;
