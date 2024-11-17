#include "Spring.h"

auto Point::clear_force(void) -> void
{
    force = {0.0, 0.0, 0.0};
}

/// @brief printing for Point
/// @param os
/// @param point
/// @return
auto operator<<(std::ostream &os, const Point &point) -> std::ostream &
{
    os << "pos: "
       << point.position << "\n"
       << "vel: "
       << point.velocity << "\n"
       << "acc: "
       << (point.force / point.mass);
    return os;
}

auto Spring::compute_elastic_forces(std::vector<Point> const &points) -> void
{
    auto pos1 = points.at(point1).position;
    auto pos2 = points.at(point2).position;
    currentLength = (pos1 - pos2).length();
    force = -stiffness * (initialLength - currentLength) * (pos1 - pos2) / (currentLength + 1e-9f);
    // Hooke's law
}

inline auto Spring::add_to_end_points(std::vector<Point> &points) -> void
{
    points.at(point1).force = -force;
    points.at(point2).force = force;
}

auto integrate_positions_euler(std::vector<Point> &points, time_step_t time_step) -> void
{
    for (auto &point : points)
    {
        point.position += point.velocity * time_step;
    }
}

auto integrate_velocities_euler(std::vector<Point> &points, time_step_t time_step) -> void
{
    for (auto &point : points)
    {
        point.velocity += point.force * time_step / point.mass;
    }
}

auto integrate_midpoint_1(std::vector<Point> &points, time_step_t time_step) -> void
{
    for (auto &point : points)
    {

        glm::vec3 x_tilde = point.position + point.velocity * time_step * 0.5f;
        glm::vec3 v_tilde = point.velocity + point.force * time_step * 0.5f;
        point.position += time_step * v_tilde;
        point.force = x_tilde;
        point.scratch = v_tilde;
    }
}

auto Spring::compute_elastic_forces_midpoint(std::vector<Point> const &points) -> void
{
    auto pos1 = points.at(point1).force; // x_tilde
    auto pos2 = points.at(point2).force; // x_tilde
    currentLength = (pos1 - pos2).length();
    force = -stiffness * (initialLength - currentLength) * (pos1 - pos2) / (currentLength + 1e-9f);
    // Hooke's law
}

auto integrate_midpoint_2(std::vector<Point> &points, time_step_t time_step) -> void
{
    for (auto &point : points)
    {
        glm::vec3 v_tilde = point.scratch;
        point.velocity += time_step * v_tilde;
    }
}

auto euler_one_step(std::vector<Point> &points, std::vector<Spring> &springs, time_step_t time_step) -> void
{

    for (auto &point : points)
    {
        point.clear_force();
    }

    for (auto &spring : springs)
    {
        spring.compute_elastic_forces(points);
        spring.add_to_end_points(points);
    }

    integrate_positions_euler(points, time_step);
    integrate_velocities_euler(points, time_step);
}

auto midpoint_one_step(std::vector<Point> &points, std::vector<Spring> &springs, time_step_t time_step) -> void
{
    for (auto &point : points)
    {
        point.clear_force();
    }

    for (auto &spring : springs)
    {
        spring.compute_elastic_forces(points);
        spring.add_to_end_points(points);
    }

    integrate_midpoint_1(points, time_step);

    for (auto &spring : springs)
    {
        spring.compute_elastic_forces_midpoint(points);
        spring.add_to_end_points(points);
    }
    integrate_midpoint_2(points, time_step);
}