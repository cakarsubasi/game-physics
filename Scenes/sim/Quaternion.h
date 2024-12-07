#pragma once

#include <glm/glm.hpp>
#include <iostream>
#include "glm_print.h"

using vec4 = glm::vec4;
using vec3 = glm::vec3;

// new type!
struct Quaternion
{
private:
    vec4 inner;

public:
    Quaternion() : inner{vec4{0.0}} {}

    Quaternion(const float elem) : inner{vec4{elem}} {}

    Quaternion(const vec4 other) : inner{other} {}

    Quaternion(const float s, const vec3 v) : inner{vec4{s, v}} {}

    Quaternion(const Quaternion &other) : inner{other.inner} {}

    static auto from_euler(vec3 angles) -> Quaternion
    {
        float phi1 = angles[0]; // need to ensure radians or degrees
        float c1 = std::cos(phi1 / 2.0f);
        float s1 = std::sin(phi1 / 2.0f);
        auto q1 = Quaternion{c1, vec3{s1, 0.0f, 0.0f}};

        float phi2 = angles[1]; // need to ensure radians or degrees
        float c2 = std::cos(phi2 / 2.0f);
        float s2 = std::sin(phi2 / 2.0f);
        auto q2 = Quaternion{c2, vec3{0.0f, s2, 0.0f}};

        float phi3 = angles[2]; // need to ensure radians or degrees
        float c3 = std::cos(phi3 / 2.0f);
        float s3 = std::sin(phi3 / 2.0f);
        auto q3 = Quaternion{c3, vec3{0.0f, 0.0f, s3}};

        Quaternion rotation = q1 * q2 * q3;

        return rotation.normalize();
    }

    inline auto w() const -> float
    {
        return inner[0];
    }

    inline auto v() const -> vec3
    {
        return vec3{inner[1], inner[2], inner[3]};
    }

    inline auto norm() const -> float
    {
        return glm::length(inner);
    }

    inline auto conj() const -> Quaternion
    {
        return Quaternion{w(), -v()};
    }

    inline auto normalize() const -> Quaternion
    {
        return *this / norm();
    }

    inline auto rotate(vec3 point) -> vec3
    {
        auto pprime = *this * Quaternion{0.0f, point} * (this->conj());
        // TODO: is this correct?
        return pprime.v();
    }

    auto operator=(Quaternion const &other) -> Quaternion &;

    friend auto operator+(Quaternion lhs, Quaternion const &rhs) -> Quaternion
    {
        return lhs += rhs;
    }

    auto operator+=(Quaternion const &rhs) -> Quaternion &;

    // Quaternion multiplication
    friend auto operator*(Quaternion lhs, Quaternion const &rhs) -> Quaternion
    {
        // note, perhaps I should move w to [3] instead of [0]
        return lhs *= rhs;
    }

    auto operator*=(Quaternion const &rhs) -> Quaternion &;

    friend auto operator*(Quaternion lhs, float rhs) -> Quaternion
    {
        return lhs *= rhs;
    }

    auto operator*=(float const &rhs) -> Quaternion &;

    friend auto operator/(Quaternion lhs, float rhs) -> Quaternion
    {
        return lhs /= rhs;
    }

    auto operator/=(float const &rhs) -> Quaternion &;

    friend auto operator<<(std::ostream &os, Quaternion const &q) -> std::ostream &
    {
        os << "w: " << q.w() << ", v:" << q.v();
        return os;
    }
};
