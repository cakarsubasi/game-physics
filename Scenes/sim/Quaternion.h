#pragma once

#include <glm/glm.hpp>

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

    static auto from_euler(vec3 euler) -> Quaternion
    {
        // TODO
        return Quaternion{};
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
};