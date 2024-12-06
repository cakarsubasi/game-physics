#pragma once

#include <glm/glm.hpp>

using vec4 = glm::vec4;
using vec3 = glm::vec3;

// new type!
struct Quaternion {
    private:
    vec4 inner;
    public:

    Quaternion() : inner { vec4 {0.0} } {}

    Quaternion(const float elem) : inner { vec4 {elem}} {}

    Quaternion(const vec4 other) : inner { other } {}

    Quaternion(const Quaternion &other) : inner { other.inner } {}

    static auto unit() -> Quaternion {
        // TODO
        return Quaternion {};
    }

    static auto from_euler(vec3 euler) -> Quaternion {
        // TODO
        return Quaternion {};
    } 

    inline auto w() const -> float;

    inline auto v() const -> vec3;

    inline auto norm() -> float;

    auto operator=(Quaternion const& other) -> Quaternion &;

    friend auto operator+(Quaternion lhs, Quaternion const& rhs) -> Quaternion {
        return lhs += rhs;
    }

    auto operator+=(Quaternion const& rhs) -> Quaternion &;

    // Quaternion multiplication
    friend auto operator*(Quaternion lhs, Quaternion const& rhs) -> Quaternion {
        // note, perhaps I should move w to [3] instead of [0]
        return lhs *= rhs;
    }

    auto operator*=(Quaternion const& rhs) -> Quaternion &;

    friend auto operator*(Quaternion lhs, float rhs) -> Quaternion {
        return lhs *= rhs;
    }

    auto operator*=(float const& rhs) -> Quaternion &;
};