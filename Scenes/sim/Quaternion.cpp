#include "Quaternion.h"

inline auto Quaternion::w() const -> float
{
    return inner[0];
}

inline auto Quaternion::v() const -> vec3
{
    return vec3{inner[1], inner[2], inner[3]};
}

inline auto Quaternion::norm() -> float
{
    return glm::length(inner);
}

auto Quaternion::operator=(Quaternion const& other) -> Quaternion &
{
    inner = other.inner;
    return *this;
}

auto Quaternion::operator+=(Quaternion const& rhs) -> Quaternion &
{
    inner += rhs.inner;
    return *this;
}

auto Quaternion::operator*=(Quaternion const& rhs) -> Quaternion &
{
    float s1 = w();
    float s2 = rhs.w();
    vec3 v1 = v();
    vec3 v2 = rhs.v();
    inner =
        vec4{
            s1 * s2 - glm::dot(v1, v2),
            s1 * v2 + s2 * v1 + glm::cross(v1, v2)};
    return *this;
}

auto Quaternion::operator*=(float const& rhs) -> Quaternion &
{
    inner *= rhs;
    return *this;
}