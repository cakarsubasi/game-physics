#include "glm_print.h"

/// @brief printing for glm::vec3
/// @param os
/// @param xyz
/// @return
auto operator<<(std::ostream &os, const glm::vec3 &xyz) -> std::ostream &
{
    os << "("
       << xyz.x << ", "
       << xyz.y << ", "
       << xyz.z << ")";
    return os;
}

/// @brief printing for glm::vec4
/// @param os
/// @param xyz
/// @return
auto operator<<(std::ostream &os, const glm::vec4 &xyzw) -> std::ostream &
{
    os << "("
       << xyzw.x << ", "
       << xyzw.y << ", "
       << xyzw.z << ", "
       << xyzw.w << ")";
    return os;
}

/// @brief printing for glm::mat3x3
/// @param os 
/// @param mat 
/// @return 
auto operator<<(std::ostream &os, const glm::mat3x3 &mat) -> std::ostream &
{
    os 
    << "[ " << mat[0][0] << ", " << mat[0][1] << ", " << mat[0][2] << " ]\n"
    << "[ " << mat[1][0] << ", " << mat[1][1] << ", " << mat[1][2] << " ]\n"
    << "[ " << mat[2][0] << ", " << mat[2][1] << ", " << mat[2][2] << " ]";
    return os;
}

auto operator<<(std::ostream &os, const glm::quat &q) -> std::ostream &
{
    float w = q.w;
    os << "w: " << q.w << ", v:" << glm::vec3 { q.x, q.y, q.z };
    return os;
}
