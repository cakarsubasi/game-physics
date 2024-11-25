#include "glm_print.h"

/// @brief printing for glm::vec3
/// @param os
/// @param xyz
/// @return
auto operator<<(std::ostream &os, const glm::vec3 &xyz) -> std::ostream &
{
    os << "("
       << xyz.x << ","
       << xyz.y << ","
       << xyz.z << ")";
    return os;
}