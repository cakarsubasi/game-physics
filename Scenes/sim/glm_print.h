#pragma once
#include <iostream>
#include <glm/glm.hpp>

auto operator<<(std::ostream &os, const glm::vec3 &xyz) -> std::ostream &;

auto operator<<(std::ostream &os, const glm::vec4 &xyzw) -> std::ostream &;

auto operator<<(std::ostream &os, const glm::mat3x3 &mat) -> std::ostream &;