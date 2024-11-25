#pragma once
#include <iostream>
#include <glm/glm.hpp>

auto operator<<(std::ostream &os, const glm::vec3 &xyz) -> std::ostream &;
