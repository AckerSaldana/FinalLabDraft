#pragma once

#include <glm/vec3.hpp>

namespace finalLab::physics {

struct Contact {
    int       body_a       = -1;
    int       body_b       = -1;
    glm::vec3 point        {0.0f};
    glm::vec3 normal       {0.0f, 1.0f, 0.0f};
    float     penetration  = 0.0f;
};

} // namespace finalLab::physics
