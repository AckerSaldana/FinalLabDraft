#pragma once

#include "scene/scene_types.hpp"

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

namespace finalLab::scene {

struct AnimatedPose {
    glm::vec3 position{0.0f};
    glm::quat orientation{1.0f, 0.0f, 0.0f, 0.0f};
};

AnimatedPose sample_animated(const AnimatedObject& anim, float t);

} // namespace finalLab::scene
