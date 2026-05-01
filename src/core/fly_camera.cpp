#include "core/fly_camera.hpp"

#include <GLFW/glfw3.h>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <cmath>

namespace finalLab::core {

namespace {
constexpr float kPitchLimit = 1.48353f; // ~85 degrees
}

void FlyCamera::seed(const glm::vec3& position, const glm::quat& orientation) {
    position_ = position;
    glm::vec3 forward = glm::normalize(orientation * glm::vec3(0.0f, 0.0f, -1.0f));
    pitch_rad_ = std::asin(std::clamp(forward.y, -1.0f, 1.0f));
    yaw_rad_   = std::atan2(-forward.x, -forward.z);
}

glm::vec3 FlyCamera::forward_vector() const {
    float cp = std::cos(pitch_rad_);
    return { -std::sin(yaw_rad_) * cp, std::sin(pitch_rad_), -std::cos(yaw_rad_) * cp };
}

glm::mat4 FlyCamera::view_matrix() const {
    return glm::lookAt(position_, position_ + forward_vector(), glm::vec3(0.0f, 1.0f, 0.0f));
}

void FlyCamera::update(GLFWwindow* window, float dt) {
    bool right_down = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;

    if (right_down && !looking_) {
        glfwGetCursorPos(window, &last_mx_, &last_my_);
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        looking_ = true;
    } else if (!right_down && looking_) {
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        looking_ = false;
    }

    if (!looking_) return;

    double mx, my;
    glfwGetCursorPos(window, &mx, &my);
    double dx = mx - last_mx_;
    double dy = my - last_my_;
    last_mx_ = mx;
    last_my_ = my;

    yaw_rad_   -= static_cast<float>(dx) * mouse_sensitivity;
    pitch_rad_ -= static_cast<float>(dy) * mouse_sensitivity;
    pitch_rad_  = std::clamp(pitch_rad_, -kPitchLimit, kPitchLimit);

    glm::vec3 forward = forward_vector();
    glm::vec3 right   = glm::normalize(glm::cross(forward, glm::vec3(0.0f, 1.0f, 0.0f)));
    glm::vec3 world_up{ 0.0f, 1.0f, 0.0f };

    glm::vec3 vel(0.0f);
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) vel += forward;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) vel -= forward;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) vel += right;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) vel -= right;
    if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) vel += world_up;
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) vel -= world_up;

    if (glm::length(vel) > 1e-4f) {
        float speed = move_speed;
        if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) speed *= fast_multiplier;
        position_ += glm::normalize(vel) * speed * dt;
    }
}

} // namespace finalLab::core
