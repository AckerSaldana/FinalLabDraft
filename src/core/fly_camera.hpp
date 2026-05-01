#pragma once

#include <glm/gtc/quaternion.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

struct GLFWwindow;

namespace finalLab::core {

class FlyCamera {
public:
    void seed(const glm::vec3& position, const glm::quat& orientation);
    void update(GLFWwindow* window, float dt);

    glm::mat4  view_matrix()  const;
    glm::vec3  position()     const { return position_; }
    float      yaw_rad()      const { return yaw_rad_; }
    float      pitch_rad()    const { return pitch_rad_; }

    float move_speed         = 8.0f;
    float fast_multiplier    = 4.0f;
    float mouse_sensitivity  = 0.0025f;

private:
    glm::vec3 forward_vector() const;

    glm::vec3 position_    = {0.0f, 2.0f, 5.0f};
    float     yaw_rad_     = 0.0f;
    float     pitch_rad_   = 0.0f;

    bool      looking_     = false;
    double    last_mx_     = 0.0;
    double    last_my_     = 0.0;
};

} // namespace finalLab::core
