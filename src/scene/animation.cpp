#include "scene/animation.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>

namespace finalLab::scene {

namespace {

float ease(float u, EasingType e) {
    if (e == EasingType::Smoothstep) return u * u * (3.0f - 2.0f * u);
    return u;
}

AnimatedPose blend(const Waypoint& a, const Waypoint& b, float u) {
    AnimatedPose p;
    p.position    = glm::mix(a.position, b.position, u);
    p.orientation = glm::slerp(a.orientation, b.orientation, u);
    return p;
}

} // namespace

AnimatedPose sample_animated(const AnimatedObject& anim, float t) {
    if (anim.waypoints.empty()) return {};
    if (anim.waypoints.size() == 1) {
        AnimatedPose p;
        p.position    = anim.waypoints[0].position;
        p.orientation = anim.waypoints[0].orientation;
        return p;
    }

    const float last_time = anim.waypoints.back().time;
    float local_t = t;

    switch (anim.path_mode) {
        case PathMode::Stop:
            local_t = std::min(t, last_time);
            break;
        case PathMode::Reverse: {
            float period = 2.0f * last_time;
            if (period <= 0.0f) { local_t = 0.0f; break; }
            local_t = std::fmod(std::max(t, 0.0f), period);
            if (local_t > last_time) local_t = period - local_t;
            break;
        }
        case PathMode::Loop: {
            float period = (anim.total_duration > last_time) ? anim.total_duration : last_time;
            if (period <= 0.0f) { local_t = 0.0f; break; }
            local_t = std::fmod(std::max(t, 0.0f), period);
            break;
        }
    }

    if (anim.path_mode == PathMode::Loop && local_t >= last_time) {
        const Waypoint& a = anim.waypoints.back();
        const Waypoint& b = anim.waypoints.front();
        float seg_dur = anim.total_duration - last_time;
        float u = (seg_dur > 1e-6f) ? (local_t - last_time) / seg_dur : 0.0f;
        return blend(a, b, ease(u, anim.easing));
    }

    for (size_t i = 0; i + 1 < anim.waypoints.size(); ++i) {
        const Waypoint& a = anim.waypoints[i];
        const Waypoint& b = anim.waypoints[i + 1];
        if (local_t >= a.time && local_t <= b.time) {
            float seg_dur = b.time - a.time;
            float u = (seg_dur > 1e-6f) ? (local_t - a.time) / seg_dur : 0.0f;
            return blend(a, b, ease(u, anim.easing));
        }
    }

    AnimatedPose p;
    p.position    = anim.waypoints.back().position;
    p.orientation = anim.waypoints.back().orientation;
    return p;
}

} // namespace finalLab::scene
