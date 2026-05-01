#pragma once

#include <glm/vec3.hpp>

#include <cstdint>
#include <vector>

namespace finalLab::physics {

struct RigidBody;
class  SpatialIndex;

enum class CombineMode : uint8_t { TruncatedSum, PrioritisedDither };

struct FlockParams {
    bool        enabled            = true;
    float       max_speed           = 6.0f;
    float       max_force           = 12.0f;
    float       neighbour_radius    = 4.0f;
    float       avoidance_radius    = 3.0f;
    float       cohesion_weight     = 1.0f;
    float       alignment_weight    = 1.2f;
    float       separation_weight   = 1.8f;
    float       avoidance_weight    = 3.0f;
    CombineMode combine_mode        = CombineMode::TruncatedSum;
};

glm::vec3 compute_boid_steering(int self_idx,
                                const std::vector<RigidBody>& bodies,
                                const FlockParams& params,
                                const SpatialIndex* spatial = nullptr);

} // namespace finalLab::physics
