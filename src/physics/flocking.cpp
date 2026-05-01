#include "physics/flocking.hpp"

#include "physics/physics_world.hpp"
#include "physics/spatial_index.hpp"

#include <glm/geometric.hpp>

#include <algorithm>
#include <cmath>

namespace finalLab::physics {

namespace {

glm::vec3 truncate(glm::vec3 v, float max_len) {
    float l2 = glm::dot(v, v);
    if (l2 > max_len * max_len) return v * (max_len / std::sqrt(l2));
    return v;
}

bool accumulate_priority(glm::vec3& running, float& budget, glm::vec3 force) {
    float l = glm::length(force);
    if (l <= 1e-6f) return budget > 0.0f;
    if (l <= budget) {
        running += force;
        budget  -= l;
        return budget > 0.0f;
    }
    running += force * (budget / l);
    budget  = 0.0f;
    return false;
}

} // namespace

glm::vec3 compute_boid_steering(int self_idx,
                                const std::vector<RigidBody>& bodies,
                                const FlockParams& p,
                                const SpatialIndex* spatial) {
    if (!p.enabled) return glm::vec3(0.0f);

    const RigidBody& me = bodies[self_idx];
    const float r_n  = p.neighbour_radius;
    const float r_n2 = r_n * r_n;
    const float r_a  = p.avoidance_radius;
    const float r_a2 = r_a * r_a;

    glm::vec3 cohesion_pos_sum(0.0f);
    glm::vec3 alignment_vel_sum(0.0f);
    glm::vec3 separation_sum(0.0f);
    glm::vec3 avoidance_sum(0.0f);
    int neighbours = 0;

    auto consider = [&](int i) {
        if (i == self_idx) return;
        const RigidBody& o = bodies[i];
        glm::vec3 to_other = o.position - me.position;
        float d2 = glm::dot(to_other, to_other);
        if (o.is_boid) {
            if (d2 > r_n2 || d2 < 1e-8f) return;
            cohesion_pos_sum  += o.position;
            alignment_vel_sum += o.linear_velocity;
            separation_sum    += -to_other / d2;
            ++neighbours;
        } else {
            if (d2 > r_a2 || d2 < 1e-8f) return;
            avoidance_sum     += -to_other / d2;
        }
    };

    if (spatial && spatial->mode != SpatialMode::None) {
        thread_local std::vector<int> nearby;
        nearby.clear();
        float r_query = std::max(r_n, r_a);
        spatial->query(me.position, r_query, nearby);
        for (int i : nearby) consider(i);
    } else {
        for (int i = 0; i < static_cast<int>(bodies.size()); ++i) consider(i);
    }

    glm::vec3 cohesion(0.0f), alignment(0.0f), separation(0.0f);
    if (neighbours > 0) {
        glm::vec3 avg_pos = cohesion_pos_sum / static_cast<float>(neighbours);
        cohesion  = avg_pos - me.position;
        glm::vec3 avg_vel = alignment_vel_sum / static_cast<float>(neighbours);
        alignment = avg_vel - me.linear_velocity;
        separation = separation_sum;
    }
    glm::vec3 avoidance = avoidance_sum;

    glm::vec3 c = cohesion   * p.cohesion_weight;
    glm::vec3 a = alignment  * p.alignment_weight;
    glm::vec3 s = separation * p.separation_weight;
    glm::vec3 v = avoidance  * p.avoidance_weight;

    glm::vec3 total(0.0f);
    if (p.combine_mode == CombineMode::TruncatedSum) {
        total = c + a + s + v;
        total = truncate(total, p.max_force);
    } else {
        float budget = p.max_force;
        if (!accumulate_priority(total, budget, v)) return total;
        if (!accumulate_priority(total, budget, s)) return total;
        if (!accumulate_priority(total, budget, a)) return total;
        accumulate_priority(total, budget, c);
    }
    return total;
}

} // namespace finalLab::physics
