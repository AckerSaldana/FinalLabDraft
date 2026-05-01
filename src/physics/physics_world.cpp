#include "physics/physics_world.hpp"

#include "physics/collision.hpp"
#include "physics/flocking.hpp"
#include "scene/animation.hpp"

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>

namespace finalLab::physics {

namespace {

glm::vec3 inverse_inertia_body_for(const scene::Object& o, float mass) {
    if (mass <= 0.0f) return glm::vec3(0.0f);

    const glm::vec3 s = o.transform.scale;
    glm::vec3 inertia(0.0f);

    switch (o.shape.kind) {
        case scene::ShapeKind::Sphere: {
            float r = o.shape.sphere.radius * std::max({s.x, s.y, s.z});
            float i = 0.4f * mass * r * r;
            inertia = glm::vec3(i);
            break;
        }
        case scene::ShapeKind::Cuboid: {
            glm::vec3 size = o.shape.cuboid.size * s;
            float w2 = size.x * size.x;
            float h2 = size.y * size.y;
            float d2 = size.z * size.z;
            float c  = (1.0f / 12.0f) * mass;
            inertia = { c * (h2 + d2), c * (w2 + d2), c * (w2 + h2) };
            break;
        }
        case scene::ShapeKind::Cylinder: {
            float r = o.shape.cylinder.radius * std::max(s.x, s.z);
            float h = o.shape.cylinder.height * s.y;
            float i_y  = 0.5f * mass * r * r;
            float i_xz = (1.0f / 12.0f) * mass * (3.0f * r * r + h * h);
            inertia = { i_xz, i_y, i_xz };
            break;
        }
        case scene::ShapeKind::Capsule: {
            float r = o.shape.capsule.radius * std::max(s.x, s.z);
            float h = o.shape.capsule.height * s.y + 2.0f * r;
            float i_y  = 0.5f * mass * r * r;
            float i_xz = (1.0f / 12.0f) * mass * (3.0f * r * r + h * h);
            inertia = { i_xz, i_y, i_xz };
            break;
        }
        case scene::ShapeKind::Plane:
            return glm::vec3(0.0f);
    }

    return {
        inertia.x > 0.0f ? 1.0f / inertia.x : 0.0f,
        inertia.y > 0.0f ? 1.0f / inertia.y : 0.0f,
        inertia.z > 0.0f ? 1.0f / inertia.z : 0.0f,
    };
}

} // namespace

void PhysicsWorld::clear() {
    bodies_.clear();
    initial_.clear();
    interactions_.clear();
    animated_.clear();
    total_time_ = 0.0f;
    last_contact_count_ = 0;
}

void PhysicsWorld::build_from_scene(const scene::Scene& scene) {
    clear();
    interactions_ = scene.interactions;

    for (size_t i = 0; i < scene.objects.size(); ++i) {
        const auto& o = scene.objects[i];

        RigidBody b;
        b.position             = o.transform.position;
        b.orientation          = o.transform.orientation;
        b.shape                = o.shape;
        b.effective_scale      = o.transform.scale;
        b.material             = o.material;
        b.collision_type       = o.collision_type;
        b.scene_object_index   = static_cast<int>(i);

        if (o.behaviour.kind == scene::BehaviourKind::Simulated) {
            b.linear_velocity      = o.behaviour.sim.initial_linear_velocity;
            b.angular_velocity_rad = o.behaviour.sim.initial_angular_velocity_rad;
            b.inverse_mass         = (o.mass > 0.0f) ? (1.0f / o.mass) : 0.0f;
            b.inverse_inertia_body = inverse_inertia_body_for(o, o.mass);
            b.owner_peer_id        = static_cast<uint8_t>(static_cast<int>(o.behaviour.sim.owner) + 1);
        } else if (o.behaviour.kind == scene::BehaviourKind::Boid) {
            b.linear_velocity      = o.behaviour.boid.initial_linear_velocity;
            b.angular_velocity_rad = o.behaviour.boid.initial_angular_velocity_rad;
            b.inverse_mass         = (o.mass > 0.0f) ? (1.0f / o.mass) : 0.0f;
            b.inverse_inertia_body = inverse_inertia_body_for(o, o.mass);
            b.owner_peer_id        = static_cast<uint8_t>(static_cast<int>(o.behaviour.boid.owner) + 1);
            b.is_boid              = true;
        } else {
            b.inverse_mass         = 0.0f;
            b.inverse_inertia_body = glm::vec3(0.0f);
            b.owner_peer_id        = 0;
        }
        bodies_.push_back(b);

        if (o.behaviour.kind == scene::BehaviourKind::Animated) {
            AnimatedRecord rec;
            rec.body_index = static_cast<int>(bodies_.size()) - 1;
            rec.anim       = o.behaviour.anim;
            animated_.push_back(std::move(rec));

            if (!o.behaviour.anim.waypoints.empty()) {
                auto pose = scene::sample_animated(o.behaviour.anim, 0.0f);
                bodies_.back().position    = pose.position;
                bodies_.back().orientation = glm::normalize(pose.orientation);
            }
        }
    }
    initial_ = bodies_;
}

void PhysicsWorld::reset_to_initial() {
    bodies_ = initial_;
    total_time_ = 0.0f;
    last_contact_count_ = 0;
}

void PhysicsWorld::add_body_from_object(const scene::Object& o, int scene_object_index) {
    RigidBody b;
    b.position             = o.transform.position;
    b.orientation          = o.transform.orientation;
    b.shape                = o.shape;
    b.effective_scale      = o.transform.scale;
    b.material             = o.material;
    b.collision_type       = o.collision_type;
    b.scene_object_index   = scene_object_index;

    if (o.behaviour.kind == scene::BehaviourKind::Simulated) {
        b.linear_velocity      = o.behaviour.sim.initial_linear_velocity;
        b.angular_velocity_rad = o.behaviour.sim.initial_angular_velocity_rad;
        b.inverse_mass         = (o.mass > 0.0f) ? (1.0f / o.mass) : 0.0f;
        b.inverse_inertia_body = inverse_inertia_body_for(o, o.mass);
        b.owner_peer_id        = static_cast<uint8_t>(static_cast<int>(o.behaviour.sim.owner) + 1);
    } else if (o.behaviour.kind == scene::BehaviourKind::Boid) {
        b.linear_velocity      = o.behaviour.boid.initial_linear_velocity;
        b.angular_velocity_rad = o.behaviour.boid.initial_angular_velocity_rad;
        b.inverse_mass         = (o.mass > 0.0f) ? (1.0f / o.mass) : 0.0f;
        b.inverse_inertia_body = inverse_inertia_body_for(o, o.mass);
        b.owner_peer_id        = static_cast<uint8_t>(static_cast<int>(o.behaviour.boid.owner) + 1);
        b.is_boid              = true;
    }
    bodies_.push_back(b);
}

ResolvedInteraction PhysicsWorld::lookup_interaction(const std::string& a, const std::string& b) const {
    for (const auto& mi : interactions_) {
        if ((mi.material_a == a && mi.material_b == b) ||
            (mi.material_a == b && mi.material_b == a)) {
            return { mi.restitution, mi.static_friction, mi.dynamic_friction };
        }
    }
    return {};
}

void PhysicsWorld::step(float dt) {
    if (paused && !single_step_pending_) return;
    single_step_pending_ = false;
    total_time_ += dt;

    if (flock_params.enabled) {
        if (spatial_index.mode != SpatialMode::None) {
            float cell = std::max(flock_params.neighbour_radius, flock_params.avoidance_radius);
            spatial_index.build(bodies_, cell);
        } else {
            spatial_index.stats_mutable() = {};
        }
        for (size_t i = 0; i < bodies_.size(); ++i) {
            RigidBody& b = bodies_[i];
            if (!b.is_boid) continue;
            if (my_peer_id != 0 && b.owner_peer_id != my_peer_id) continue;
            glm::vec3 accel = compute_boid_steering(static_cast<int>(i), bodies_, flock_params, &spatial_index);
            b.linear_velocity += accel * dt;
            float speed2 = glm::dot(b.linear_velocity, b.linear_velocity);
            if (speed2 > flock_params.max_speed * flock_params.max_speed) {
                b.linear_velocity *= flock_params.max_speed / std::sqrt(speed2);
            }
        }
    }

    for (auto& b : bodies_) {
        if (b.inverse_mass <= 0.0f) continue;
        bool mine = (my_peer_id == 0) || (b.owner_peer_id == 0) || (b.owner_peer_id == my_peer_id);

        if (mine) {
            if (gravity_on) b.linear_velocity += gravity * dt;
            b.position += b.linear_velocity * dt;
            const glm::vec3& w = b.angular_velocity_rad;
            glm::quat omega_q(0.0f, w.x, w.y, w.z);
            b.orientation += 0.5f * dt * (omega_q * b.orientation);
            b.orientation  = glm::normalize(b.orientation);
        } else {
            b.position += b.linear_velocity * dt;
            const glm::vec3& w = b.angular_velocity_rad;
            glm::quat omega_q(0.0f, w.x, w.y, w.z);
            b.orientation += 0.5f * dt * (omega_q * b.orientation);
            b.orientation  = glm::normalize(b.orientation);
            if (net_smoothing_enabled && b.has_net_target) {
                float blend = 1.0f - std::exp(-net_correction_rate * dt);
                b.position    = glm::mix (b.position,    b.net_target_position,    blend);
                b.orientation = glm::slerp(b.orientation, b.net_target_orientation, blend);
            }
        }
    }

    for (const auto& rec : animated_) {
        RigidBody& b = bodies_[rec.body_index];
        glm::vec3 prev_pos = b.position;
        glm::quat prev_orn = b.orientation;

        auto pose = scene::sample_animated(rec.anim, total_time_);
        b.position    = pose.position;
        b.orientation = glm::normalize(pose.orientation);

        if (dt > 1e-6f) {
            b.linear_velocity = (b.position - prev_pos) / dt;
            glm::quat dq = b.orientation * glm::conjugate(prev_orn);
            if (dq.w < 0.0f) dq = -dq;
            glm::vec3 axis(dq.x, dq.y, dq.z);
            float ax_len = glm::length(axis);
            if (ax_len > 1e-6f) {
                float angle = 2.0f * std::atan2(ax_len, dq.w);
                b.angular_velocity_rad = (axis / ax_len) * (angle / dt);
            } else {
                b.angular_velocity_rad = glm::vec3(0.0f);
            }
        }
    }

    detect_contacts(bodies_, contacts_scratch_);
    last_contact_count_ = contacts_scratch_.size();

    for (int it = 0; it < solver_iterations; ++it) {
        for (Contact& c : contacts_scratch_) {
            const auto& ma = bodies_[c.body_a].material;
            const auto& mb = bodies_[c.body_b].material;
            ResolvedInteraction mat = lookup_interaction(ma, mb);
            resolve_contact(c, bodies_, mat, position_correction, penetration_slop, my_peer_id);
        }
    }
}

} // namespace finalLab::physics
