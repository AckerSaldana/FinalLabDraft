#include "physics/spawner.hpp"

#include <glm/glm.hpp>

#include <cmath>
#include <string>

namespace finalLab::physics {

namespace {

constexpr float kPi = 3.14159265358979323846f;

float sample_float(const scene::FloatRange& r, std::mt19937& rng) {
    if (r.max <= r.min) return r.min;
    std::uniform_real_distribution<float> d(r.min, r.max);
    return d(rng);
}

glm::vec3 sample_vec3(const scene::Vec3Range& r, std::mt19937& rng) {
    auto axis = [&](float lo, float hi) {
        if (hi <= lo) return lo;
        std::uniform_real_distribution<float> d(lo, hi);
        return d(rng);
    };
    return { axis(r.min.x, r.max.x), axis(r.min.y, r.max.y), axis(r.min.z, r.max.z) };
}

glm::vec3 sample_position(const scene::SpawnLocation& loc, std::mt19937& rng) {
    switch (loc.kind) {
        case scene::SpawnLocationKind::Fixed:
            return loc.fixed.transform.position;
        case scene::SpawnLocationKind::RandomBox: {
            scene::Vec3Range r{ loc.box.min, loc.box.max };
            return sample_vec3(r, rng);
        }
        case scene::SpawnLocationKind::RandomSphere: {
            std::uniform_real_distribution<float> u(0.0f, 1.0f);
            float r     = loc.sphere.radius * std::cbrt(u(rng));
            float theta = u(rng) * 2.0f * kPi;
            float phi   = std::acos(1.0f - 2.0f * u(rng));
            float sp = std::sin(phi);
            return loc.sphere.center + glm::vec3(r * sp * std::cos(theta),
                                                 r * std::cos(phi),
                                                 r * sp * std::sin(theta));
        }
    }
    return glm::vec3(0.0f);
}

scene::OwnerId resolve_owner(const scene::Spawner& def, int& sequential_counter) {
    using SO = scene::SpawnerOwnerKind;
    switch (def.base.owner) {
        case SO::One:        return scene::OwnerId::One;
        case SO::Two:        return scene::OwnerId::Two;
        case SO::Three:      return scene::OwnerId::Three;
        case SO::Four:       return scene::OwnerId::Four;
        case SO::Sequential: {
            scene::OwnerId o = static_cast<scene::OwnerId>(sequential_counter % 4);
            sequential_counter = (sequential_counter + 1) % 4;
            return o;
        }
    }
    return scene::OwnerId::One;
}

// Which peer is the *runner* (i.e. responsible for advancing this spawner's
// state and broadcasting spawned objects). For fixed-owner spawners, that's
// the named peer; for Sequential spawners, peer 1 by convention so a single
// peer is always authoritative for the spawn timing.
int runner_peer_for(const scene::Spawner& def) {
    using SO = scene::SpawnerOwnerKind;
    switch (def.base.owner) {
        case SO::One:        return 1;
        case SO::Two:        return 2;
        case SO::Three:      return 3;
        case SO::Four:       return 4;
        case SO::Sequential: return 1;
    }
    return 1;
}

scene::Object make_object(const scene::Spawner& def, int seq, std::mt19937& rng) {
    scene::Object obj;
    obj.name = def.base.name + " #" + std::to_string(seq);

    obj.transform.position    = sample_position(def.base.location, rng);
    obj.transform.orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    obj.transform.scale       = glm::vec3(1.0f);
    obj.material              = def.base.material;
    obj.collision_type        = scene::CollisionType::Solid;

    obj.shape.kind = def.shape_kind;
    switch (def.shape_kind) {
        case scene::ShapeKind::Sphere:
            obj.shape.sphere.radius = sample_float(def.radius_range, rng);
            break;
        case scene::ShapeKind::Cylinder:
            obj.shape.cylinder.radius = sample_float(def.radius_range, rng);
            obj.shape.cylinder.height = sample_float(def.height_range, rng);
            break;
        case scene::ShapeKind::Capsule:
            obj.shape.capsule.radius = sample_float(def.radius_range, rng);
            obj.shape.capsule.height = sample_float(def.height_range, rng);
            break;
        case scene::ShapeKind::Cuboid:
            obj.shape.cuboid.size = sample_vec3(def.size_range, rng);
            break;
        case scene::ShapeKind::Plane:
            obj.shape.plane.normal = glm::vec3(0.0f, 1.0f, 0.0f);
            break;
    }

    glm::vec3 lin = sample_vec3(def.base.linear_velocity, rng);
    glm::vec3 ang = sample_vec3(def.base.angular_velocity_rad, rng);
    if (def.produces_boids) {
        obj.behaviour.kind                                  = scene::BehaviourKind::Boid;
        obj.behaviour.boid.initial_linear_velocity          = lin;
        obj.behaviour.boid.initial_angular_velocity_rad     = ang;
    } else {
        obj.behaviour.kind                                  = scene::BehaviourKind::Simulated;
        obj.behaviour.sim.initial_linear_velocity           = lin;
        obj.behaviour.sim.initial_angular_velocity_rad      = ang;
    }
    return obj;
}

} // namespace

void SpawnerSystem::build(const std::vector<scene::Spawner>& defs) {
    defs_ = defs;
    states_.assign(defs_.size(), {});
}

void SpawnerSystem::clear() {
    defs_.clear();
    states_.clear();
}

void SpawnerSystem::reset() {
    states_.assign(defs_.size(), {});
}

int SpawnerSystem::advance(float current_time, std::vector<scene::Object>& out_objects, std::mt19937& rng) {
    int spawned_total = 0;
    for (size_t i = 0; i < defs_.size(); ++i) {
        const scene::Spawner& def = defs_[i];
        State& s = states_[i];
        if (current_time < def.base.start_time) continue;

        if (def.base.timing.kind == scene::SpawnKind::SingleBurst) {
            if (s.single_burst_done) continue;
            int n = static_cast<int>(def.base.timing.burst.count);
            for (int k = 0; k < n; ++k) {
                scene::Object o = make_object(def, ++s.count, rng);
                scene::OwnerId owner = resolve_owner(def, s.sequential_next);
                if (o.behaviour.kind == scene::BehaviourKind::Boid) o.behaviour.boid.owner = owner;
                else o.behaviour.sim.owner = owner;
                out_objects.push_back(std::move(o));
                ++spawned_total;
            }
            s.single_burst_done = true;
        } else {
            float interval = def.base.timing.repeating.interval > 0.0f ? def.base.timing.repeating.interval : 1.0f;
            uint32_t max_count = def.base.timing.repeating.max_count;

            if (s.next_fire_time < 0.0f) s.next_fire_time = def.base.start_time;

            while (current_time >= s.next_fire_time
                   && (max_count == 0 || static_cast<uint32_t>(s.count) < max_count)) {
                scene::Object o = make_object(def, ++s.count, rng);
                scene::OwnerId owner = resolve_owner(def, s.sequential_next);
                if (o.behaviour.kind == scene::BehaviourKind::Boid) o.behaviour.boid.owner = owner;
                else o.behaviour.sim.owner = owner;
                out_objects.push_back(std::move(o));
                ++spawned_total;
                s.next_fire_time += interval;
            }
        }
    }
    return spawned_total;
}

int SpawnerSystem::advance_owned(float current_time, std::vector<scene::Object>& out_objects,
                                  std::mt19937& rng, int my_peer_id,
                                  std::vector<size_t>* spawner_indices_for_each_object) {
    int spawned_total = 0;
    for (size_t i = 0; i < defs_.size(); ++i) {
        const scene::Spawner& def = defs_[i];
        State& s = states_[i];
        if (current_time < def.base.start_time) continue;

        // Single-peer mode (my_peer_id == 0) → run every spawner; networked
        // mode → only the spawner's runner peer advances + broadcasts.
        if (my_peer_id != 0 && runner_peer_for(def) != my_peer_id) continue;

        if (def.base.timing.kind == scene::SpawnKind::SingleBurst) {
            if (s.single_burst_done) continue;
            int n = static_cast<int>(def.base.timing.burst.count);
            for (int k = 0; k < n; ++k) {
                scene::Object o = make_object(def, ++s.count, rng);
                scene::OwnerId owner = resolve_owner(def, s.sequential_next);
                if (o.behaviour.kind == scene::BehaviourKind::Boid) o.behaviour.boid.owner = owner;
                else o.behaviour.sim.owner = owner;
                out_objects.push_back(std::move(o));
                if (spawner_indices_for_each_object) spawner_indices_for_each_object->push_back(i);
                ++spawned_total;
            }
            s.single_burst_done = true;
        } else {
            float interval = def.base.timing.repeating.interval > 0.0f ? def.base.timing.repeating.interval : 1.0f;
            uint32_t max_count = def.base.timing.repeating.max_count;

            if (s.next_fire_time < 0.0f) s.next_fire_time = def.base.start_time;

            while (current_time >= s.next_fire_time
                   && (max_count == 0 || static_cast<uint32_t>(s.count) < max_count)) {
                scene::Object o = make_object(def, ++s.count, rng);
                scene::OwnerId owner = resolve_owner(def, s.sequential_next);
                if (o.behaviour.kind == scene::BehaviourKind::Boid) o.behaviour.boid.owner = owner;
                else o.behaviour.sim.owner = owner;
                out_objects.push_back(std::move(o));
                if (spawner_indices_for_each_object) spawner_indices_for_each_object->push_back(i);
                ++spawned_total;
                s.next_fire_time += interval;
            }
        }
    }
    return spawned_total;
}

void SpawnerSystem::apply_remote_object(scene::Object o, std::vector<scene::Object>& out) {
    out.push_back(std::move(o));
}

int SpawnerSystem::spawned_count(size_t i) const {
    return (i < states_.size()) ? states_[i].count : 0;
}

float SpawnerSystem::next_fire_time(size_t i) const {
    return (i < states_.size()) ? states_[i].next_fire_time : 0.0f;
}

bool SpawnerSystem::exhausted(size_t i) const {
    if (i >= defs_.size()) return true;
    const auto& d = defs_[i];
    const auto& s = states_[i];
    if (d.base.timing.kind == scene::SpawnKind::SingleBurst) return s.single_burst_done;
    uint32_t mx = d.base.timing.repeating.max_count;
    return mx != 0 && static_cast<uint32_t>(s.count) >= mx;
}

} // namespace finalLab::physics
