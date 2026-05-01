#include "sim/sim_runtime.hpp"

#include "scene/animation.hpp"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <windows.h>

#include <algorithm>
#include <chrono>
#include <cstring>

namespace finalLab::sim {

namespace {

glm::mat4 shape_local_scale_for(const scene::Shape& s) {
    using SK = scene::ShapeKind;
    switch (s.kind) {
        case SK::Sphere:   return glm::scale(glm::mat4(1.0f), glm::vec3(s.sphere.radius));
        case SK::Cuboid:   return glm::scale(glm::mat4(1.0f), s.cuboid.size);
        case SK::Cylinder: return glm::scale(glm::mat4(1.0f),
                                glm::vec3(s.cylinder.radius, s.cylinder.height, s.cylinder.radius));
        case SK::Capsule:  return glm::scale(glm::mat4(1.0f),
                                glm::vec3(s.capsule.radius,
                                          s.capsule.height * 0.5f + s.capsule.radius,
                                          s.capsule.radius));
        case SK::Plane:    return glm::mat4(1.0f);
    }
    return glm::mat4(1.0f);
}

float density_for(const std::string& mat, const scene::Scene& s) {
    for (const auto& m : s.materials) if (m.name == mat) return m.density;
    return 1000.0f;
}

// scene::Object → wire-friendly SpawnEntry, used by the spawner-owner peer.
net::SpawnEntry make_spawn_entry(const scene::Object& o, uint32_t canonical_id) {
    net::SpawnEntry e{};
    e.body_id = canonical_id;

    using SK = scene::ShapeKind;
    using BK = scene::BehaviourKind;

    if (o.behaviour.kind == BK::Boid) {
        e.behaviour_kind = static_cast<uint8_t>(net::SpawnBehaviourKind::Boid);
        e.owner          = static_cast<uint8_t>(o.behaviour.boid.owner);
        e.lvx = o.behaviour.boid.initial_linear_velocity.x;
        e.lvy = o.behaviour.boid.initial_linear_velocity.y;
        e.lvz = o.behaviour.boid.initial_linear_velocity.z;
        e.avx = o.behaviour.boid.initial_angular_velocity_rad.x;
        e.avy = o.behaviour.boid.initial_angular_velocity_rad.y;
        e.avz = o.behaviour.boid.initial_angular_velocity_rad.z;
    } else {
        e.behaviour_kind = static_cast<uint8_t>(net::SpawnBehaviourKind::Simulated);
        e.owner          = static_cast<uint8_t>(o.behaviour.sim.owner);
        e.lvx = o.behaviour.sim.initial_linear_velocity.x;
        e.lvy = o.behaviour.sim.initial_linear_velocity.y;
        e.lvz = o.behaviour.sim.initial_linear_velocity.z;
        e.avx = o.behaviour.sim.initial_angular_velocity_rad.x;
        e.avy = o.behaviour.sim.initial_angular_velocity_rad.y;
        e.avz = o.behaviour.sim.initial_angular_velocity_rad.z;
    }

    e.px = o.transform.position.x; e.py = o.transform.position.y; e.pz = o.transform.position.z;
    e.qw = o.transform.orientation.w; e.qx = o.transform.orientation.x;
    e.qy = o.transform.orientation.y; e.qz = o.transform.orientation.z;
    e.sx = o.transform.scale.x; e.sy = o.transform.scale.y; e.sz = o.transform.scale.z;

    switch (o.shape.kind) {
        case SK::Sphere:   e.shape_kind = static_cast<uint8_t>(net::SpawnShapeKind::Sphere);
                           e.dim0 = o.shape.sphere.radius; break;
        case SK::Cuboid:   e.shape_kind = static_cast<uint8_t>(net::SpawnShapeKind::Cuboid);
                           e.dim0 = o.shape.cuboid.size.x; e.dim1 = o.shape.cuboid.size.y; e.dim2 = o.shape.cuboid.size.z; break;
        case SK::Cylinder: e.shape_kind = static_cast<uint8_t>(net::SpawnShapeKind::Cylinder);
                           e.dim0 = o.shape.cylinder.radius; e.dim1 = o.shape.cylinder.height; break;
        case SK::Capsule:  e.shape_kind = static_cast<uint8_t>(net::SpawnShapeKind::Capsule);
                           e.dim0 = o.shape.capsule.radius; e.dim1 = o.shape.capsule.height; break;
        case SK::Plane:    e.shape_kind = static_cast<uint8_t>(net::SpawnShapeKind::Plane);
                           e.dim0 = o.shape.plane.normal.x; e.dim1 = o.shape.plane.normal.y; e.dim2 = o.shape.plane.normal.z; break;
    }

    std::memset(e.material, 0, sizeof(e.material));
    std::strncpy(e.material, o.material.c_str(), sizeof(e.material) - 1);
    return e;
}

// Receiver side: SpawnEntry → scene::Object.
scene::Object materialize_spawn(const net::SpawnEntry& e) {
    scene::Object o;
    o.name = "remote-spawn";
    o.transform.position    = { e.px, e.py, e.pz };
    o.transform.orientation = glm::quat(e.qw, e.qx, e.qy, e.qz);
    o.transform.scale       = { e.sx, e.sy, e.sz };
    size_t mlen = 0;
    while (mlen < sizeof(e.material) && e.material[mlen] != '\0') ++mlen;
    o.material         = std::string(e.material, mlen);
    o.collision_type   = scene::CollisionType::Solid;

    using SK = scene::ShapeKind;
    switch (static_cast<net::SpawnShapeKind>(e.shape_kind)) {
        case net::SpawnShapeKind::Sphere:
            o.shape.kind = SK::Sphere;
            o.shape.sphere.radius = e.dim0;
            break;
        case net::SpawnShapeKind::Cuboid:
            o.shape.kind = SK::Cuboid;
            o.shape.cuboid.size = { e.dim0, e.dim1, e.dim2 };
            break;
        case net::SpawnShapeKind::Cylinder:
            o.shape.kind = SK::Cylinder;
            o.shape.cylinder.radius = e.dim0;
            o.shape.cylinder.height = e.dim1;
            break;
        case net::SpawnShapeKind::Capsule:
            o.shape.kind = SK::Capsule;
            o.shape.capsule.radius = e.dim0;
            o.shape.capsule.height = e.dim1;
            break;
        case net::SpawnShapeKind::Plane:
            o.shape.kind = SK::Plane;
            o.shape.plane.normal = { e.dim0, e.dim1, e.dim2 };
            break;
    }

    glm::vec3 lin{ e.lvx, e.lvy, e.lvz };
    glm::vec3 ang{ e.avx, e.avy, e.avz };
    auto owner = static_cast<scene::OwnerId>(e.owner);
    if (static_cast<net::SpawnBehaviourKind>(e.behaviour_kind) == net::SpawnBehaviourKind::Boid) {
        o.behaviour.kind = scene::BehaviourKind::Boid;
        o.behaviour.boid.initial_linear_velocity      = lin;
        o.behaviour.boid.initial_angular_velocity_rad = ang;
        o.behaviour.boid.owner = owner;
    } else {
        o.behaviour.kind = scene::BehaviourKind::Simulated;
        o.behaviour.sim.initial_linear_velocity       = lin;
        o.behaviour.sim.initial_angular_velocity_rad  = ang;
        o.behaviour.sim.owner = owner;
    }
    return o;
}

} // namespace

uint64_t cores_to_mask(int first_one_indexed, int last_one_indexed) {
    uint64_t mask = 0;
    for (int c = first_one_indexed; c <= last_one_indexed; ++c) {
        if (c >= 1 && c <= 64) mask |= (uint64_t{1} << (c - 1));
    }
    return mask;
}

SimRuntime::SimRuntime() = default;

SimRuntime::~SimRuntime() {
    stop();
}

void SimRuntime::attach_network(net::NetworkRuntime* nr) { net_ = nr; }
void SimRuntime::set_my_peer_id(int peer_id)             { my_peer_id_ = peer_id; world_.my_peer_id = peer_id; }

void SimRuntime::start(uint64_t affinity_mask) {
    if (running_.load()) return;
    affinity_mask_ = affinity_mask;
    running_.store(true);
    thread_ = std::thread([this] { thread_main(); });
}

void SimRuntime::stop() {
    if (!running_.load()) return;
    running_.store(false);
    if (thread_.joinable()) thread_.join();
}

void SimRuntime::load_scene(scene::Scene scene_copy) {
    std::scoped_lock lock(control_mutex_);
    pending_scene_ = std::move(scene_copy);
    scene_pending_ = true;
}

void SimRuntime::set_paused(bool p)               { paused_.store(p); }
void SimRuntime::single_step() {
    std::scoped_lock lock(control_mutex_);
    single_step_pending_ = true;
}
void SimRuntime::reset() {
    std::scoped_lock lock(control_mutex_);
    reset_pending_ = true;
}
void SimRuntime::set_gravity_on(bool on)          { gravity_on_.store(on); }
void SimRuntime::set_gravity(glm::vec3 g) {
    std::scoped_lock lock(control_mutex_);
    pending_gravity_ = g;
    gravity_dirty_   = true;
}
void SimRuntime::set_solver_iterations(int it) {
    std::scoped_lock lock(control_mutex_);
    pending_solver_iters_ = std::clamp(it, 1, 60);
    solver_dirty_ = true;
}
void SimRuntime::set_position_correction(float v) {
    std::scoped_lock lock(control_mutex_);
    pending_position_correction_ = std::clamp(v, 0.0f, 1.0f);
    tuning_dirty_ = true;
}
void SimRuntime::set_penetration_slop(float v) {
    std::scoped_lock lock(control_mutex_);
    pending_penetration_slop_ = std::clamp(v, 0.0f, 0.05f);
    tuning_dirty_ = true;
}
void SimRuntime::set_sim_hz(int hz) {
    sim_hz_target_.store(std::clamp(hz, 30, 4000));
}
void SimRuntime::set_flock_params(physics::FlockParams params) {
    std::scoped_lock lock(control_mutex_);
    pending_flock_params_ = params;
    flock_dirty_          = true;
}
void SimRuntime::set_net_smoothing(bool enabled, float rate) {
    std::scoped_lock lock(control_mutex_);
    pending_net_smoothing_enabled_ = enabled;
    pending_net_correction_rate_   = rate;
    net_smooth_dirty_              = true;
}
void SimRuntime::set_spatial_mode(physics::SpatialMode mode) {
    std::scoped_lock lock(control_mutex_);
    pending_spatial_mode_ = mode;
    spatial_dirty_        = true;
}

void SimRuntime::set_boid_velocities(std::vector<BoidVelocityOverride> overrides) {
    std::scoped_lock lock(control_mutex_);
    // Most-recent wins — replace any pending overrides from a prior render frame.
    pending_boid_overrides_ = std::move(overrides);
}

void SimRuntime::edit_body(uint32_t body_id,
                           glm::vec3 position,
                           glm::quat orientation,
                           glm::vec3 linear_velocity,
                           glm::vec3 angular_velocity_rad,
                           bool broadcast) {
    std::scoped_lock lock(control_mutex_);
    PendingBodyEdit e;
    e.body_id              = body_id;
    e.position             = position;
    e.orientation          = orientation;
    e.linear_velocity      = linear_velocity;
    e.angular_velocity_rad = angular_velocity_rad;
    e.broadcast            = broadcast;
    pending_body_edits_.push_back(e);
}

void SimRuntime::drain_snapshot(SnapshotData& out) {
    std::scoped_lock lock(snapshot_mutex_);
    out.bodies                  = snapshot_.bodies;
    out.spawned_since_last      = std::move(snapshot_.spawned_since_last);
    out.scene_version           = snapshot_.scene_version;
    out.total_time              = snapshot_.total_time;
    out.contact_count           = snapshot_.contact_count;
    out.body_count              = snapshot_.body_count;
    out.sim_hz_actual           = snapshot_.sim_hz_actual;
    out.steps_since_last_drain  = snapshot_.steps_since_last_drain;
    snapshot_.spawned_since_last.clear();
    snapshot_.steps_since_last_drain = 0;
}

void SimRuntime::apply_pending() {
    bool do_reset = false, do_step = false, do_scene = false;
    scene::Scene incoming;
    bool grav_dirty = false; glm::vec3 grav{};
    bool solv_dirty = false; int solv = 6;
    bool tune_dirty = false; float pcorr = 0.5f, pslop = 0.001f;
    bool flock_dirty = false; physics::FlockParams flock{};
    bool net_smooth_dirty = false; bool net_smooth_enabled = true; float net_corr_rate = 8.0f;
    bool spatial_dirty = false; physics::SpatialMode spatial_mode = physics::SpatialMode::None;
    std::vector<PendingBodyEdit> body_edits;
    std::vector<BoidVelocityOverride> boid_overrides;

    {
        std::scoped_lock lock(control_mutex_);
        do_reset = std::exchange(reset_pending_, false);
        do_step  = std::exchange(single_step_pending_, false);
        do_scene = std::exchange(scene_pending_, false);
        if (do_scene) incoming = std::move(pending_scene_);
        if ((grav_dirty = std::exchange(gravity_dirty_, false))) grav = pending_gravity_;
        if ((solv_dirty = std::exchange(solver_dirty_, false))) solv = pending_solver_iters_;
        if ((tune_dirty = std::exchange(tuning_dirty_, false))) {
            pcorr = pending_position_correction_;
            pslop = pending_penetration_slop_;
        }
        if ((flock_dirty = std::exchange(flock_dirty_, false))) flock = pending_flock_params_;
        if ((net_smooth_dirty = std::exchange(net_smooth_dirty_, false))) {
            net_smooth_enabled = pending_net_smoothing_enabled_;
            net_corr_rate      = pending_net_correction_rate_;
        }
        if ((spatial_dirty = std::exchange(spatial_dirty_, false))) spatial_mode = pending_spatial_mode_;
        body_edits.swap(pending_body_edits_);
        boid_overrides.swap(pending_boid_overrides_);
    }

    if (do_scene) {
        scene_ = std::move(incoming);
        world_.build_from_scene(scene_);
        spawners_.build(scene_.spawners);
        scene_version_.fetch_add(1);
        std::scoped_lock lock(snapshot_mutex_);
        snapshot_.spawned_since_last.clear();
    }

    if (do_reset) {
        world_.reset_to_initial();
        spawners_.reset();
        scene_.objects.resize(world_.body_count());
        std::scoped_lock lock(snapshot_mutex_);
        snapshot_.spawned_since_last.clear();
    }

    if (grav_dirty)  world_.gravity = grav;
    if (solv_dirty)  world_.solver_iterations = solv;
    if (tune_dirty) {
        world_.position_correction = pcorr;
        world_.penetration_slop    = pslop;
    }
    if (flock_dirty) world_.flock_params = flock;
    if (net_smooth_dirty) {
        world_.net_smoothing_enabled = net_smooth_enabled;
        world_.net_correction_rate   = net_corr_rate;
    }
    if (spatial_dirty) world_.spatial_index.mode = spatial_mode;

    world_.gravity_on = gravity_on_.load();
    world_.paused     = paused_.load();
    if (do_step) world_.request_single_step();

    if (!boid_overrides.empty()) {
        auto& bodies = world_.bodies_mutable();
        for (const auto& bv : boid_overrides) {
            int local = world_.find_local_index(bv.canonical_body_id);
            if (local < 0) continue;
            auto& b = bodies[local];
            if (!b.is_boid) continue;             // safety: only override boids
            // Only apply on the owner so two peers don't fight over the value.
            if (my_peer_id_ != 0 && b.owner_peer_id != my_peer_id_) continue;
            b.linear_velocity = bv.linear_velocity;
        }
    }

    if (!body_edits.empty()) {
        auto& bodies = world_.bodies_mutable();
        for (const auto& e : body_edits) {
            // e.body_id is interpreted as a CANONICAL id (peer-prefixed for
            // spawned bodies, scene-index for pre-loaded). Find the local slot.
            int local = world_.find_local_index(e.body_id);
            if (local < 0) continue;
            auto& b = bodies[local];
            b.position             = e.position;
            b.orientation          = glm::normalize(e.orientation);
            b.linear_velocity      = e.linear_velocity;
            b.angular_velocity_rad = e.angular_velocity_rad;
            // Edits clear any stale net-smoothing target so the body snaps cleanly.
            b.has_net_target        = false;
            b.net_target_position   = e.position;
            b.net_target_orientation = b.orientation;

            if (e.broadcast && net_ != nullptr) {
                net::BodyEditEntry msg{};
                msg.body_id = e.body_id;        // canonical, unchanged
                msg.px = e.position.x; msg.py = e.position.y; msg.pz = e.position.z;
                msg.qw = b.orientation.w; msg.qx = b.orientation.x;
                msg.qy = b.orientation.y; msg.qz = b.orientation.z;
                msg.lvx = e.linear_velocity.x; msg.lvy = e.linear_velocity.y; msg.lvz = e.linear_velocity.z;
                msg.avx = e.angular_velocity_rad.x; msg.avy = e.angular_velocity_rad.y; msg.avz = e.angular_velocity_rad.z;
                net_->broadcast_body_edit(msg);
            }
        }
    }
}

void SimRuntime::publish_snapshot(const std::vector<scene::Object>& spawned_now) {
    std::vector<BodySnapshot> body_snaps;
    body_snaps.reserve(world_.bodies().size());
    for (const auto& b : world_.bodies()) {
        BodySnapshot s;
        s.scene_object_index   = b.scene_object_index;
        s.canonical_body_id    = b.canonical_body_id;
        s.owner_peer_id        = b.owner_peer_id;
        s.position             = b.position;
        s.orientation          = b.orientation;
        s.linear_velocity      = b.linear_velocity;
        s.angular_velocity_rad = b.angular_velocity_rad;
        s.inverse_mass         = b.inverse_mass;
        s.inverse_inertia_body = b.inverse_inertia_body;

        glm::mat4 T = glm::translate(glm::mat4(1.0f), b.position);
        glm::mat4 R = glm::mat4_cast(b.orientation);
        glm::mat4 S = glm::scale(glm::mat4(1.0f), b.effective_scale);
        glm::mat4 L = shape_local_scale_for(b.shape);
        s.model = T * R * S * L;
        body_snaps.push_back(s);
    }

    std::scoped_lock lock(snapshot_mutex_);
    snapshot_.bodies        = std::move(body_snaps);
    snapshot_.scene_version = scene_version_.load();
    snapshot_.total_time    = world_.simulation_time();
    snapshot_.contact_count = world_.last_contact_count();
    snapshot_.body_count    = world_.body_count();
    for (const auto& obj : spawned_now) snapshot_.spawned_since_last.push_back(obj);
    snapshot_.steps_since_last_drain += 1;
    snapshot_.sim_hz_actual = sim_hz_actual_.load();
    snapshot_.spatial_stats = world_.spatial_stats();
    snapshot_.spatial_mode  = world_.spatial_index.mode;
}

void SimRuntime::thread_main() {
    if (affinity_mask_ != 0) {
        SetThreadAffinityMask(GetCurrentThread(), static_cast<DWORD_PTR>(affinity_mask_));
    }

    auto next_tick = std::chrono::steady_clock::now();
    auto last_hz_sample = std::chrono::steady_clock::now();
    int  steps_in_sample = 0;

    while (running_.load()) {
        // Drain inbound control messages BEFORE apply_pending so a peer-driven
        // body edit reaches pending_body_edits_ in time to be applied this tick.
        if (net_) {
            std::vector<net::InboundBodyEdit> remote_edits;
            net_->drain_inbound_body_edits(remote_edits);
            for (const auto& re : remote_edits) {
                glm::vec3 pos { re.entry.px, re.entry.py, re.entry.pz };
                glm::quat ori { re.entry.qw, re.entry.qx, re.entry.qy, re.entry.qz };
                glm::vec3 lv  { re.entry.lvx, re.entry.lvy, re.entry.lvz };
                glm::vec3 av  { re.entry.avx, re.entry.avy, re.entry.avz };
                edit_body(re.entry.body_id, pos, ori, lv, av, /*broadcast=*/false);
            }
        }

        apply_pending();

        int hz = sim_hz_target_.load();
        float fixed_dt = 1.0f / static_cast<float>(hz);

        // Drain inbound spawn broadcasts before stepping/sending — receivers add
        // bodies in arrival order so this peer's local indices match what the
        // broadcaster's id space expects via canonical_body_id lookups.
        std::vector<scene::Object> remote_spawned;
        if (net_) {
            std::vector<net::InboundSpawn> spawns;
            net_->drain_inbound_spawns(spawns);
            for (const auto& s : spawns) {
                if (world_.find_local_index(s.entry.body_id) >= 0) continue;  // dup guard
                scene::Object o = materialize_spawn(s.entry);
                o.mass = density_for(o.material, scene_)
                       * scene::shape_volume(o.shape, o.transform.scale);
                int idx = static_cast<int>(scene_.objects.size());
                scene_.objects.push_back(o);
                world_.add_body_from_object(scene_.objects.back(), idx, s.entry.body_id);
                remote_spawned.push_back(o);
            }
        }

        if (net_) {
            std::vector<net::InboundState> inbound;
            net_->drain_inbound(inbound);
            for (const auto& in : inbound) {
                int local = world_.find_local_index(in.entry.body_id);
                if (local < 0) continue;          // body not yet known to us
                auto& bodies = world_.bodies_mutable();
                auto& b = bodies[local];
                if (b.owner_peer_id != 0 && b.owner_peer_id != my_peer_id_) {
                    b.linear_velocity        = { in.entry.lvx, in.entry.lvy, in.entry.lvz };
                    b.angular_velocity_rad   = { in.entry.avx, in.entry.avy, in.entry.avz };
                    b.net_target_position    = { in.entry.px, in.entry.py, in.entry.pz };
                    b.net_target_orientation = { in.entry.qw, in.entry.qx, in.entry.qy, in.entry.qz };
                    if (!b.has_net_target) {
                        b.position    = b.net_target_position;
                        b.orientation = b.net_target_orientation;
                        b.has_net_target = true;
                    }
                }
            }
        }

        std::vector<scene::Object> new_now;
        if (!world_.paused) {
            world_.step(fixed_dt);
            // Owner-gated spawn: only this peer's spawners produce here.
            spawners_.advance_owned(world_.simulation_time(), new_now, rng_, my_peer_id_);
            for (auto& obj : new_now) {
                obj.mass = density_for(obj.material, scene_)
                         * scene::shape_volume(obj.shape, obj.transform.scale);
                int idx = static_cast<int>(scene_.objects.size());
                // Allocate a canonical id from this peer's namespace (peer<<24).
                // In single-peer mode my_peer_id_==0 → fall back to scene index.
                uint32_t cid = (my_peer_id_ == 0)
                    ? static_cast<uint32_t>(idx)
                    : physics::SpawnerSystem::canonical_id_for(my_peer_id_, ++my_spawn_seq_);
                scene_.objects.push_back(obj);
                world_.add_body_from_object(scene_.objects.back(), idx, cid);

                if (net_ && my_peer_id_ > 0) {
                    net::SpawnEntry se = make_spawn_entry(obj, cid);
                    net_->broadcast_spawn(se);
                }
            }
        } else {
            world_.step(fixed_dt);
        }

        // Append remote-spawned to the new_now list so the renderer learns about
        // them via the snapshot's spawned_since_last channel.
        for (auto& o : remote_spawned) new_now.push_back(std::move(o));

        if (net_ && my_peer_id_ > 0) {
            std::vector<net::StateEntry> out;
            const auto& bodies = world_.bodies();
            for (size_t i = 0; i < bodies.size(); ++i) {
                const auto& b = bodies[i];
                if (b.owner_peer_id != my_peer_id_) continue;
                net::StateEntry se{};
                se.body_id = b.canonical_body_id;
                se.px = b.position.x; se.py = b.position.y; se.pz = b.position.z;
                se.qw = b.orientation.w; se.qx = b.orientation.x;
                se.qy = b.orientation.y; se.qz = b.orientation.z;
                se.lvx = b.linear_velocity.x; se.lvy = b.linear_velocity.y; se.lvz = b.linear_velocity.z;
                se.avx = b.angular_velocity_rad.x; se.avy = b.angular_velocity_rad.y; se.avz = b.angular_velocity_rad.z;
                out.push_back(se);
            }
            if (!out.empty()) net_->queue_outbound(std::move(out));
        }

        publish_snapshot(new_now);

        ++steps_in_sample;
        auto now = std::chrono::steady_clock::now();
        if (now - last_hz_sample >= std::chrono::milliseconds(500)) {
            float secs = std::chrono::duration<float>(now - last_hz_sample).count();
            sim_hz_actual_.store(static_cast<int>(steps_in_sample / secs + 0.5f));
            steps_in_sample = 0;
            last_hz_sample  = now;
        }

        next_tick += std::chrono::microseconds(1'000'000 / hz);
        if (next_tick < now) next_tick = now;
        std::this_thread::sleep_until(next_tick);
    }
}

} // namespace finalLab::sim
