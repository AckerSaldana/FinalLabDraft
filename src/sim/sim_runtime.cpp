#include "sim/sim_runtime.hpp"

#include "scene/animation.hpp"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <windows.h>

#include <algorithm>
#include <chrono>

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
}

void SimRuntime::publish_snapshot(const std::vector<scene::Object>& spawned_now) {
    std::vector<BodySnapshot> body_snaps;
    body_snaps.reserve(world_.bodies().size());
    for (const auto& b : world_.bodies()) {
        BodySnapshot s;
        s.scene_object_index   = b.scene_object_index;
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
        apply_pending();

        int hz = sim_hz_target_.load();
        float fixed_dt = 1.0f / static_cast<float>(hz);

        if (net_) {
            std::vector<net::InboundState> inbound;
            net_->drain_inbound(inbound);
            for (const auto& in : inbound) {
                uint32_t id = in.entry.body_id;
                if (id < world_.bodies().size()) {
                    auto& bodies = world_.bodies_mutable();
                    auto& b = bodies[id];
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
        }

        std::vector<scene::Object> new_now;
        if (!world_.paused) {
            world_.step(fixed_dt);
            spawners_.advance(world_.simulation_time(), new_now, rng_);
            for (auto& obj : new_now) {
                obj.mass = density_for(obj.material, scene_)
                         * scene::shape_volume(obj.shape, obj.transform.scale);
                int idx = static_cast<int>(scene_.objects.size());
                scene_.objects.push_back(obj);
                world_.add_body_from_object(scene_.objects.back(), idx);
            }
        } else {
            world_.step(fixed_dt);
        }

        if (net_ && my_peer_id_ > 0) {
            std::vector<net::StateEntry> out;
            const auto& bodies = world_.bodies();
            for (size_t i = 0; i < bodies.size(); ++i) {
                const auto& b = bodies[i];
                if (b.owner_peer_id != my_peer_id_) continue;
                net::StateEntry se{};
                se.body_id = static_cast<uint32_t>(i);
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
