#pragma once

#include "net/network_runtime.hpp"
#include "physics/physics_world.hpp"
#include "physics/spawner.hpp"
#include "scene/scene_types.hpp"

#include <glm/gtc/quaternion.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

#include <atomic>
#include <cstdint>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

namespace finalLab::sim {

struct BodySnapshot {
    int        scene_object_index   = -1;
    uint32_t   canonical_body_id    = 0;       // used by edit broadcasts
    uint8_t    owner_peer_id        = 0;
    glm::mat4  model                {1.0f};
    glm::vec3  position             {0.0f};
    glm::quat  orientation          {1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3  linear_velocity      {0.0f};
    glm::vec3  angular_velocity_rad {0.0f};
    float      inverse_mass         = 0.0f;
    glm::vec3  inverse_inertia_body {0.0f};
};

struct SnapshotData {
    std::vector<BodySnapshot>   bodies;
    std::vector<scene::Object>  spawned_since_last;
    int                         scene_version = 0;
    float                       total_time    = 0.0f;
    size_t                      contact_count = 0;
    size_t                      body_count    = 0;
    int                         sim_hz_actual = 0;
    int                         steps_since_last_drain = 0;
    physics::SpatialStats       spatial_stats{};
    physics::SpatialMode        spatial_mode = physics::SpatialMode::None;
};

class SimRuntime {
public:
    SimRuntime();
    ~SimRuntime();

    void start(uint64_t affinity_mask);
    void stop();

    void attach_network(net::NetworkRuntime* nr);
    void set_my_peer_id(int peer_id);

    void load_scene(scene::Scene scene_copy);

    void set_paused(bool);
    void single_step();
    void reset();
    void set_gravity_on(bool);
    void set_gravity(glm::vec3);
    void set_solver_iterations(int);
    void set_position_correction(float);
    void set_penetration_slop(float);
    void set_sim_hz(int);
    void set_flock_params(physics::FlockParams params);
    void set_net_smoothing(bool enabled, float rate);
    void set_spatial_mode(physics::SpatialMode mode);

    // Inspector-driven mutation. The UI calls this; the sim thread applies the
    // edit on the next tick and (when broadcast=true) propagates to peers.
    void edit_body(uint32_t body_id,
                   glm::vec3 position,
                   glm::quat orientation,
                   glm::vec3 linear_velocity,
                   glm::vec3 angular_velocity_rad,
                   bool broadcast);

    // GPU-compute path: the renderer runs Reynolds steering on the GPU and
    // pushes new linear velocities back here keyed on canonical body id. The
    // sim thread applies these on the next tick (overriding the CPU steering
    // path for the affected boids). Empty vector clears any pending overrides.
    struct BoidVelocityOverride {
        uint32_t  canonical_body_id = 0;
        glm::vec3 linear_velocity   {0.0f};
    };
    void set_boid_velocities(std::vector<BoidVelocityOverride> overrides);

    void drain_snapshot(SnapshotData& out);

    int  sim_hz_actual()       const { return sim_hz_actual_.load(); }
    int  current_scene_version() const { return scene_version_.load(); }

private:
    void thread_main();
    void apply_pending();
    void publish_snapshot(const std::vector<scene::Object>& spawned_now);

    std::thread          thread_;
    std::atomic<bool>    running_{false};
    uint64_t             affinity_mask_ = 0;

    physics::PhysicsWorld   world_;
    physics::SpawnerSystem  spawners_;
    scene::Scene            scene_;
    std::mt19937            rng_{0x5eedface};
    std::atomic<int>        scene_version_{0};
    uint32_t                my_spawn_seq_{0};   // monotonic per-peer spawn count

    net::NetworkRuntime*    net_ = nullptr;
    int                     my_peer_id_ = 0;

    std::atomic<bool>    paused_{false};
    std::atomic<bool>    gravity_on_{true};
    std::atomic<int>     sim_hz_target_{240};
    std::atomic<int>     sim_hz_actual_{0};

    std::mutex           control_mutex_;
    bool                 single_step_pending_ = false;
    bool                 reset_pending_       = false;
    bool                 scene_pending_       = false;
    scene::Scene         pending_scene_;
    bool                 gravity_dirty_       = false;
    glm::vec3            pending_gravity_     {0.0f, -9.81f, 0.0f};
    bool                 solver_dirty_        = false;
    int                  pending_solver_iters_= 6;
    bool                 tuning_dirty_        = false;
    float                pending_position_correction_ = 0.5f;
    float                pending_penetration_slop_    = 0.001f;
    bool                 flock_dirty_         = false;
    physics::FlockParams pending_flock_params_;
    bool                 net_smooth_dirty_    = false;
    bool                 pending_net_smoothing_enabled_ = true;
    float                pending_net_correction_rate_   = 8.0f;
    bool                 spatial_dirty_       = false;
    physics::SpatialMode pending_spatial_mode_ = physics::SpatialMode::None;

    struct PendingBodyEdit {
        uint32_t  body_id = 0;
        glm::vec3 position{0.0f};
        glm::quat orientation{1.0f, 0.0f, 0.0f, 0.0f};
        glm::vec3 linear_velocity{0.0f};
        glm::vec3 angular_velocity_rad{0.0f};
        bool      broadcast = false;
    };
    std::vector<PendingBodyEdit> pending_body_edits_;
    std::vector<BoidVelocityOverride> pending_boid_overrides_;

    std::mutex           snapshot_mutex_;
    SnapshotData         snapshot_;
};

uint64_t cores_to_mask(int first_one_indexed, int last_one_indexed);

} // namespace finalLab::sim
