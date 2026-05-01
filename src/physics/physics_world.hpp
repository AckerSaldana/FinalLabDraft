#pragma once

#include "physics/contact.hpp"
#include "physics/flocking.hpp"
#include "physics/spatial_index.hpp"
#include "scene/scene_types.hpp"

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace finalLab::physics {

struct RigidBody {
    glm::vec3 position{0.0f};
    glm::quat orientation{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 linear_velocity{0.0f};
    glm::vec3 angular_velocity_rad{0.0f};

    float                inverse_mass            = 0.0f;
    glm::vec3            inverse_inertia_body    = {0.0f, 0.0f, 0.0f};

    scene::Shape         shape;
    glm::vec3            effective_scale         = {1.0f, 1.0f, 1.0f};
    std::string          material;
    scene::CollisionType collision_type          = scene::CollisionType::Solid;

    int                  scene_object_index      = -1;
    uint8_t              owner_peer_id           = 0;
    bool                 is_boid                 = false;
    // Canonical body ID — identical across peers for the same logical body.
    // Pre-loaded objects use their scene_object_index; spawned objects use a
    // peer-prefixed counter assigned by the spawner-owner so two peers cannot
    // collide. State and edit messages reference this, never the local index.
    uint32_t             canonical_body_id       = 0;

    glm::vec3            net_target_position     {0.0f};
    glm::quat            net_target_orientation  {1.0f, 0.0f, 0.0f, 0.0f};
    bool                 has_net_target          = false;
};

struct ResolvedInteraction {
    float restitution      = 0.3f;
    float static_friction  = 0.5f;
    float dynamic_friction = 0.4f;
};

class PhysicsWorld {
public:
    void build_from_scene(const scene::Scene& scene);
    void clear();
    void step(float dt);
    void reset_to_initial();

    int  my_peer_id  = 0;

    void add_body_from_object(const scene::Object& o, int scene_object_index, uint32_t canonical_body_id);

    // Returns local index for the given canonical id, or -1 if not present.
    int  find_local_index(uint32_t canonical_body_id) const;

    void request_single_step() { single_step_pending_ = true; }

    const std::vector<RigidBody>& bodies()         const { return bodies_; }
    std::vector<RigidBody>&       bodies_mutable()       { return bodies_; }
    size_t                        body_count()     const { return bodies_.size(); }
    size_t                        last_contact_count() const { return last_contact_count_; }

    ResolvedInteraction lookup_interaction(const std::string& a, const std::string& b) const;

    bool       paused              = false;
    bool       gravity_on          = true;
    glm::vec3  gravity             = { 0.0f, -9.81f, 0.0f };
    int        solver_iterations   = 6;
    float      position_correction = 0.5f;
    float      penetration_slop    = 0.001f;
    FlockParams flock_params{};

    bool       net_smoothing_enabled  = true;
    float      net_correction_rate    = 8.0f;

    SpatialIndex spatial_index;
    const SpatialStats& spatial_stats() const { return spatial_index.stats(); }

    float      simulation_time() const { return total_time_; }

private:
    struct AnimatedRecord {
        int                       body_index = -1;
        scene::AnimatedObject     anim;
    };

    std::vector<RigidBody>                  bodies_;
    std::vector<RigidBody>                  initial_;
    std::unordered_map<uint32_t, uint32_t>  canonical_to_local_;
    std::vector<scene::MaterialInteraction> interactions_;
    std::vector<AnimatedRecord>             animated_;
    std::vector<Contact>                    contacts_scratch_;
    bool                                    single_step_pending_ = false;
    size_t                                  last_contact_count_  = 0;
    float                                   total_time_          = 0.0f;
};

} // namespace finalLab::physics
