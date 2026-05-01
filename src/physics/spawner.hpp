#pragma once

#include "scene/scene_types.hpp"

#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

namespace finalLab::physics {

class SpawnerSystem {
public:
    void build(const std::vector<scene::Spawner>& defs);
    void clear();
    void reset();

    int  advance(float current_time, std::vector<scene::Object>& out_objects, std::mt19937& rng);

    // Owner-gated variant: only spawners whose runner-peer matches my_peer_id
    // produce objects. The runner-peer is derived from the spawner's owner kind:
    //   One/Two/Three/Four → that peer index runs the spawner.
    //   Sequential         → peer 1 runs the spawner (the lowest-id peer is
    //                        a deterministic, well-defined choice; the spawned
    //                        objects' OWNERSHIP still rotates among 1..4 as
    //                        described by the spec).
    // Pass my_peer_id == 0 in single-peer mode to fall back to "everyone runs
    // every spawner" (equivalent to the unfiltered advance()).
    int  advance_owned(float current_time, std::vector<scene::Object>& out_objects,
                       std::mt19937& rng, int my_peer_id,
                       std::vector<size_t>* spawner_indices_for_each_object = nullptr);

    // Non-owner path: append a single object that arrived over the network. The
    // spawner state isn't touched (the owner is authoritative on count / timing).
    void apply_remote_object(scene::Object o, std::vector<scene::Object>& out);

    // For owner→canonical_body_id encoding: the spawner runner can use these to
    // produce a stable per-peer prefix, leaving the lower bits for spawn count.
    static constexpr uint32_t kSpawnedIdPeerShift = 24;
    static constexpr uint32_t kSpawnedIdMaskCount = (1u << kSpawnedIdPeerShift) - 1u;
    static uint32_t canonical_id_for(int my_peer_id, uint32_t my_spawn_seq) {
        // peer_id 1..4 lives in bits 24..31; lower 24 bits = monotonic sequence.
        // Pre-loaded scene objects keep canonical_id < (1 << 24) reserved for them.
        return (static_cast<uint32_t>(my_peer_id) << kSpawnedIdPeerShift) | (my_spawn_seq & kSpawnedIdMaskCount);
    }

    size_t spawner_count()                    const { return defs_.size(); }
    int    spawned_count(size_t spawner_idx)  const;
    float  next_fire_time(size_t spawner_idx) const;
    bool   exhausted(size_t spawner_idx)      const;

private:
    struct State {
        int   count             = 0;
        float next_fire_time    = -1.0f;
        bool  single_burst_done = false;
        int   sequential_next   = 0;
    };

    std::vector<scene::Spawner> defs_;
    std::vector<State>          states_;
};

} // namespace finalLab::physics
