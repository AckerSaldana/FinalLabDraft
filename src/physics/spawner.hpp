#pragma once

#include "scene/scene_types.hpp"

#include <random>
#include <vector>

namespace finalLab::physics {

class SpawnerSystem {
public:
    void build(const std::vector<scene::Spawner>& defs);
    void clear();
    void reset();

    int  advance(float current_time, std::vector<scene::Object>& out_objects, std::mt19937& rng);

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
