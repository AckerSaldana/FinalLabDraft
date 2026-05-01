#pragma once

#include <glm/vec3.hpp>

#include <array>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace finalLab::physics {

struct RigidBody;

enum class SpatialMode : uint8_t { None, UniformGrid, Octree };

struct SpatialStats {
    int64_t build_us       = 0;
    int64_t query_us_total = 0;
    int     query_count    = 0;
    size_t  memory_bytes   = 0;
};

class SpatialIndex {
public:
    void build(const std::vector<RigidBody>& bodies, float cell_size);
    void query(const glm::vec3& center, float radius, std::vector<int>& out) const;

    size_t memory_bytes()             const;
    const char* mode_name()           const;
    const SpatialStats& stats()       const { return stats_; }
    SpatialStats& stats_mutable()           { return stats_; }

    SpatialMode mode = SpatialMode::None;

private:
    float                                              grid_cell_size_ = 4.0f;
    std::unordered_map<int64_t, std::vector<int>>      grid_cells_;

    struct OctreeNode {
        glm::vec3            center{0.0f};
        float                half_extent = 0.0f;
        std::vector<int>     bodies;
        std::array<int, 8>   children{-1,-1,-1,-1,-1,-1,-1,-1};
    };
    std::vector<OctreeNode> octree_nodes_;
    static constexpr int    kOctreeMaxPerLeaf = 8;
    static constexpr float  kOctreeMinExtent  = 1.0f;

    SpatialStats stats_{};

    void build_uniform_grid(const std::vector<RigidBody>& bodies, float cell_size);
    void build_octree      (const std::vector<RigidBody>& bodies);
    void query_uniform_grid(const glm::vec3& center, float radius, std::vector<int>& out) const;
    void query_octree      (const glm::vec3& center, float radius, std::vector<int>& out) const;
    void query_octree_node (int node_id, const glm::vec3& center, float radius, std::vector<int>& out) const;
};

} // namespace finalLab::physics
