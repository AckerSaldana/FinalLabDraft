#include "physics/spatial_index.hpp"

#include "physics/physics_world.hpp"

#include <glm/common.hpp>
#include <glm/geometric.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>

namespace finalLab::physics {

namespace {

int64_t cell_hash(int x, int y, int z) {
    return (static_cast<int64_t>(x) * 73856093ll)
         ^ (static_cast<int64_t>(y) * 19349663ll)
         ^ (static_cast<int64_t>(z) * 83492791ll);
}

int octant_for(const glm::vec3& center, const glm::vec3& p) {
    int o = 0;
    if (p.x >= center.x) o |= 1;
    if (p.y >= center.y) o |= 2;
    if (p.z >= center.z) o |= 4;
    return o;
}

} // namespace

const char* SpatialIndex::mode_name() const {
    switch (mode) {
        case SpatialMode::None:        return "None (O(n^2))";
        case SpatialMode::UniformGrid: return "Uniform Grid";
        case SpatialMode::Octree:      return "Octree";
    }
    return "?";
}

size_t SpatialIndex::memory_bytes() const {
    size_t total = 0;
    if (mode == SpatialMode::UniformGrid) {
        total += sizeof(grid_cells_);
        for (const auto& kv : grid_cells_) {
            total += sizeof(decltype(grid_cells_)::node_type);
            total += kv.second.capacity() * sizeof(int);
        }
    } else if (mode == SpatialMode::Octree) {
        total += octree_nodes_.capacity() * sizeof(OctreeNode);
        for (const auto& n : octree_nodes_) total += n.bodies.capacity() * sizeof(int);
    }
    return total;
}

void SpatialIndex::build(const std::vector<RigidBody>& bodies, float cell_size) {
    auto t0 = std::chrono::steady_clock::now();
    stats_.query_us_total = 0;
    stats_.query_count    = 0;

    if (mode == SpatialMode::UniformGrid)      build_uniform_grid(bodies, cell_size);
    else if (mode == SpatialMode::Octree)      build_octree(bodies);
    else { grid_cells_.clear(); octree_nodes_.clear(); }

    auto t1 = std::chrono::steady_clock::now();
    stats_.build_us    = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
    stats_.memory_bytes = memory_bytes();
}

void SpatialIndex::query(const glm::vec3& center, float radius, std::vector<int>& out) const {
    auto t0 = std::chrono::steady_clock::now();
    if (mode == SpatialMode::UniformGrid)      query_uniform_grid(center, radius, out);
    else if (mode == SpatialMode::Octree)      query_octree(center, radius, out);
    auto t1 = std::chrono::steady_clock::now();
    auto& s = const_cast<SpatialStats&>(stats_);
    s.query_us_total += std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
    s.query_count    += 1;
}

// ─────────────────────── Uniform grid ───────────────────────

void SpatialIndex::build_uniform_grid(const std::vector<RigidBody>& bodies, float cell_size) {
    grid_cells_.clear();
    grid_cell_size_ = (cell_size > 0.1f) ? cell_size : 1.0f;
    for (size_t i = 0; i < bodies.size(); ++i) {
        const glm::vec3& p = bodies[i].position;
        int cx = static_cast<int>(std::floor(p.x / grid_cell_size_));
        int cy = static_cast<int>(std::floor(p.y / grid_cell_size_));
        int cz = static_cast<int>(std::floor(p.z / grid_cell_size_));
        grid_cells_[cell_hash(cx, cy, cz)].push_back(static_cast<int>(i));
    }
}

void SpatialIndex::query_uniform_grid(const glm::vec3& center, float radius, std::vector<int>& out) const {
    int min_cx = static_cast<int>(std::floor((center.x - radius) / grid_cell_size_));
    int max_cx = static_cast<int>(std::floor((center.x + radius) / grid_cell_size_));
    int min_cy = static_cast<int>(std::floor((center.y - radius) / grid_cell_size_));
    int max_cy = static_cast<int>(std::floor((center.y + radius) / grid_cell_size_));
    int min_cz = static_cast<int>(std::floor((center.z - radius) / grid_cell_size_));
    int max_cz = static_cast<int>(std::floor((center.z + radius) / grid_cell_size_));
    for (int cx = min_cx; cx <= max_cx; ++cx)
    for (int cy = min_cy; cy <= max_cy; ++cy)
    for (int cz = min_cz; cz <= max_cz; ++cz) {
        auto it = grid_cells_.find(cell_hash(cx, cy, cz));
        if (it != grid_cells_.end()) {
            for (int id : it->second) out.push_back(id);
        }
    }
}

// ───────────────────────── Octree ──────────────────────────

void SpatialIndex::build_octree(const std::vector<RigidBody>& bodies) {
    octree_nodes_.clear();
    if (bodies.empty()) return;

    glm::vec3 mn(bodies[0].position), mx(bodies[0].position);
    for (const auto& b : bodies) { mn = glm::min(mn, b.position); mx = glm::max(mx, b.position); }
    glm::vec3 center = (mn + mx) * 0.5f;
    float half = std::max({mx.x - mn.x, mx.y - mn.y, mx.z - mn.z}) * 0.5f + 1.0f;

    OctreeNode root;
    root.center      = center;
    root.half_extent = half;
    octree_nodes_.push_back(root);

    struct StackEntry { int node_id; std::vector<int> ids; };
    std::vector<StackEntry> stack;
    std::vector<int> all_ids(bodies.size());
    std::iota(all_ids.begin(), all_ids.end(), 0);
    stack.push_back({0, std::move(all_ids)});

    while (!stack.empty()) {
        StackEntry entry = std::move(stack.back());
        stack.pop_back();

        glm::vec3 ncenter = octree_nodes_[entry.node_id].center;
        float     nhalf   = octree_nodes_[entry.node_id].half_extent;

        if (entry.ids.size() <= static_cast<size_t>(kOctreeMaxPerLeaf) || nhalf < kOctreeMinExtent) {
            octree_nodes_[entry.node_id].bodies = std::move(entry.ids);
            continue;
        }

        std::array<std::vector<int>, 8> octs;
        for (int id : entry.ids) {
            int o = octant_for(ncenter, bodies[id].position);
            octs[o].push_back(id);
        }
        float chalf = nhalf * 0.5f;
        for (int c = 0; c < 8; ++c) {
            OctreeNode child;
            child.half_extent = chalf;
            child.center.x    = ncenter.x + ((c & 1) ? chalf : -chalf);
            child.center.y    = ncenter.y + ((c & 2) ? chalf : -chalf);
            child.center.z    = ncenter.z + ((c & 4) ? chalf : -chalf);
            int new_idx = static_cast<int>(octree_nodes_.size());
            octree_nodes_.push_back(child);
            octree_nodes_[entry.node_id].children[c] = new_idx;
            stack.push_back({ new_idx, std::move(octs[c]) });
        }
    }
}

void SpatialIndex::query_octree(const glm::vec3& center, float radius, std::vector<int>& out) const {
    if (octree_nodes_.empty()) return;
    query_octree_node(0, center, radius, out);
}

void SpatialIndex::query_octree_node(int node_id, const glm::vec3& q, float r, std::vector<int>& out) const {
    const OctreeNode& n = octree_nodes_[node_id];
    glm::vec3 d = glm::abs(q - n.center) - glm::vec3(n.half_extent);
    d = glm::max(d, glm::vec3(0.0f));
    if (glm::dot(d, d) > r * r) return;

    if (n.children[0] == -1) {
        for (int id : n.bodies) out.push_back(id);
        return;
    }
    for (int c = 0; c < 8; ++c) {
        if (n.children[c] != -1) query_octree_node(n.children[c], q, r, out);
    }
}

} // namespace finalLab::physics
