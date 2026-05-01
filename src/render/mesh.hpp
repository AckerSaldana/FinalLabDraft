#pragma once

#include "render/buffer.hpp"

#include <glm/vec3.hpp>

#include <cstdint>
#include <vector>

namespace finalLab::render {

struct Vertex {
    glm::vec3 position;
    glm::vec3 normal;
};

struct MeshData {
    std::vector<Vertex>   vertices;
    std::vector<uint32_t> indices;
};

struct Mesh {
    Buffer   vbuf;
    Buffer   ibuf;
    uint32_t index_count = 0;
};

MeshData make_sphere(uint32_t latitudes = 16, uint32_t longitudes = 24);
MeshData make_cuboid();
MeshData make_cylinder(uint32_t segments = 24);
MeshData make_capsule(uint32_t segments = 16, uint32_t rings = 8);
MeshData make_plane();

MeshData make_container(const MeshData& solid);

Mesh upload_mesh(VkDevice device, VkPhysicalDevice pd,
                 VkQueue queue, VkCommandPool pool,
                 const MeshData& data);

void destroy_mesh(VkDevice device, Mesh& mesh);

} // namespace finalLab::render
