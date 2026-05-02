#include "render/mesh.hpp"

#include <glm/geometric.hpp>
#include <glm/trigonometric.hpp>

#include <cmath>

namespace finalLab::render {

namespace {

constexpr float kPi = 3.14159265358979323846f;

void push_quad(std::vector<uint32_t>& idx, uint32_t a, uint32_t b, uint32_t c, uint32_t d) {
    idx.push_back(a); idx.push_back(b); idx.push_back(c);
    idx.push_back(a); idx.push_back(c); idx.push_back(d);
}

} // namespace

MeshData make_sphere(uint32_t latitudes, uint32_t longitudes) {
    MeshData m;
    for (uint32_t lat = 0; lat <= latitudes; ++lat) {
        float v = static_cast<float>(lat) / latitudes;
        float phi = v * kPi;
        float y = std::cos(phi);
        float r = std::sin(phi);
        for (uint32_t lon = 0; lon <= longitudes; ++lon) {
            float u = static_cast<float>(lon) / longitudes;
            float theta = u * 2.0f * kPi;
            float x = r * std::cos(theta);
            float z = r * std::sin(theta);
            glm::vec3 n{ x, y, z };
            m.vertices.push_back({ n, n });
        }
    }
    for (uint32_t lat = 0; lat < latitudes; ++lat) {
        for (uint32_t lon = 0; lon < longitudes; ++lon) {
            uint32_t a = lat * (longitudes + 1) + lon;
            uint32_t b = a + (longitudes + 1);
            m.indices.push_back(a);
            m.indices.push_back(a + 1);
            m.indices.push_back(b);
            m.indices.push_back(b);
            m.indices.push_back(a + 1);
            m.indices.push_back(b + 1);
        }
    }
    return m;
}

MeshData make_cuboid() {
    MeshData m;
    struct Face { glm::vec3 n; glm::vec3 u; glm::vec3 v; };
    const Face faces[6] = {
        { { 1, 0, 0}, {0, 0,-1}, {0, 1, 0} },
        { {-1, 0, 0}, {0, 0, 1}, {0, 1, 0} },
        { { 0, 1, 0}, {1, 0, 0}, {0, 0, 1} },
        { { 0,-1, 0}, {1, 0, 0}, {0, 0,-1} },
        { { 0, 0, 1}, {1, 0, 0}, {0, 1, 0} },
        { { 0, 0,-1}, {-1,0, 0}, {0, 1, 0} },
    };
    for (const Face& f : faces) {
        glm::vec3 c = f.n * 0.5f;
        uint32_t base = static_cast<uint32_t>(m.vertices.size());
        m.vertices.push_back({ c - 0.5f * f.u - 0.5f * f.v, f.n });
        m.vertices.push_back({ c + 0.5f * f.u - 0.5f * f.v, f.n });
        m.vertices.push_back({ c + 0.5f * f.u + 0.5f * f.v, f.n });
        m.vertices.push_back({ c - 0.5f * f.u + 0.5f * f.v, f.n });
        push_quad(m.indices, base, base + 1, base + 2, base + 3);
    }
    return m;
}

MeshData make_cylinder(uint32_t segments) {
    MeshData m;
    const float h = 0.5f;

    for (uint32_t i = 0; i <= segments; ++i) {
        float t = static_cast<float>(i) / segments * 2.0f * kPi;
        float x = std::cos(t), z = std::sin(t);
        glm::vec3 n{ x, 0, z };
        m.vertices.push_back({ {x,  h, z}, n });
        m.vertices.push_back({ {x, -h, z}, n });
    }
    for (uint32_t i = 0; i < segments; ++i) {
        uint32_t a = i * 2;
        uint32_t b = a + 2;
        m.indices.push_back(a); m.indices.push_back(b);     m.indices.push_back(a + 1);
        m.indices.push_back(b); m.indices.push_back(b + 1); m.indices.push_back(a + 1);
    }

    uint32_t top_center = static_cast<uint32_t>(m.vertices.size());
    m.vertices.push_back({ { 0,  h, 0 }, { 0, 1, 0 } });
    uint32_t top_start = static_cast<uint32_t>(m.vertices.size());
    for (uint32_t i = 0; i <= segments; ++i) {
        float t = static_cast<float>(i) / segments * 2.0f * kPi;
        float x = std::cos(t), z = std::sin(t);
        m.vertices.push_back({ { x, h, z }, { 0, 1, 0 } });
    }
    for (uint32_t i = 0; i < segments; ++i) {
        m.indices.push_back(top_center);
        m.indices.push_back(top_start + i + 1);
        m.indices.push_back(top_start + i);
    }

    uint32_t bot_center = static_cast<uint32_t>(m.vertices.size());
    m.vertices.push_back({ { 0, -h, 0 }, { 0, -1, 0 } });
    uint32_t bot_start = static_cast<uint32_t>(m.vertices.size());
    for (uint32_t i = 0; i <= segments; ++i) {
        float t = static_cast<float>(i) / segments * 2.0f * kPi;
        float x = std::cos(t), z = std::sin(t);
        m.vertices.push_back({ { x, -h, z }, { 0, -1, 0 } });
    }
    for (uint32_t i = 0; i < segments; ++i) {
        m.indices.push_back(bot_center);
        m.indices.push_back(bot_start + i);
        m.indices.push_back(bot_start + i + 1);
    }

    return m;
}

// Capsule template: radius 1, cylinder body from y=-0.5 to y=+0.5, hemispheres of radius 1 on each end.
// Total bounding height = 2.5 from y=-1.5 to y=+1.5 when r=1 body-h=1... actually body=1 plus r=1 each end => -1.5 to +1.5.
// For instancing, uniform scaling reshapes consistently (round caps stay round).
MeshData make_capsule(uint32_t segments, uint32_t rings) {
    MeshData m;
    const float h_half = 0.5f;

    auto add_hemi = [&](bool top) {
        float y_offset = top ? h_half : -h_half;
        float y_sign   = top ? 1.0f : -1.0f;
        uint32_t base  = static_cast<uint32_t>(m.vertices.size());
        for (uint32_t lat = 0; lat <= rings; ++lat) {
            float v = static_cast<float>(lat) / rings;
            float phi = v * (kPi * 0.5f);
            float r = std::sin(phi);
            float y = y_sign * std::cos(phi);
            for (uint32_t lon = 0; lon <= segments; ++lon) {
                float u = static_cast<float>(lon) / segments;
                float theta = u * 2.0f * kPi;
                float x = r * std::cos(theta);
                float z = r * std::sin(theta);
                glm::vec3 n{ x, y, z };
                m.vertices.push_back({ { x, y + y_offset, z }, n });
            }
        }
        for (uint32_t lat = 0; lat < rings; ++lat) {
            for (uint32_t lon = 0; lon < segments; ++lon) {
                uint32_t a = base + lat * (segments + 1) + lon;
                uint32_t b = a + (segments + 1);
                if (top) {
                    m.indices.push_back(a); m.indices.push_back(a + 1); m.indices.push_back(b);
                    m.indices.push_back(b); m.indices.push_back(a + 1); m.indices.push_back(b + 1);
                } else {
                    m.indices.push_back(a); m.indices.push_back(b); m.indices.push_back(a + 1);
                    m.indices.push_back(b); m.indices.push_back(b + 1); m.indices.push_back(a + 1);
                }
            }
        }
    };
    add_hemi(true);
    add_hemi(false);

    uint32_t side_base = static_cast<uint32_t>(m.vertices.size());
    for (uint32_t i = 0; i <= segments; ++i) {
        float t = static_cast<float>(i) / segments * 2.0f * kPi;
        float x = std::cos(t), z = std::sin(t);
        glm::vec3 n{ x, 0, z };
        m.vertices.push_back({ { x,  h_half, z }, n });
        m.vertices.push_back({ { x, -h_half, z }, n });
    }
    for (uint32_t i = 0; i < segments; ++i) {
        uint32_t a = side_base + i * 2;
        uint32_t b = a + 2;
        m.indices.push_back(a); m.indices.push_back(b);     m.indices.push_back(a + 1);
        m.indices.push_back(b); m.indices.push_back(b + 1); m.indices.push_back(a + 1);
    }
    return m;
}

MeshData make_plane() {
    MeshData m;
    glm::vec3 n{ 0, 1, 0 };
    m.vertices.push_back({ { -0.5f, 0, -0.5f }, n });
    m.vertices.push_back({ {  0.5f, 0, -0.5f }, n });
    m.vertices.push_back({ {  0.5f, 0,  0.5f }, n });
    m.vertices.push_back({ { -0.5f, 0,  0.5f }, n });
    // CCW-when-viewed-from-above winding. The previous (0,1,2,3) order produced
    // a geometric front normal of -y while the stored vertex normal was +y, so
    // backface culling killed the floor from any above-camera view.
    push_quad(m.indices, 0, 3, 2, 1);
    return m;
}

MeshData make_container(const MeshData& solid) {
    MeshData m = solid;
    for (Vertex& v : m.vertices) v.normal = -v.normal;
    for (size_t i = 0; i + 2 < m.indices.size(); i += 3) {
        std::swap(m.indices[i + 1], m.indices[i + 2]);
    }
    return m;
}

Mesh upload_mesh(VkDevice device, VkPhysicalDevice pd,
                 VkQueue queue, VkCommandPool pool,
                 const MeshData& data) {
    Mesh mesh;
    mesh.index_count = static_cast<uint32_t>(data.indices.size());

    VkDeviceSize vbytes = sizeof(Vertex)   * data.vertices.size();
    VkDeviceSize ibytes = sizeof(uint32_t) * data.indices.size();

    mesh.vbuf = create_buffer(device, pd, vbytes,
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    mesh.ibuf = create_buffer(device, pd, ibytes,
        VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    upload_to_device_local(device, pd, queue, pool, mesh.vbuf, data.vertices.data(), vbytes);
    upload_to_device_local(device, pd, queue, pool, mesh.ibuf, data.indices.data(),  ibytes);
    return mesh;
}

void destroy_mesh(VkDevice device, Mesh& mesh) {
    destroy_buffer(device, mesh.vbuf);
    destroy_buffer(device, mesh.ibuf);
    mesh.index_count = 0;
}

} // namespace finalLab::render
