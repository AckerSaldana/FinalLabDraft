#pragma once

#include "render/buffer.hpp"
#include "render/mesh.hpp"

#include <vulkan/vulkan.h>
#include <glm/mat4x4.hpp>
#include <glm/vec4.hpp>

#include <array>
#include <cstdint>
#include <span>
#include <vector>

namespace finalLab::render {

class VulkanContext;

enum class ShapeKind : uint8_t { Sphere, Cuboid, Cylinder, Capsule, Plane, Count };

struct RenderInstance {
    glm::mat4 model;
    glm::vec4 color;
    ShapeKind shape     = ShapeKind::Sphere;
    bool      container = false;
};

class InstanceRenderer {
public:
    static constexpr uint32_t kFramesInFlight = 2;

    InstanceRenderer(VulkanContext& ctx, VkFormat color_format, VkFormat depth_format);
    ~InstanceRenderer();

    InstanceRenderer(const InstanceRenderer&) = delete;
    InstanceRenderer& operator=(const InstanceRenderer&) = delete;

    void draw(uint32_t frame_index, VkCommandBuffer cmd,
              const glm::mat4& view_proj,
              std::span<const RenderInstance> instances);

private:
    struct InstanceGPU {
        glm::mat4 model;
        glm::vec4 color;
    };

    void create_meshes(VkQueue queue, VkCommandPool pool);
    void create_descriptors();
    void create_pipeline(VkFormat color_format, VkFormat depth_format);
    void create_per_frame_buffers();
    void ensure_instance_capacity(uint32_t frame_index, VkDeviceSize required_bytes);
    void update_descriptor_for_frame(uint32_t frame_index);

    VulkanContext& ctx_;

    std::array<Mesh, static_cast<size_t>(ShapeKind::Count) * 2> meshes_{};

    VkCommandPool                                                  upload_pool_     = VK_NULL_HANDLE;
    VkDescriptorSetLayout                                          set_layout_      = VK_NULL_HANDLE;
    VkDescriptorPool                                               desc_pool_       = VK_NULL_HANDLE;
    std::array<VkDescriptorSet, kFramesInFlight>                   desc_sets_{};
    VkPipelineLayout                                               pipeline_layout_ = VK_NULL_HANDLE;
    VkPipeline                                                     pipeline_        = VK_NULL_HANDLE;

    std::array<Buffer, kFramesInFlight>                            instance_bufs_{};
    std::array<VkDeviceSize, kFramesInFlight>                      instance_cap_{};
};

} // namespace finalLab::render
