#pragma once

#include "render/buffer.hpp"

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include <vulkan/vulkan.h>

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace finalLab::render {

class VulkanContext;

// GPU compute pipeline for Reynolds boid steering. Uses a single host-visible
// SSBO that the host fills with positions+velocities; the shader writes new
// velocities back to the same buffer for readback. The pipeline records its
// own one-shot command buffer and submits to the graphics queue (all Vulkan
// graphics queues support COMPUTE per spec, so no separate compute queue is
// required for the lab demo).
class BoidComputePipeline {
public:
    struct BoidGPU {
        glm::vec4 position;     // .xyz used, .w padding
        glm::vec4 velocity;     // .xyz used, .w padding
    };
    static_assert(sizeof(BoidGPU) == 32, "BoidGPU layout must match the GLSL std430 expectation");

    struct PushConstants {
        uint32_t count            = 0;
        float    dt               = 1.0f / 240.0f;
        float    neighbour_radius = 4.0f;
        float    avoidance_radius = 4.0f;
        float    w_cohesion       = 1.0f;
        float    w_alignment      = 1.0f;
        float    w_separation     = 1.5f;
        float    max_speed        = 8.0f;
        float    max_force        = 12.0f;
        float    pad0             = 0.0f;
    };

    BoidComputePipeline(VulkanContext& ctx, const std::filesystem::path& spv_path);
    ~BoidComputePipeline();

    BoidComputePipeline(const BoidComputePipeline&)            = delete;
    BoidComputePipeline& operator=(const BoidComputePipeline&) = delete;

    // Ensure the SSBO can hold at least `n` boids; reallocates and rebuilds the
    // descriptor write if the current capacity is too small.
    void ensure_capacity(uint32_t n);

    // Run the steering kernel synchronously: upload→dispatch→readback.
    // boids[]   in-place data, modified to receive new velocities on return.
    // pc        steering parameters; .count is overwritten with boids.size().
    void run(std::vector<BoidGPU>& boids, PushConstants pc);

    // Telemetry — last successful dispatch, in microseconds.
    long long last_dispatch_us() const { return last_dispatch_us_; }
    uint32_t  capacity()         const { return capacity_; }

private:
    void create_descriptor_layout();
    void create_pipeline(const std::filesystem::path& spv_path);
    void create_command_pool();
    void create_descriptor_pool_and_set();
    void destroy_buffer_resources();
    void allocate_buffer(uint32_t n);
    void update_descriptor_set();

    VulkanContext&        ctx_;
    VkDevice              device_                  = VK_NULL_HANDLE;
    VkDescriptorSetLayout set_layout_              = VK_NULL_HANDLE;
    VkPipelineLayout      pipeline_layout_         = VK_NULL_HANDLE;
    VkPipeline            pipeline_                = VK_NULL_HANDLE;
    VkShaderModule        shader_                  = VK_NULL_HANDLE;
    VkDescriptorPool      descriptor_pool_         = VK_NULL_HANDLE;
    VkDescriptorSet       descriptor_set_          = VK_NULL_HANDLE;
    VkCommandPool         command_pool_            = VK_NULL_HANDLE;
    VkCommandBuffer       command_buffer_          = VK_NULL_HANDLE;

    Buffer                ssbo_{};
    uint32_t              capacity_                = 0;
    long long             last_dispatch_us_        = 0;
};

// Read a SPIR-V binary file. Throws on failure.
std::vector<uint32_t> load_spirv(const std::filesystem::path& path);

} // namespace finalLab::render
