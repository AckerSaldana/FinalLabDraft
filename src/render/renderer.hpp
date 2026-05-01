#pragma once

#include <vulkan/vulkan.h>

#include <array>
#include <cstdint>

namespace finalLab::core { class Window; }

namespace finalLab::render {

class VulkanContext;

struct FrameContext {
    VkCommandBuffer cmd         = VK_NULL_HANDLE;
    uint32_t        image_index = 0;
    uint32_t        frame_index = 0;
    VkExtent2D      extent      = {0, 0};
};

class Renderer {
public:
    static constexpr uint32_t kFramesInFlight = 2;

    Renderer(VulkanContext& ctx, core::Window& window);
    ~Renderer();

    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    bool begin_frame(FrameContext& out);
    void end_frame(FrameContext& frame);

    void notify_resized() { framebuffer_resized_ = true; }

    VkFormat color_format() const;
    VkFormat depth_format() const { return depth_format_; }

private:
    void create_frame_resources();
    void destroy_frame_resources();
    void create_depth(VkExtent2D extent);
    void destroy_depth();

    VulkanContext& ctx_;
    core::Window&  window_;

    VkCommandPool                                             command_pool_     = VK_NULL_HANDLE;
    std::array<VkCommandBuffer,    kFramesInFlight>           command_buffers_{};
    std::array<VkSemaphore,        kFramesInFlight>           image_available_{};
    std::array<VkSemaphore,        kFramesInFlight>           render_finished_{};
    std::array<VkFence,            kFramesInFlight>           in_flight_fences_{};

    VkFormat        depth_format_  = VK_FORMAT_D32_SFLOAT;
    VkImage         depth_image_   = VK_NULL_HANDLE;
    VkDeviceMemory  depth_memory_  = VK_NULL_HANDLE;
    VkImageView     depth_view_    = VK_NULL_HANDLE;
    VkExtent2D      depth_extent_  = {0, 0};

    uint32_t current_frame_       = 0;
    bool     framebuffer_resized_ = false;
};

} // namespace finalLab::render
