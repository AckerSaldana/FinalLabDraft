#pragma once

#include <vulkan/vulkan.h>

#include <cstdint>
#include <vector>

namespace finalLab::core { class Window; }

namespace finalLab::render {

struct QueueFamilies {
    uint32_t graphics = UINT32_MAX;
    uint32_t present  = UINT32_MAX;
    bool complete() const { return graphics != UINT32_MAX && present != UINT32_MAX; }
};

class VulkanContext {
public:
    VulkanContext(core::Window& window, bool enable_validation);
    ~VulkanContext();

    VulkanContext(const VulkanContext&) = delete;
    VulkanContext& operator=(const VulkanContext&) = delete;

    VkInstance       instance()         const { return instance_; }
    VkPhysicalDevice physical_device()  const { return physical_device_; }
    VkDevice         device()           const { return device_; }
    VkQueue          graphics_queue()   const { return graphics_queue_; }
    VkQueue          present_queue()    const { return present_queue_; }
    QueueFamilies    queue_families()   const { return queue_families_; }
    VkSurfaceKHR     surface()          const { return surface_; }

    VkSwapchainKHR   swapchain()        const { return swapchain_; }
    VkFormat         swapchain_format() const { return swapchain_format_; }
    VkExtent2D       swapchain_extent() const { return swapchain_extent_; }
    const std::vector<VkImage>&     swapchain_images()      const { return swapchain_images_; }
    const std::vector<VkImageView>& swapchain_image_views() const { return swapchain_image_views_; }

    uint32_t swapchain_image_count() const { return static_cast<uint32_t>(swapchain_images_.size()); }

    void recreate_swapchain(core::Window& window);

private:
    void create_instance();
    void create_debug_messenger();
    void pick_physical_device();
    void create_logical_device();
    void create_swapchain(core::Window& window);
    void destroy_swapchain();

    bool enable_validation_ = false;

    VkInstance               instance_         = VK_NULL_HANDLE;
    VkDebugUtilsMessengerEXT debug_messenger_  = VK_NULL_HANDLE;
    VkSurfaceKHR             surface_          = VK_NULL_HANDLE;
    VkPhysicalDevice         physical_device_  = VK_NULL_HANDLE;
    VkDevice                 device_           = VK_NULL_HANDLE;
    QueueFamilies            queue_families_;
    VkQueue                  graphics_queue_   = VK_NULL_HANDLE;
    VkQueue                  present_queue_    = VK_NULL_HANDLE;

    VkSwapchainKHR           swapchain_        = VK_NULL_HANDLE;
    VkFormat                 swapchain_format_ = VK_FORMAT_UNDEFINED;
    VkExtent2D               swapchain_extent_ = {0, 0};
    std::vector<VkImage>     swapchain_images_;
    std::vector<VkImageView> swapchain_image_views_;
};

} // namespace finalLab::render
