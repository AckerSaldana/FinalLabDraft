#pragma once

#include <vulkan/vulkan.h>

struct GLFWwindow;

namespace finalLab::render {

class VulkanContext;

class ImGuiLayer {
public:
    ImGuiLayer(VulkanContext& ctx, GLFWwindow* window, VkFormat color_format);
    ~ImGuiLayer();

    ImGuiLayer(const ImGuiLayer&) = delete;
    ImGuiLayer& operator=(const ImGuiLayer&) = delete;

    void begin_frame();
    void render(VkCommandBuffer cmd);

private:
    VulkanContext& ctx_;
    VkDescriptorPool descriptor_pool_ = VK_NULL_HANDLE;
};

} // namespace finalLab::render
