#include "render/imgui_layer.hpp"

#include "render/vulkan_context.hpp"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_vulkan.h>

#include <array>
#include <stdexcept>

namespace finalLab::render {

namespace {

VkDescriptorPool make_descriptor_pool(VkDevice device) {
    std::array<VkDescriptorPoolSize, 2> sizes{{
        { VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 256 },
        { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,         64  },
    }};

    VkDescriptorPoolCreateInfo ci{};
    ci.sType         = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    ci.flags         = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    ci.maxSets       = 256;
    ci.poolSizeCount = static_cast<uint32_t>(sizes.size());
    ci.pPoolSizes    = sizes.data();

    VkDescriptorPool pool = VK_NULL_HANDLE;
    if (vkCreateDescriptorPool(device, &ci, nullptr, &pool) != VK_SUCCESS) {
        throw std::runtime_error("vkCreateDescriptorPool (imgui) failed");
    }
    return pool;
}

} // namespace

ImGuiLayer::ImGuiLayer(VulkanContext& ctx, GLFWwindow* window, VkFormat color_format,
                       VkFormat depth_format)
    : ctx_(ctx) {
    descriptor_pool_ = make_descriptor_pool(ctx_.device());

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForVulkan(window, true);

    VkPipelineRenderingCreateInfo pipeline_rendering{};
    pipeline_rendering.sType                   = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
    pipeline_rendering.colorAttachmentCount    = 1;
    pipeline_rendering.pColorAttachmentFormats = &color_format;
    // ImGui pipeline must declare the depth format used by the render pass it
    // will be invoked inside, even if it doesn't write depth itself; otherwise
    // VK validation logs format-mismatch warnings every draw call.
    pipeline_rendering.depthAttachmentFormat   = depth_format;

    ImGui_ImplVulkan_InitInfo init{};
    init.Instance        = ctx_.instance();
    init.PhysicalDevice  = ctx_.physical_device();
    init.Device          = ctx_.device();
    init.QueueFamily     = ctx_.queue_families().graphics;
    init.Queue           = ctx_.graphics_queue();
    init.DescriptorPool  = descriptor_pool_;
    init.MinImageCount   = ctx_.swapchain_image_count();
    init.ImageCount      = ctx_.swapchain_image_count();
    init.PipelineInfoMain.MSAASamples                 = VK_SAMPLE_COUNT_1_BIT;
    init.PipelineInfoMain.PipelineRenderingCreateInfo = pipeline_rendering;
    init.UseDynamicRendering                          = true;

    if (!ImGui_ImplVulkan_Init(&init)) {
        throw std::runtime_error("ImGui_ImplVulkan_Init failed");
    }
}

ImGuiLayer::~ImGuiLayer() {
    vkDeviceWaitIdle(ctx_.device());
    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    if (descriptor_pool_) vkDestroyDescriptorPool(ctx_.device(), descriptor_pool_, nullptr);
}

void ImGuiLayer::begin_frame() {
    ImGui_ImplVulkan_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void ImGuiLayer::render(VkCommandBuffer cmd) {
    ImGui::Render();
    ImDrawData* data = ImGui::GetDrawData();
    if (data) ImGui_ImplVulkan_RenderDrawData(data, cmd);
}

} // namespace finalLab::render
