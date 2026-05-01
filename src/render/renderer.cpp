#include "render/renderer.hpp"

#include "core/window.hpp"
#include "render/buffer.hpp"
#include "render/vulkan_context.hpp"

#include <stdexcept>

namespace finalLab::render {

namespace {

void image_barrier(VkCommandBuffer cmd, VkImage image,
                   VkImageLayout old_layout, VkImageLayout new_layout,
                   VkPipelineStageFlags2 src_stage, VkAccessFlags2 src_access,
                   VkPipelineStageFlags2 dst_stage, VkAccessFlags2 dst_access) {
    VkImageMemoryBarrier2 b{};
    b.sType            = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
    b.srcStageMask     = src_stage;
    b.srcAccessMask    = src_access;
    b.dstStageMask     = dst_stage;
    b.dstAccessMask    = dst_access;
    b.oldLayout        = old_layout;
    b.newLayout        = new_layout;
    b.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    b.image            = image;
    b.subresourceRange.aspectMask     = VK_IMAGE_ASPECT_COLOR_BIT;
    b.subresourceRange.baseMipLevel   = 0;
    b.subresourceRange.levelCount     = 1;
    b.subresourceRange.baseArrayLayer = 0;
    b.subresourceRange.layerCount     = 1;

    VkDependencyInfo dep{};
    dep.sType                   = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
    dep.imageMemoryBarrierCount = 1;
    dep.pImageMemoryBarriers    = &b;
    vkCmdPipelineBarrier2(cmd, &dep);
}

} // namespace

Renderer::Renderer(VulkanContext& ctx, core::Window& window)
    : ctx_(ctx), window_(window) {
    create_frame_resources();
    create_depth(ctx_.swapchain_extent());
}

Renderer::~Renderer() {
    vkDeviceWaitIdle(ctx_.device());
    destroy_depth();
    destroy_frame_resources();
}

VkFormat Renderer::color_format() const { return ctx_.swapchain_format(); }

void Renderer::create_frame_resources() {
    VkCommandPoolCreateInfo pi{};
    pi.sType            = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    pi.flags            = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    pi.queueFamilyIndex = ctx_.queue_families().graphics;
    if (vkCreateCommandPool(ctx_.device(), &pi, nullptr, &command_pool_) != VK_SUCCESS) {
        throw std::runtime_error("vkCreateCommandPool failed");
    }

    VkCommandBufferAllocateInfo ai{};
    ai.sType              = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    ai.commandPool        = command_pool_;
    ai.level              = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    ai.commandBufferCount = kFramesInFlight;
    if (vkAllocateCommandBuffers(ctx_.device(), &ai, command_buffers_.data()) != VK_SUCCESS) {
        throw std::runtime_error("vkAllocateCommandBuffers failed");
    }

    VkSemaphoreCreateInfo si{};
    si.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
    VkFenceCreateInfo fi{};
    fi.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    fi.flags = VK_FENCE_CREATE_SIGNALED_BIT;
    for (uint32_t i = 0; i < kFramesInFlight; ++i) {
        if (vkCreateSemaphore(ctx_.device(), &si, nullptr, &image_available_[i]) != VK_SUCCESS
         || vkCreateSemaphore(ctx_.device(), &si, nullptr, &render_finished_[i]) != VK_SUCCESS
         || vkCreateFence    (ctx_.device(), &fi, nullptr, &in_flight_fences_[i]) != VK_SUCCESS) {
            throw std::runtime_error("frame sync object creation failed");
        }
    }
}

void Renderer::destroy_frame_resources() {
    for (uint32_t i = 0; i < kFramesInFlight; ++i) {
        if (image_available_[i])  vkDestroySemaphore(ctx_.device(), image_available_[i],  nullptr);
        if (render_finished_[i])  vkDestroySemaphore(ctx_.device(), render_finished_[i],  nullptr);
        if (in_flight_fences_[i]) vkDestroyFence    (ctx_.device(), in_flight_fences_[i], nullptr);
    }
    if (command_pool_) vkDestroyCommandPool(ctx_.device(), command_pool_, nullptr);
}

void Renderer::create_depth(VkExtent2D extent) {
    VkImageCreateInfo ici{};
    ici.sType         = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    ici.imageType     = VK_IMAGE_TYPE_2D;
    ici.format        = depth_format_;
    ici.extent        = { extent.width, extent.height, 1 };
    ici.mipLevels     = 1;
    ici.arrayLayers   = 1;
    ici.samples       = VK_SAMPLE_COUNT_1_BIT;
    ici.tiling        = VK_IMAGE_TILING_OPTIMAL;
    ici.usage         = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
    ici.sharingMode   = VK_SHARING_MODE_EXCLUSIVE;
    ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    if (vkCreateImage(ctx_.device(), &ici, nullptr, &depth_image_) != VK_SUCCESS) {
        throw std::runtime_error("depth image creation failed");
    }

    VkMemoryRequirements mr{};
    vkGetImageMemoryRequirements(ctx_.device(), depth_image_, &mr);
    VkMemoryAllocateInfo ai{};
    ai.sType           = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    ai.allocationSize  = mr.size;
    ai.memoryTypeIndex = find_memory_type(ctx_.physical_device(), mr.memoryTypeBits,
                                          VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    if (vkAllocateMemory(ctx_.device(), &ai, nullptr, &depth_memory_) != VK_SUCCESS) {
        throw std::runtime_error("depth memory allocation failed");
    }
    vkBindImageMemory(ctx_.device(), depth_image_, depth_memory_, 0);

    VkImageViewCreateInfo ivi{};
    ivi.sType    = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    ivi.image    = depth_image_;
    ivi.viewType = VK_IMAGE_VIEW_TYPE_2D;
    ivi.format   = depth_format_;
    ivi.subresourceRange.aspectMask     = VK_IMAGE_ASPECT_DEPTH_BIT;
    ivi.subresourceRange.baseMipLevel   = 0;
    ivi.subresourceRange.levelCount     = 1;
    ivi.subresourceRange.baseArrayLayer = 0;
    ivi.subresourceRange.layerCount     = 1;
    if (vkCreateImageView(ctx_.device(), &ivi, nullptr, &depth_view_) != VK_SUCCESS) {
        throw std::runtime_error("depth image view creation failed");
    }
    depth_extent_ = extent;
}

void Renderer::destroy_depth() {
    if (depth_view_)   vkDestroyImageView(ctx_.device(), depth_view_,   nullptr);
    if (depth_image_)  vkDestroyImage    (ctx_.device(), depth_image_,  nullptr);
    if (depth_memory_) vkFreeMemory      (ctx_.device(), depth_memory_, nullptr);
    depth_view_   = VK_NULL_HANDLE;
    depth_image_  = VK_NULL_HANDLE;
    depth_memory_ = VK_NULL_HANDLE;
    depth_extent_ = {0, 0};
}

bool Renderer::begin_frame(FrameContext& out) {
    vkWaitForFences(ctx_.device(), 1, &in_flight_fences_[current_frame_], VK_TRUE, UINT64_MAX);

    uint32_t image_index = 0;
    VkResult acquire = vkAcquireNextImageKHR(
        ctx_.device(), ctx_.swapchain(), UINT64_MAX,
        image_available_[current_frame_], VK_NULL_HANDLE, &image_index);

    if (acquire == VK_ERROR_OUT_OF_DATE_KHR) {
        ctx_.recreate_swapchain(window_);
        return false;
    }
    if (acquire != VK_SUCCESS && acquire != VK_SUBOPTIMAL_KHR) {
        throw std::runtime_error("vkAcquireNextImageKHR failed");
    }

    vkResetFences(ctx_.device(), 1, &in_flight_fences_[current_frame_]);

    VkExtent2D extent = ctx_.swapchain_extent();
    if (extent.width != depth_extent_.width || extent.height != depth_extent_.height) {
        destroy_depth();
        create_depth(extent);
    }

    VkCommandBuffer cmd = command_buffers_[current_frame_];
    vkResetCommandBuffer(cmd, 0);

    VkCommandBufferBeginInfo bi{};
    bi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(cmd, &bi);

    image_barrier(cmd, ctx_.swapchain_images()[image_index],
                  VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                  VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,           0,
                  VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);

    {
        VkImageMemoryBarrier2 b{};
        b.sType            = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
        b.srcStageMask     = VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT;
        b.srcAccessMask    = 0;
        b.dstStageMask     = VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT;
        b.dstAccessMask    = VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
        b.oldLayout        = VK_IMAGE_LAYOUT_UNDEFINED;
        b.newLayout        = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
        b.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        b.image            = depth_image_;
        b.subresourceRange.aspectMask     = VK_IMAGE_ASPECT_DEPTH_BIT;
        b.subresourceRange.baseMipLevel   = 0;
        b.subresourceRange.levelCount     = 1;
        b.subresourceRange.baseArrayLayer = 0;
        b.subresourceRange.layerCount     = 1;
        VkDependencyInfo dep{};
        dep.sType                   = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
        dep.imageMemoryBarrierCount = 1;
        dep.pImageMemoryBarriers    = &b;
        vkCmdPipelineBarrier2(cmd, &dep);
    }

    VkRenderingAttachmentInfo color{};
    color.sType              = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
    color.imageView          = ctx_.swapchain_image_views()[image_index];
    color.imageLayout        = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    color.loadOp             = VK_ATTACHMENT_LOAD_OP_CLEAR;
    color.storeOp            = VK_ATTACHMENT_STORE_OP_STORE;
    color.clearValue.color   = { { 0.05f, 0.06f, 0.08f, 1.0f } };

    VkRenderingAttachmentInfo depth{};
    depth.sType                    = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
    depth.imageView                = depth_view_;
    depth.imageLayout              = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
    depth.loadOp                   = VK_ATTACHMENT_LOAD_OP_CLEAR;
    depth.storeOp                  = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depth.clearValue.depthStencil  = { 1.0f, 0 };

    VkRenderingInfo ri{};
    ri.sType                = VK_STRUCTURE_TYPE_RENDERING_INFO;
    ri.renderArea.offset    = {0, 0};
    ri.renderArea.extent    = ctx_.swapchain_extent();
    ri.layerCount           = 1;
    ri.colorAttachmentCount = 1;
    ri.pColorAttachments    = &color;
    ri.pDepthAttachment     = &depth;
    vkCmdBeginRendering(cmd, &ri);

    VkViewport vp{};
    vp.x = 0.0f; vp.y = 0.0f;
    vp.width  = static_cast<float>(ctx_.swapchain_extent().width);
    vp.height = static_cast<float>(ctx_.swapchain_extent().height);
    vp.minDepth = 0.0f; vp.maxDepth = 1.0f;
    vkCmdSetViewport(cmd, 0, 1, &vp);

    VkRect2D scissor{ {0, 0}, ctx_.swapchain_extent() };
    vkCmdSetScissor(cmd, 0, 1, &scissor);

    out.cmd         = cmd;
    out.image_index = image_index;
    out.frame_index = current_frame_;
    out.extent      = ctx_.swapchain_extent();
    return true;
}

void Renderer::end_frame(FrameContext& frame) {
    VkCommandBuffer cmd = frame.cmd;
    vkCmdEndRendering(cmd);

    image_barrier(cmd, ctx_.swapchain_images()[frame.image_index],
                  VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
                  VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                  VK_PIPELINE_STAGE_2_BOTTOM_OF_PIPE_BIT,          0);

    vkEndCommandBuffer(cmd);

    VkSemaphoreSubmitInfo wait{};
    wait.sType     = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO;
    wait.semaphore = image_available_[current_frame_];
    wait.stageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;

    VkSemaphoreSubmitInfo signal{};
    signal.sType     = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO;
    signal.semaphore = render_finished_[current_frame_];
    signal.stageMask = VK_PIPELINE_STAGE_2_ALL_GRAPHICS_BIT;

    VkCommandBufferSubmitInfo cmdsi{};
    cmdsi.sType         = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO;
    cmdsi.commandBuffer = cmd;

    VkSubmitInfo2 si{};
    si.sType                    = VK_STRUCTURE_TYPE_SUBMIT_INFO_2;
    si.waitSemaphoreInfoCount   = 1;
    si.pWaitSemaphoreInfos      = &wait;
    si.commandBufferInfoCount   = 1;
    si.pCommandBufferInfos      = &cmdsi;
    si.signalSemaphoreInfoCount = 1;
    si.pSignalSemaphoreInfos    = &signal;

    if (vkQueueSubmit2(ctx_.graphics_queue(), 1, &si, in_flight_fences_[current_frame_]) != VK_SUCCESS) {
        throw std::runtime_error("vkQueueSubmit2 failed");
    }

    VkPresentInfoKHR pi{};
    pi.sType              = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
    pi.waitSemaphoreCount = 1;
    pi.pWaitSemaphores    = &render_finished_[current_frame_];
    VkSwapchainKHR sc = ctx_.swapchain();
    pi.swapchainCount     = 1;
    pi.pSwapchains        = &sc;
    pi.pImageIndices      = &frame.image_index;

    VkResult present = vkQueuePresentKHR(ctx_.present_queue(), &pi);
    if (present == VK_ERROR_OUT_OF_DATE_KHR || present == VK_SUBOPTIMAL_KHR || framebuffer_resized_) {
        framebuffer_resized_ = false;
        ctx_.recreate_swapchain(window_);
    } else if (present != VK_SUCCESS) {
        throw std::runtime_error("vkQueuePresentKHR failed");
    }

    current_frame_ = (current_frame_ + 1) % kFramesInFlight;
}

} // namespace finalLab::render
