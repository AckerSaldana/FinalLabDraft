#include "render/compute_pipeline.hpp"

#include "render/vulkan_context.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace finalLab::render {

std::vector<uint32_t> load_spirv(const std::filesystem::path& path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) throw std::runtime_error("compute pipeline: failed to open " + path.string());
    const std::streamsize sz = f.tellg();
    if (sz <= 0 || (sz % 4) != 0) {
        throw std::runtime_error("compute pipeline: SPIR-V file has bad size: " + path.string());
    }
    std::vector<uint32_t> out(static_cast<size_t>(sz) / 4);
    f.seekg(0);
    f.read(reinterpret_cast<char*>(out.data()), sz);
    return out;
}

BoidComputePipeline::BoidComputePipeline(VulkanContext& ctx, const std::filesystem::path& spv_path)
    : ctx_(ctx), device_(ctx.device()) {
    create_descriptor_layout();
    create_pipeline(spv_path);
    create_command_pool();
    create_descriptor_pool_and_set();
    // Lazy-allocate the SSBO on the first ensure_capacity() call so we do not
    // pay for a 0-byte allocation if the user never enables GPU steering.
}

BoidComputePipeline::~BoidComputePipeline() {
    destroy_buffer_resources();
    if (command_pool_)     { vkDestroyCommandPool(device_, command_pool_, nullptr);              command_pool_ = VK_NULL_HANDLE; }
    if (descriptor_pool_)  { vkDestroyDescriptorPool(device_, descriptor_pool_, nullptr);        descriptor_pool_ = VK_NULL_HANDLE; }
    if (pipeline_)         { vkDestroyPipeline(device_, pipeline_, nullptr);                     pipeline_ = VK_NULL_HANDLE; }
    if (pipeline_layout_)  { vkDestroyPipelineLayout(device_, pipeline_layout_, nullptr);        pipeline_layout_ = VK_NULL_HANDLE; }
    if (set_layout_)       { vkDestroyDescriptorSetLayout(device_, set_layout_, nullptr);        set_layout_ = VK_NULL_HANDLE; }
    if (shader_)           { vkDestroyShaderModule(device_, shader_, nullptr);                   shader_ = VK_NULL_HANDLE; }
}

void BoidComputePipeline::create_descriptor_layout() {
    VkDescriptorSetLayoutBinding b{};
    b.binding         = 0;
    b.descriptorType  = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    b.descriptorCount = 1;
    b.stageFlags      = VK_SHADER_STAGE_COMPUTE_BIT;

    VkDescriptorSetLayoutCreateInfo ci{};
    ci.sType        = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    ci.bindingCount = 1;
    ci.pBindings    = &b;
    if (vkCreateDescriptorSetLayout(device_, &ci, nullptr, &set_layout_) != VK_SUCCESS) {
        throw std::runtime_error("compute pipeline: vkCreateDescriptorSetLayout failed");
    }
}

void BoidComputePipeline::create_pipeline(const std::filesystem::path& spv_path) {
    auto code = load_spirv(spv_path);

    VkShaderModuleCreateInfo smi{};
    smi.sType    = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    smi.codeSize = code.size() * sizeof(uint32_t);
    smi.pCode    = code.data();
    if (vkCreateShaderModule(device_, &smi, nullptr, &shader_) != VK_SUCCESS) {
        throw std::runtime_error("compute pipeline: vkCreateShaderModule failed for boids.comp");
    }

    VkPushConstantRange pcr{};
    pcr.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    pcr.offset     = 0;
    pcr.size       = sizeof(PushConstants);

    VkPipelineLayoutCreateInfo pli{};
    pli.sType                  = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pli.setLayoutCount         = 1;
    pli.pSetLayouts            = &set_layout_;
    pli.pushConstantRangeCount = 1;
    pli.pPushConstantRanges    = &pcr;
    if (vkCreatePipelineLayout(device_, &pli, nullptr, &pipeline_layout_) != VK_SUCCESS) {
        throw std::runtime_error("compute pipeline: vkCreatePipelineLayout failed");
    }

    VkPipelineShaderStageCreateInfo ssi{};
    ssi.sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    ssi.stage  = VK_SHADER_STAGE_COMPUTE_BIT;
    ssi.module = shader_;
    ssi.pName  = "main";

    VkComputePipelineCreateInfo cpi{};
    cpi.sType  = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    cpi.stage  = ssi;
    cpi.layout = pipeline_layout_;
    if (vkCreateComputePipelines(device_, VK_NULL_HANDLE, 1, &cpi, nullptr, &pipeline_) != VK_SUCCESS) {
        throw std::runtime_error("compute pipeline: vkCreateComputePipelines failed");
    }
}

void BoidComputePipeline::create_command_pool() {
    VkCommandPoolCreateInfo ci{};
    ci.sType            = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    ci.flags            = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    ci.queueFamilyIndex = ctx_.queue_families().graphics;
    if (vkCreateCommandPool(device_, &ci, nullptr, &command_pool_) != VK_SUCCESS) {
        throw std::runtime_error("compute pipeline: vkCreateCommandPool failed");
    }

    VkCommandBufferAllocateInfo ai{};
    ai.sType              = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    ai.commandPool        = command_pool_;
    ai.level              = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    ai.commandBufferCount = 1;
    if (vkAllocateCommandBuffers(device_, &ai, &command_buffer_) != VK_SUCCESS) {
        throw std::runtime_error("compute pipeline: vkAllocateCommandBuffers failed");
    }
}

void BoidComputePipeline::create_descriptor_pool_and_set() {
    VkDescriptorPoolSize ps{};
    ps.type            = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    ps.descriptorCount = 1;

    VkDescriptorPoolCreateInfo dpi{};
    dpi.sType         = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    dpi.maxSets       = 1;
    dpi.poolSizeCount = 1;
    dpi.pPoolSizes    = &ps;
    if (vkCreateDescriptorPool(device_, &dpi, nullptr, &descriptor_pool_) != VK_SUCCESS) {
        throw std::runtime_error("compute pipeline: vkCreateDescriptorPool failed");
    }

    VkDescriptorSetAllocateInfo dai{};
    dai.sType              = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    dai.descriptorPool     = descriptor_pool_;
    dai.descriptorSetCount = 1;
    dai.pSetLayouts        = &set_layout_;
    if (vkAllocateDescriptorSets(device_, &dai, &descriptor_set_) != VK_SUCCESS) {
        throw std::runtime_error("compute pipeline: vkAllocateDescriptorSets failed");
    }
}

void BoidComputePipeline::destroy_buffer_resources() {
    if (ssbo_.buffer) destroy_buffer(device_, ssbo_);
    capacity_ = 0;
}

void BoidComputePipeline::allocate_buffer(uint32_t n) {
    destroy_buffer_resources();
    if (n == 0) return;
    VkDeviceSize size = static_cast<VkDeviceSize>(n) * sizeof(BoidGPU);
    ssbo_ = create_buffer(device_, ctx_.physical_device(), size,
                          VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                          VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    capacity_ = n;
    update_descriptor_set();
}

void BoidComputePipeline::update_descriptor_set() {
    if (!ssbo_.buffer) return;
    VkDescriptorBufferInfo bi{};
    bi.buffer = ssbo_.buffer;
    bi.offset = 0;
    bi.range  = ssbo_.size;

    VkWriteDescriptorSet w{};
    w.sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    w.dstSet          = descriptor_set_;
    w.dstBinding      = 0;
    w.dstArrayElement = 0;
    w.descriptorType  = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    w.descriptorCount = 1;
    w.pBufferInfo     = &bi;
    vkUpdateDescriptorSets(device_, 1, &w, 0, nullptr);
}

void BoidComputePipeline::ensure_capacity(uint32_t n) {
    // Re-allocate at 1.5× growth to avoid thrash when scenes spawn boids.
    if (n <= capacity_) return;
    uint32_t target = std::max<uint32_t>(64, n + n / 2);
    allocate_buffer(target);
}

void BoidComputePipeline::run(std::vector<BoidGPU>& boids, PushConstants pc) {
    pc.count = static_cast<uint32_t>(boids.size());
    if (pc.count == 0) {
        last_dispatch_us_ = 0;
        return;
    }
    ensure_capacity(pc.count);

    // Upload current state.
    void* mapped = map_buffer(device_, ssbo_);
    std::memcpy(mapped, boids.data(), pc.count * sizeof(BoidGPU));
    unmap_buffer(device_, ssbo_);

    // Record + submit.
    auto t0 = std::chrono::steady_clock::now();

    vkResetCommandBuffer(command_buffer_, 0);

    VkCommandBufferBeginInfo cbi{};
    cbi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    cbi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(command_buffer_, &cbi);

    vkCmdBindPipeline(command_buffer_, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_);
    vkCmdBindDescriptorSets(command_buffer_, VK_PIPELINE_BIND_POINT_COMPUTE,
                            pipeline_layout_, 0, 1, &descriptor_set_, 0, nullptr);
    vkCmdPushConstants(command_buffer_, pipeline_layout_,
                       VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(PushConstants), &pc);

    uint32_t group_count = (pc.count + 63u) / 64u;
    vkCmdDispatch(command_buffer_, group_count, 1, 1);

    // Memory barrier so the host sees the shader's writes after queue-wait.
    VkMemoryBarrier mb{};
    mb.sType         = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
    mb.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
    mb.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
    vkCmdPipelineBarrier(command_buffer_,
                         VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                         VK_PIPELINE_STAGE_HOST_BIT,
                         0, 1, &mb, 0, nullptr, 0, nullptr);

    vkEndCommandBuffer(command_buffer_);

    VkSubmitInfo si{};
    si.sType              = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    si.commandBufferCount = 1;
    si.pCommandBuffers    = &command_buffer_;
    vkQueueSubmit(ctx_.graphics_queue(), 1, &si, VK_NULL_HANDLE);
    vkQueueWaitIdle(ctx_.graphics_queue());

    auto t1 = std::chrono::steady_clock::now();
    last_dispatch_us_ = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

    // Read back.
    mapped = map_buffer(device_, ssbo_);
    std::memcpy(boids.data(), mapped, pc.count * sizeof(BoidGPU));
    unmap_buffer(device_, ssbo_);
}

} // namespace finalLab::render
