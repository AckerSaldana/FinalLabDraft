#include "render/instance_renderer.hpp"

#include "render/vulkan_context.hpp"

#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace finalLab::render {

namespace {

std::vector<uint32_t> load_spirv(const std::filesystem::path& path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) throw std::runtime_error("shader not found: " + path.string());
    size_t size = static_cast<size_t>(f.tellg());
    if (size % 4 != 0) throw std::runtime_error("spv size not 4-aligned: " + path.string());
    std::vector<uint32_t> data(size / 4);
    f.seekg(0);
    f.read(reinterpret_cast<char*>(data.data()), size);
    return data;
}

VkShaderModule create_shader_module(VkDevice device, const std::vector<uint32_t>& code) {
    VkShaderModuleCreateInfo ci{};
    ci.sType    = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    ci.codeSize = code.size() * sizeof(uint32_t);
    ci.pCode    = code.data();
    VkShaderModule m = VK_NULL_HANDLE;
    if (vkCreateShaderModule(device, &ci, nullptr, &m) != VK_SUCCESS) {
        throw std::runtime_error("vkCreateShaderModule failed");
    }
    return m;
}

std::filesystem::path shader_path(const char* name) {
    return std::filesystem::path("shaders") / name;
}

size_t mesh_slot(ShapeKind s, bool container) {
    return static_cast<size_t>(s) * 2 + (container ? 1 : 0);
}

} // namespace

InstanceRenderer::InstanceRenderer(VulkanContext& ctx, VkFormat color_format, VkFormat depth_format)
    : ctx_(ctx) {
    VkCommandPoolCreateInfo pi{};
    pi.sType            = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    pi.flags            = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
    pi.queueFamilyIndex = ctx_.queue_families().graphics;
    if (vkCreateCommandPool(ctx_.device(), &pi, nullptr, &upload_pool_) != VK_SUCCESS) {
        throw std::runtime_error("upload command pool creation failed");
    }

    create_meshes(ctx_.graphics_queue(), upload_pool_);
    create_descriptors();
    create_per_frame_buffers();
    create_pipeline(color_format, depth_format);
}

InstanceRenderer::~InstanceRenderer() {
    vkDeviceWaitIdle(ctx_.device());
    for (auto& m : meshes_) destroy_mesh(ctx_.device(), m);
    for (auto& b : instance_bufs_) destroy_buffer(ctx_.device(), b);
    if (pipeline_)        vkDestroyPipeline      (ctx_.device(), pipeline_,        nullptr);
    if (pipeline_layout_) vkDestroyPipelineLayout(ctx_.device(), pipeline_layout_, nullptr);
    if (desc_pool_)       vkDestroyDescriptorPool(ctx_.device(), desc_pool_,       nullptr);
    if (set_layout_)      vkDestroyDescriptorSetLayout(ctx_.device(), set_layout_, nullptr);
    if (upload_pool_)     vkDestroyCommandPool   (ctx_.device(), upload_pool_,     nullptr);
}

void InstanceRenderer::create_meshes(VkQueue queue, VkCommandPool pool) {
    MeshData data[static_cast<size_t>(ShapeKind::Count)] = {
        make_sphere(),
        make_cuboid(),
        make_cylinder(),
        make_capsule(),
        make_plane(),
    };
    for (size_t i = 0; i < static_cast<size_t>(ShapeKind::Count); ++i) {
        meshes_[i * 2]     = upload_mesh(ctx_.device(), ctx_.physical_device(), queue, pool, data[i]);
        MeshData inv = make_container(data[i]);
        meshes_[i * 2 + 1] = upload_mesh(ctx_.device(), ctx_.physical_device(), queue, pool, inv);
    }
}

void InstanceRenderer::create_descriptors() {
    VkDescriptorSetLayoutBinding binding{};
    binding.binding         = 0;
    binding.descriptorType  = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    binding.descriptorCount = 1;
    binding.stageFlags      = VK_SHADER_STAGE_VERTEX_BIT;

    VkDescriptorSetLayoutCreateInfo lci{};
    lci.sType        = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    lci.bindingCount = 1;
    lci.pBindings    = &binding;
    if (vkCreateDescriptorSetLayout(ctx_.device(), &lci, nullptr, &set_layout_) != VK_SUCCESS) {
        throw std::runtime_error("descriptor set layout creation failed");
    }

    VkDescriptorPoolSize size{ VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, kFramesInFlight };
    VkDescriptorPoolCreateInfo pci{};
    pci.sType         = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    pci.flags         = VK_DESCRIPTOR_POOL_CREATE_UPDATE_AFTER_BIND_BIT;
    pci.maxSets       = kFramesInFlight;
    pci.poolSizeCount = 1;
    pci.pPoolSizes    = &size;
    if (vkCreateDescriptorPool(ctx_.device(), &pci, nullptr, &desc_pool_) != VK_SUCCESS) {
        throw std::runtime_error("descriptor pool creation failed");
    }

    std::array<VkDescriptorSetLayout, kFramesInFlight> layouts;
    layouts.fill(set_layout_);

    VkDescriptorSetAllocateInfo ai{};
    ai.sType              = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    ai.descriptorPool     = desc_pool_;
    ai.descriptorSetCount = kFramesInFlight;
    ai.pSetLayouts        = layouts.data();
    if (vkAllocateDescriptorSets(ctx_.device(), &ai, desc_sets_.data()) != VK_SUCCESS) {
        throw std::runtime_error("descriptor set allocation failed");
    }
}

void InstanceRenderer::create_per_frame_buffers() {
    constexpr VkDeviceSize initial = sizeof(InstanceGPU) * 256;
    for (uint32_t i = 0; i < kFramesInFlight; ++i) {
        instance_bufs_[i] = create_buffer(ctx_.device(), ctx_.physical_device(), initial,
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        instance_cap_[i] = initial;
        update_descriptor_for_frame(i);
    }
}

void InstanceRenderer::ensure_instance_capacity(uint32_t frame_index, VkDeviceSize required_bytes) {
    if (instance_cap_[frame_index] >= required_bytes) return;

    VkDeviceSize new_cap = instance_cap_[frame_index];
    while (new_cap < required_bytes) new_cap *= 2;

    vkDeviceWaitIdle(ctx_.device());
    destroy_buffer(ctx_.device(), instance_bufs_[frame_index]);
    instance_bufs_[frame_index] = create_buffer(ctx_.device(), ctx_.physical_device(), new_cap,
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    instance_cap_[frame_index] = new_cap;
    update_descriptor_for_frame(frame_index);
}

void InstanceRenderer::update_descriptor_for_frame(uint32_t frame_index) {
    VkDescriptorBufferInfo bi{};
    bi.buffer = instance_bufs_[frame_index].buffer;
    bi.offset = 0;
    bi.range  = VK_WHOLE_SIZE;

    VkWriteDescriptorSet w{};
    w.sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    w.dstSet          = desc_sets_[frame_index];
    w.dstBinding      = 0;
    w.descriptorCount = 1;
    w.descriptorType  = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    w.pBufferInfo     = &bi;
    vkUpdateDescriptorSets(ctx_.device(), 1, &w, 0, nullptr);
}

void InstanceRenderer::create_pipeline(VkFormat color_format, VkFormat depth_format) {
    auto vert_spv = load_spirv(shader_path("instance.vert.spv"));
    auto frag_spv = load_spirv(shader_path("instance.frag.spv"));
    VkShaderModule vert_mod = create_shader_module(ctx_.device(), vert_spv);
    VkShaderModule frag_mod = create_shader_module(ctx_.device(), frag_spv);

    VkPushConstantRange push{};
    push.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    push.offset     = 0;
    push.size       = sizeof(glm::mat4);

    VkPipelineLayoutCreateInfo lci{};
    lci.sType                  = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    lci.setLayoutCount         = 1;
    lci.pSetLayouts            = &set_layout_;
    lci.pushConstantRangeCount = 1;
    lci.pPushConstantRanges    = &push;
    if (vkCreatePipelineLayout(ctx_.device(), &lci, nullptr, &pipeline_layout_) != VK_SUCCESS) {
        throw std::runtime_error("pipeline layout creation failed");
    }

    VkPipelineShaderStageCreateInfo stages[2]{};
    stages[0].sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[0].stage  = VK_SHADER_STAGE_VERTEX_BIT;
    stages[0].module = vert_mod;
    stages[0].pName  = "main";
    stages[1].sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[1].stage  = VK_SHADER_STAGE_FRAGMENT_BIT;
    stages[1].module = frag_mod;
    stages[1].pName  = "main";

    VkVertexInputBindingDescription vbinding{};
    vbinding.binding   = 0;
    vbinding.stride    = sizeof(Vertex);
    vbinding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    VkVertexInputAttributeDescription vattrs[2]{};
    vattrs[0].location = 0; vattrs[0].binding = 0;
    vattrs[0].format   = VK_FORMAT_R32G32B32_SFLOAT;
    vattrs[0].offset   = offsetof(Vertex, position);
    vattrs[1].location = 1; vattrs[1].binding = 0;
    vattrs[1].format   = VK_FORMAT_R32G32B32_SFLOAT;
    vattrs[1].offset   = offsetof(Vertex, normal);

    VkPipelineVertexInputStateCreateInfo vi{};
    vi.sType                           = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
    vi.vertexBindingDescriptionCount   = 1;
    vi.pVertexBindingDescriptions      = &vbinding;
    vi.vertexAttributeDescriptionCount = 2;
    vi.pVertexAttributeDescriptions    = vattrs;

    VkPipelineInputAssemblyStateCreateInfo ia{};
    ia.sType    = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    ia.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkPipelineViewportStateCreateInfo vp{};
    vp.sType         = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
    vp.viewportCount = 1;
    vp.scissorCount  = 1;

    VkPipelineRasterizationStateCreateInfo rs{};
    rs.sType       = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    rs.polygonMode = VK_POLYGON_MODE_FILL;
    rs.cullMode    = VK_CULL_MODE_BACK_BIT;
    rs.frontFace   = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rs.lineWidth   = 1.0f;

    VkPipelineMultisampleStateCreateInfo ms{};
    ms.sType                = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
    ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

    VkPipelineDepthStencilStateCreateInfo ds{};
    ds.sType            = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
    ds.depthTestEnable  = VK_TRUE;
    ds.depthWriteEnable = VK_TRUE;
    ds.depthCompareOp   = VK_COMPARE_OP_LESS;

    VkPipelineColorBlendAttachmentState ba{};
    ba.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT
                      | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    ba.blendEnable    = VK_FALSE;
    VkPipelineColorBlendStateCreateInfo cb{};
    cb.sType           = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
    cb.attachmentCount = 1;
    cb.pAttachments    = &ba;

    VkDynamicState dyn_states[] = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
    VkPipelineDynamicStateCreateInfo dyn{};
    dyn.sType             = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
    dyn.dynamicStateCount = 2;
    dyn.pDynamicStates    = dyn_states;

    VkPipelineRenderingCreateInfo rci{};
    rci.sType                   = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
    rci.colorAttachmentCount    = 1;
    rci.pColorAttachmentFormats = &color_format;
    rci.depthAttachmentFormat   = depth_format;

    VkGraphicsPipelineCreateInfo gci{};
    gci.sType               = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
    gci.pNext               = &rci;
    gci.stageCount          = 2;
    gci.pStages             = stages;
    gci.pVertexInputState   = &vi;
    gci.pInputAssemblyState = &ia;
    gci.pViewportState      = &vp;
    gci.pRasterizationState = &rs;
    gci.pMultisampleState   = &ms;
    gci.pDepthStencilState  = &ds;
    gci.pColorBlendState    = &cb;
    gci.pDynamicState       = &dyn;
    gci.layout              = pipeline_layout_;

    VkResult r = vkCreateGraphicsPipelines(ctx_.device(), VK_NULL_HANDLE, 1, &gci, nullptr, &pipeline_);

    vkDestroyShaderModule(ctx_.device(), vert_mod, nullptr);
    vkDestroyShaderModule(ctx_.device(), frag_mod, nullptr);

    if (r != VK_SUCCESS) throw std::runtime_error("graphics pipeline creation failed");
}

void InstanceRenderer::draw(uint32_t frame_index, VkCommandBuffer cmd,
                             const glm::mat4& view_proj,
                             std::span<const RenderInstance> instances) {
    if (instances.empty()) return;

    std::array<std::vector<InstanceGPU>, static_cast<size_t>(ShapeKind::Count) * 2> buckets;
    for (const RenderInstance& ri : instances) {
        size_t slot = mesh_slot(ri.shape, ri.container);
        buckets[slot].push_back({ ri.model, ri.color });
    }

    std::vector<InstanceGPU> packed;
    packed.reserve(instances.size());
    std::array<uint32_t, buckets.size()> bucket_offsets{};
    std::array<uint32_t, buckets.size()> bucket_counts{};
    for (size_t i = 0; i < buckets.size(); ++i) {
        bucket_offsets[i] = static_cast<uint32_t>(packed.size());
        bucket_counts[i]  = static_cast<uint32_t>(buckets[i].size());
        packed.insert(packed.end(), buckets[i].begin(), buckets[i].end());
    }

    VkDeviceSize needed = sizeof(InstanceGPU) * packed.size();
    ensure_instance_capacity(frame_index, needed);

    void* mapped = map_buffer(ctx_.device(), instance_bufs_[frame_index]);
    std::memcpy(mapped, packed.data(), static_cast<size_t>(needed));
    unmap_buffer(ctx_.device(), instance_bufs_[frame_index]);

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_layout_,
                            0, 1, &desc_sets_[frame_index], 0, nullptr);
    vkCmdPushConstants(cmd, pipeline_layout_, VK_SHADER_STAGE_VERTEX_BIT,
                       0, sizeof(glm::mat4), &view_proj);

    for (size_t slot = 0; slot < buckets.size(); ++slot) {
        if (bucket_counts[slot] == 0) continue;
        const Mesh& mesh = meshes_[slot];
        VkDeviceSize voff = 0;
        vkCmdBindVertexBuffers(cmd, 0, 1, &mesh.vbuf.buffer, &voff);
        vkCmdBindIndexBuffer(cmd, mesh.ibuf.buffer, 0, VK_INDEX_TYPE_UINT32);
        vkCmdDrawIndexed(cmd, mesh.index_count, bucket_counts[slot], 0, 0, bucket_offsets[slot]);
    }
}

} // namespace finalLab::render
