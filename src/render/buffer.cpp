#include "render/buffer.hpp"

#include <cstring>
#include <stdexcept>

namespace finalLab::render {

uint32_t find_memory_type(VkPhysicalDevice pd, uint32_t type_bits, VkMemoryPropertyFlags props) {
    VkPhysicalDeviceMemoryProperties mem{};
    vkGetPhysicalDeviceMemoryProperties(pd, &mem);
    for (uint32_t i = 0; i < mem.memoryTypeCount; ++i) {
        if ((type_bits & (1u << i)) && (mem.memoryTypes[i].propertyFlags & props) == props) {
            return i;
        }
    }
    throw std::runtime_error("no matching memory type");
}

Buffer create_buffer(VkDevice device, VkPhysicalDevice pd,
                     VkDeviceSize size, VkBufferUsageFlags usage,
                     VkMemoryPropertyFlags props) {
    Buffer b{};
    b.size = size;

    VkBufferCreateInfo bi{};
    bi.sType       = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    bi.size        = size;
    bi.usage       = usage;
    bi.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    if (vkCreateBuffer(device, &bi, nullptr, &b.buffer) != VK_SUCCESS) {
        throw std::runtime_error("vkCreateBuffer failed");
    }

    VkMemoryRequirements mr{};
    vkGetBufferMemoryRequirements(device, b.buffer, &mr);

    VkMemoryAllocateInfo ai{};
    ai.sType           = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    ai.allocationSize  = mr.size;
    ai.memoryTypeIndex = find_memory_type(pd, mr.memoryTypeBits, props);
    if (vkAllocateMemory(device, &ai, nullptr, &b.memory) != VK_SUCCESS) {
        vkDestroyBuffer(device, b.buffer, nullptr);
        throw std::runtime_error("vkAllocateMemory failed");
    }

    vkBindBufferMemory(device, b.buffer, b.memory, 0);
    return b;
}

void destroy_buffer(VkDevice device, Buffer& b) {
    if (b.buffer) { vkDestroyBuffer(device, b.buffer, nullptr); b.buffer = VK_NULL_HANDLE; }
    if (b.memory) { vkFreeMemory   (device, b.memory, nullptr); b.memory = VK_NULL_HANDLE; }
    b.size = 0;
}

void upload_to_device_local(VkDevice device, VkPhysicalDevice pd,
                            VkQueue queue, VkCommandPool pool,
                            Buffer& dst, const void* data, VkDeviceSize size) {
    Buffer staging = create_buffer(device, pd, size,
        VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

    void* mapped = nullptr;
    vkMapMemory(device, staging.memory, 0, size, 0, &mapped);
    std::memcpy(mapped, data, static_cast<size_t>(size));
    vkUnmapMemory(device, staging.memory);

    VkCommandBufferAllocateInfo cbai{};
    cbai.sType              = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    cbai.commandPool        = pool;
    cbai.level              = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cbai.commandBufferCount = 1;
    VkCommandBuffer cmd = VK_NULL_HANDLE;
    vkAllocateCommandBuffers(device, &cbai, &cmd);

    VkCommandBufferBeginInfo bi{};
    bi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(cmd, &bi);
    VkBufferCopy copy{ 0, 0, size };
    vkCmdCopyBuffer(cmd, staging.buffer, dst.buffer, 1, &copy);
    vkEndCommandBuffer(cmd);

    VkSubmitInfo si{};
    si.sType              = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    si.commandBufferCount = 1;
    si.pCommandBuffers    = &cmd;
    vkQueueSubmit(queue, 1, &si, VK_NULL_HANDLE);
    vkQueueWaitIdle(queue);

    vkFreeCommandBuffers(device, pool, 1, &cmd);
    destroy_buffer(device, staging);
}

void* map_buffer(VkDevice device, const Buffer& b) {
    void* mapped = nullptr;
    if (vkMapMemory(device, b.memory, 0, b.size, 0, &mapped) != VK_SUCCESS) {
        throw std::runtime_error("vkMapMemory failed");
    }
    return mapped;
}

void unmap_buffer(VkDevice device, const Buffer& b) {
    vkUnmapMemory(device, b.memory);
}

} // namespace finalLab::render
