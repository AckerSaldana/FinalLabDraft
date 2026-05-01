#pragma once

#include <vulkan/vulkan.h>

#include <cstddef>
#include <cstdint>

namespace finalLab::render {

struct Buffer {
    VkBuffer       buffer = VK_NULL_HANDLE;
    VkDeviceMemory memory = VK_NULL_HANDLE;
    VkDeviceSize   size   = 0;
};

uint32_t find_memory_type(VkPhysicalDevice pd, uint32_t type_bits, VkMemoryPropertyFlags props);

Buffer create_buffer(VkDevice device, VkPhysicalDevice pd,
                     VkDeviceSize size, VkBufferUsageFlags usage,
                     VkMemoryPropertyFlags props);

void destroy_buffer(VkDevice device, Buffer& b);

void upload_to_device_local(VkDevice device, VkPhysicalDevice pd,
                            VkQueue queue, VkCommandPool pool,
                            Buffer& dst, const void* data, VkDeviceSize size);

void* map_buffer(VkDevice device, const Buffer& b);
void  unmap_buffer(VkDevice device, const Buffer& b);

} // namespace finalLab::render
