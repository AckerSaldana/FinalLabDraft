#include "render/vulkan_context.hpp"

#include "core/window.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <set>
#include <stdexcept>

namespace finalLab::render {

namespace {

constexpr std::array<const char*, 1> kValidationLayers = {
    "VK_LAYER_KHRONOS_validation"
};

constexpr std::array<const char*, 1> kDeviceExtensions = {
    VK_KHR_SWAPCHAIN_EXTENSION_NAME
};

VKAPI_ATTR VkBool32 VKAPI_CALL debug_callback(
    VkDebugUtilsMessageSeverityFlagBitsEXT severity,
    VkDebugUtilsMessageTypeFlagsEXT,
    const VkDebugUtilsMessengerCallbackDataEXT* data,
    void*) {
    if (severity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT) {
        std::fprintf(stderr, "[vk] %s\n", data->pMessage);
    }
    return VK_FALSE;
}

bool validation_available() {
    uint32_t count = 0;
    vkEnumerateInstanceLayerProperties(&count, nullptr);
    std::vector<VkLayerProperties> layers(count);
    vkEnumerateInstanceLayerProperties(&count, layers.data());
    for (const char* want : kValidationLayers) {
        bool found = false;
        for (const auto& l : layers) if (std::strcmp(want, l.layerName) == 0) { found = true; break; }
        if (!found) return false;
    }
    return true;
}

QueueFamilies find_queue_families(VkPhysicalDevice pd, VkSurfaceKHR surface) {
    QueueFamilies qf;
    uint32_t count = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(pd, &count, nullptr);
    std::vector<VkQueueFamilyProperties> props(count);
    vkGetPhysicalDeviceQueueFamilyProperties(pd, &count, props.data());

    for (uint32_t i = 0; i < count; ++i) {
        if ((props[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) && qf.graphics == UINT32_MAX) {
            qf.graphics = i;
        }
        VkBool32 present = VK_FALSE;
        vkGetPhysicalDeviceSurfaceSupportKHR(pd, i, surface, &present);
        if (present && qf.present == UINT32_MAX) {
            qf.present = i;
        }
        if (qf.complete()) break;
    }
    return qf;
}

bool device_supports_extensions(VkPhysicalDevice pd) {
    uint32_t count = 0;
    vkEnumerateDeviceExtensionProperties(pd, nullptr, &count, nullptr);
    std::vector<VkExtensionProperties> exts(count);
    vkEnumerateDeviceExtensionProperties(pd, nullptr, &count, exts.data());
    for (const char* want : kDeviceExtensions) {
        bool found = false;
        for (const auto& e : exts) if (std::strcmp(want, e.extensionName) == 0) { found = true; break; }
        if (!found) return false;
    }
    return true;
}

bool device_supports_vulkan13_dynamic_rendering(VkPhysicalDevice pd) {
    VkPhysicalDeviceVulkan13Features v13{};
    v13.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES;
    VkPhysicalDeviceFeatures2 f2{};
    f2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
    f2.pNext = &v13;
    vkGetPhysicalDeviceFeatures2(pd, &f2);
    return v13.dynamicRendering && v13.synchronization2;
}

int score_device(VkPhysicalDevice pd, VkSurfaceKHR surface) {
    VkPhysicalDeviceProperties props{};
    vkGetPhysicalDeviceProperties(pd, &props);
    if (props.apiVersion < VK_API_VERSION_1_3) return -1;
    if (!find_queue_families(pd, surface).complete()) return -1;
    if (!device_supports_extensions(pd)) return -1;
    if (!device_supports_vulkan13_dynamic_rendering(pd)) return -1;

    int score = 0;
    if (props.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) score += 1000;
    score += static_cast<int>(props.limits.maxImageDimension2D);
    return score;
}

VkSurfaceFormatKHR pick_surface_format(const std::vector<VkSurfaceFormatKHR>& formats) {
    for (const auto& f : formats) {
        if (f.format == VK_FORMAT_B8G8R8A8_SRGB && f.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
            return f;
        }
    }
    return formats.front();
}

VkPresentModeKHR pick_present_mode(const std::vector<VkPresentModeKHR>& modes) {
    for (VkPresentModeKHR m : modes) {
        if (m == VK_PRESENT_MODE_MAILBOX_KHR) return m;
    }
    return VK_PRESENT_MODE_FIFO_KHR;
}

} // namespace

VulkanContext::VulkanContext(core::Window& window, bool enable_validation)
    : enable_validation_(enable_validation && validation_available()) {
    create_instance();
    if (enable_validation_) create_debug_messenger();
    surface_ = window.create_surface(instance_);
    pick_physical_device();
    create_logical_device();
    create_swapchain(window);
}

VulkanContext::~VulkanContext() {
    destroy_swapchain();
    if (device_) vkDestroyDevice(device_, nullptr);
    if (surface_) vkDestroySurfaceKHR(instance_, surface_, nullptr);
    if (debug_messenger_) {
        auto fp = reinterpret_cast<PFN_vkDestroyDebugUtilsMessengerEXT>(
            vkGetInstanceProcAddr(instance_, "vkDestroyDebugUtilsMessengerEXT"));
        if (fp) fp(instance_, debug_messenger_, nullptr);
    }
    if (instance_) vkDestroyInstance(instance_, nullptr);
}

void VulkanContext::create_instance() {
    VkApplicationInfo app{};
    app.sType              = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    app.pApplicationName   = "finalLab";
    app.applicationVersion = VK_MAKE_VERSION(0, 1, 0);
    app.pEngineName        = "finalLab";
    app.engineVersion      = VK_MAKE_VERSION(0, 1, 0);
    app.apiVersion         = VK_API_VERSION_1_3;

    uint32_t glfw_ext_count = 0;
    const char** glfw_exts = glfwGetRequiredInstanceExtensions(&glfw_ext_count);
    std::vector<const char*> extensions(glfw_exts, glfw_exts + glfw_ext_count);
    if (enable_validation_) extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);

    VkInstanceCreateInfo ci{};
    ci.sType                   = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    ci.pApplicationInfo        = &app;
    ci.enabledExtensionCount   = static_cast<uint32_t>(extensions.size());
    ci.ppEnabledExtensionNames = extensions.data();
    if (enable_validation_) {
        ci.enabledLayerCount   = static_cast<uint32_t>(kValidationLayers.size());
        ci.ppEnabledLayerNames = kValidationLayers.data();
    }

    if (vkCreateInstance(&ci, nullptr, &instance_) != VK_SUCCESS) {
        throw std::runtime_error("vkCreateInstance failed");
    }
}

void VulkanContext::create_debug_messenger() {
    VkDebugUtilsMessengerCreateInfoEXT ci{};
    ci.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
    ci.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT
                       | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
    ci.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT
                   | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT
                   | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
    ci.pfnUserCallback = debug_callback;

    auto fp = reinterpret_cast<PFN_vkCreateDebugUtilsMessengerEXT>(
        vkGetInstanceProcAddr(instance_, "vkCreateDebugUtilsMessengerEXT"));
    if (!fp || fp(instance_, &ci, nullptr, &debug_messenger_) != VK_SUCCESS) {
        std::fprintf(stderr, "warning: debug messenger setup failed\n");
        debug_messenger_ = VK_NULL_HANDLE;
    }
}

void VulkanContext::pick_physical_device() {
    uint32_t count = 0;
    vkEnumeratePhysicalDevices(instance_, &count, nullptr);
    if (count == 0) throw std::runtime_error("no Vulkan-capable GPUs found");
    std::vector<VkPhysicalDevice> devices(count);
    vkEnumeratePhysicalDevices(instance_, &count, devices.data());

    int best = -1;
    for (auto pd : devices) {
        int s = score_device(pd, surface_);
        if (s > best) { best = s; physical_device_ = pd; }
    }
    if (physical_device_ == VK_NULL_HANDLE || best < 0) {
        throw std::runtime_error("no suitable GPU (need Vulkan 1.3 + dynamic rendering + sync2)");
    }
    queue_families_ = find_queue_families(physical_device_, surface_);
}

void VulkanContext::create_logical_device() {
    std::set<uint32_t> unique_families = { queue_families_.graphics, queue_families_.present };
    std::vector<VkDeviceQueueCreateInfo> qcis;
    float priority = 1.0f;
    for (uint32_t fam : unique_families) {
        VkDeviceQueueCreateInfo q{};
        q.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
        q.queueFamilyIndex = fam;
        q.queueCount = 1;
        q.pQueuePriorities = &priority;
        qcis.push_back(q);
    }

    VkPhysicalDeviceVulkan13Features v13{};
    v13.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES;
    v13.dynamicRendering = VK_TRUE;
    v13.synchronization2 = VK_TRUE;

    VkPhysicalDeviceFeatures2 f2{};
    f2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
    f2.pNext = &v13;
    f2.features.samplerAnisotropy = VK_TRUE;
    f2.features.fillModeNonSolid  = VK_TRUE;

    VkDeviceCreateInfo ci{};
    ci.sType                   = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    ci.pNext                   = &f2;
    ci.queueCreateInfoCount    = static_cast<uint32_t>(qcis.size());
    ci.pQueueCreateInfos       = qcis.data();
    ci.enabledExtensionCount   = static_cast<uint32_t>(kDeviceExtensions.size());
    ci.ppEnabledExtensionNames = kDeviceExtensions.data();

    if (vkCreateDevice(physical_device_, &ci, nullptr, &device_) != VK_SUCCESS) {
        throw std::runtime_error("vkCreateDevice failed");
    }
    vkGetDeviceQueue(device_, queue_families_.graphics, 0, &graphics_queue_);
    vkGetDeviceQueue(device_, queue_families_.present,  0, &present_queue_);
}

void VulkanContext::create_swapchain(core::Window& window) {
    VkSurfaceCapabilitiesKHR caps{};
    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physical_device_, surface_, &caps);

    uint32_t fcount = 0;
    vkGetPhysicalDeviceSurfaceFormatsKHR(physical_device_, surface_, &fcount, nullptr);
    std::vector<VkSurfaceFormatKHR> formats(fcount);
    vkGetPhysicalDeviceSurfaceFormatsKHR(physical_device_, surface_, &fcount, formats.data());

    uint32_t mcount = 0;
    vkGetPhysicalDeviceSurfacePresentModesKHR(physical_device_, surface_, &mcount, nullptr);
    std::vector<VkPresentModeKHR> modes(mcount);
    vkGetPhysicalDeviceSurfacePresentModesKHR(physical_device_, surface_, &mcount, modes.data());

    VkSurfaceFormatKHR fmt = pick_surface_format(formats);
    VkPresentModeKHR   pm  = pick_present_mode(modes);

    VkExtent2D extent = caps.currentExtent;
    if (extent.width == UINT32_MAX) {
        int w = 0, h = 0;
        window.framebuffer_size(w, h);
        extent.width  = std::clamp(static_cast<uint32_t>(w), caps.minImageExtent.width,  caps.maxImageExtent.width);
        extent.height = std::clamp(static_cast<uint32_t>(h), caps.minImageExtent.height, caps.maxImageExtent.height);
    }

    uint32_t image_count = caps.minImageCount + 1;
    if (caps.maxImageCount > 0 && image_count > caps.maxImageCount) image_count = caps.maxImageCount;

    VkSwapchainCreateInfoKHR ci{};
    ci.sType            = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
    ci.surface          = surface_;
    ci.minImageCount    = image_count;
    ci.imageFormat      = fmt.format;
    ci.imageColorSpace  = fmt.colorSpace;
    ci.imageExtent      = extent;
    ci.imageArrayLayers = 1;
    ci.imageUsage       = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
    ci.preTransform     = caps.currentTransform;
    ci.compositeAlpha   = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    ci.presentMode      = pm;
    ci.clipped          = VK_TRUE;

    uint32_t family_indices[] = { queue_families_.graphics, queue_families_.present };
    if (queue_families_.graphics != queue_families_.present) {
        ci.imageSharingMode      = VK_SHARING_MODE_CONCURRENT;
        ci.queueFamilyIndexCount = 2;
        ci.pQueueFamilyIndices   = family_indices;
    } else {
        ci.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    }

    if (vkCreateSwapchainKHR(device_, &ci, nullptr, &swapchain_) != VK_SUCCESS) {
        throw std::runtime_error("vkCreateSwapchainKHR failed");
    }

    swapchain_format_ = fmt.format;
    swapchain_extent_ = extent;

    uint32_t sc_count = 0;
    vkGetSwapchainImagesKHR(device_, swapchain_, &sc_count, nullptr);
    swapchain_images_.resize(sc_count);
    vkGetSwapchainImagesKHR(device_, swapchain_, &sc_count, swapchain_images_.data());

    swapchain_image_views_.resize(sc_count);
    for (uint32_t i = 0; i < sc_count; ++i) {
        VkImageViewCreateInfo iv{};
        iv.sType    = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        iv.image    = swapchain_images_[i];
        iv.viewType = VK_IMAGE_VIEW_TYPE_2D;
        iv.format   = swapchain_format_;
        iv.subresourceRange.aspectMask     = VK_IMAGE_ASPECT_COLOR_BIT;
        iv.subresourceRange.baseMipLevel   = 0;
        iv.subresourceRange.levelCount     = 1;
        iv.subresourceRange.baseArrayLayer = 0;
        iv.subresourceRange.layerCount     = 1;
        if (vkCreateImageView(device_, &iv, nullptr, &swapchain_image_views_[i]) != VK_SUCCESS) {
            throw std::runtime_error("vkCreateImageView (swapchain) failed");
        }
    }
}

void VulkanContext::destroy_swapchain() {
    for (auto v : swapchain_image_views_) vkDestroyImageView(device_, v, nullptr);
    swapchain_image_views_.clear();
    swapchain_images_.clear();
    if (swapchain_) { vkDestroySwapchainKHR(device_, swapchain_, nullptr); swapchain_ = VK_NULL_HANDLE; }
}

void VulkanContext::recreate_swapchain(core::Window& window) {
    window.wait_while_minimized();
    vkDeviceWaitIdle(device_);
    destroy_swapchain();
    create_swapchain(window);
}

} // namespace finalLab::render
