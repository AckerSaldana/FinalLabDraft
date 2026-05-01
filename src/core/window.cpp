#include "core/window.hpp"

#include <stdexcept>

namespace finalLab::core {

Window::Window(int width, int height, const std::string& title) {
    if (!glfwInit()) {
        throw std::runtime_error("glfwInit failed");
    }
    if (!glfwVulkanSupported()) {
        glfwTerminate();
        throw std::runtime_error("Vulkan loader not present");
    }

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    window_ = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
    if (!window_) {
        glfwTerminate();
        throw std::runtime_error("glfwCreateWindow failed");
    }

    glfwSetWindowUserPointer(window_, this);
    glfwSetFramebufferSizeCallback(window_, framebuffer_resize_cb);
}

Window::~Window() {
    if (window_) glfwDestroyWindow(window_);
    glfwTerminate();
}

bool Window::should_close() const { return glfwWindowShouldClose(window_); }
void Window::poll_events() const { glfwPollEvents(); }

VkSurfaceKHR Window::create_surface(VkInstance instance) const {
    VkSurfaceKHR surface = VK_NULL_HANDLE;
    if (glfwCreateWindowSurface(instance, window_, nullptr, &surface) != VK_SUCCESS) {
        throw std::runtime_error("glfwCreateWindowSurface failed");
    }
    return surface;
}

void Window::framebuffer_size(int& width, int& height) const {
    glfwGetFramebufferSize(window_, &width, &height);
}

void Window::wait_while_minimized() const {
    int w = 0, h = 0;
    glfwGetFramebufferSize(window_, &w, &h);
    while (w == 0 || h == 0) {
        glfwWaitEvents();
        glfwGetFramebufferSize(window_, &w, &h);
    }
}

void Window::framebuffer_resize_cb(GLFWwindow* w, int width, int height) {
    auto* self = static_cast<Window*>(glfwGetWindowUserPointer(w));
    if (self && self->on_resize_) self->on_resize_(width, height);
}

} // namespace finalLab::core
