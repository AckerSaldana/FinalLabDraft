#pragma once

#include <GLFW/glfw3.h>
#include <vulkan/vulkan.h>

#include <functional>
#include <string>

namespace finalLab::core {

class Window {
public:
    using ResizeFn = std::function<void(int, int)>;

    Window(int width, int height, const std::string& title);
    ~Window();

    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;

    GLFWwindow* handle() const { return window_; }
    bool should_close() const;
    void poll_events() const;

    VkSurfaceKHR create_surface(VkInstance instance) const;
    void framebuffer_size(int& width, int& height) const;
    void wait_while_minimized() const;

    void set_resize_callback(ResizeFn cb) { on_resize_ = std::move(cb); }

private:
    static void framebuffer_resize_cb(GLFWwindow* w, int width, int height);

    GLFWwindow* window_ = nullptr;
    ResizeFn on_resize_;
};

} // namespace finalLab::core
