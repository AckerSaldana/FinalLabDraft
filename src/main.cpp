#include "core/fly_camera.hpp"
#include "core/window.hpp"
#include "net/network_runtime.hpp"
#include "net/peer_config.hpp"
#include "physics/flocking.hpp"
#include "render/imgui_layer.hpp"
#include "render/instance_renderer.hpp"
#include "render/renderer.hpp"
#include "render/vulkan_context.hpp"
#include "scene/scene_loader.hpp"
#include "scene/scene_types.hpp"
#include "sim/sim_runtime.hpp"

#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <imgui.h>

#include <windows.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr int kWidth  = 1280;
constexpr int kHeight = 720;

#ifdef NDEBUG
constexpr bool kEnableValidation = false;
#else
constexpr bool kEnableValidation = true;
#endif

enum class ColorMode { ByOwner, ByMaterial, ByShape };
constexpr const char* kColorModeNames[] = { "By owner", "By material", "By shape" };

std::filesystem::path exe_dir() {
    char buf[MAX_PATH];
    DWORD n = GetModuleFileNameA(nullptr, buf, MAX_PATH);
    return std::filesystem::path(std::string(buf, n)).parent_path();
}

std::vector<std::filesystem::path> scan_scene_files() {
    auto root = exe_dir() / "assets" / "scenes";
    std::vector<std::filesystem::path> out;
    std::error_code ec;
    if (!std::filesystem::exists(root, ec)) return out;
    for (const auto& e : std::filesystem::directory_iterator(root, ec)) {
        if (e.is_regular_file() && e.path().extension() == ".scene") out.push_back(e.path());
    }
    std::sort(out.begin(), out.end());
    return out;
}

glm::vec4 owner_color(finalLab::scene::OwnerId o) {
    switch (o) {
        case finalLab::scene::OwnerId::One:   return { 0.90f, 0.25f, 0.25f, 1.0f };
        case finalLab::scene::OwnerId::Two:   return { 0.25f, 0.85f, 0.30f, 1.0f };
        case finalLab::scene::OwnerId::Three: return { 0.30f, 0.45f, 0.95f, 1.0f };
        case finalLab::scene::OwnerId::Four:  return { 0.95f, 0.85f, 0.20f, 1.0f };
    }
    return { 0.75f, 0.75f, 0.75f, 1.0f };
}

glm::vec4 material_color(const std::string& name, const finalLab::scene::Scene& s) {
    static constexpr std::array<glm::vec4, 8> palette = {{
        { 0.85f, 0.50f, 0.30f, 1.0f }, { 0.30f, 0.70f, 0.85f, 1.0f },
        { 0.55f, 0.85f, 0.40f, 1.0f }, { 0.85f, 0.40f, 0.70f, 1.0f },
        { 0.65f, 0.60f, 0.90f, 1.0f }, { 0.95f, 0.85f, 0.45f, 1.0f },
        { 0.45f, 0.85f, 0.70f, 1.0f }, { 0.85f, 0.70f, 0.50f, 1.0f },
    }};
    for (size_t i = 0; i < s.materials.size(); ++i) {
        if (s.materials[i].name == name) return palette[i % palette.size()];
    }
    return { 0.7f, 0.7f, 0.7f, 1.0f };
}

glm::vec4 shape_color(finalLab::scene::ShapeKind k) {
    using SK = finalLab::scene::ShapeKind;
    switch (k) {
        case SK::Sphere:   return { 0.95f, 0.40f, 0.40f, 1.0f };
        case SK::Cuboid:   return { 0.40f, 0.95f, 0.40f, 1.0f };
        case SK::Cylinder: return { 0.40f, 0.55f, 0.95f, 1.0f };
        case SK::Capsule:  return { 0.95f, 0.85f, 0.40f, 1.0f };
        case SK::Plane:    return { 0.55f, 0.55f, 0.60f, 1.0f };
    }
    return { 0.75f, 0.75f, 0.75f, 1.0f };
}

glm::vec4 object_color(const finalLab::scene::Object& o, ColorMode mode,
                       const finalLab::scene::Scene& s) {
    using namespace finalLab::scene;
    if (mode == ColorMode::ByMaterial) return material_color(o.material, s);
    if (mode == ColorMode::ByShape)    return shape_color(o.shape.kind);
    switch (o.behaviour.kind) {
        case BehaviourKind::Simulated: return owner_color(o.behaviour.sim.owner);
        case BehaviourKind::Boid:      return owner_color(o.behaviour.boid.owner);
        case BehaviourKind::Animated:  return { 0.70f, 0.45f, 0.85f, 1.0f };
        case BehaviourKind::Static:    return { 0.45f, 0.45f, 0.48f, 1.0f };
    }
    return { 1.0f, 1.0f, 1.0f, 1.0f };
}

finalLab::render::ShapeKind to_render_shape(finalLab::scene::ShapeKind s) {
    using SK = finalLab::scene::ShapeKind;
    using RK = finalLab::render::ShapeKind;
    switch (s) {
        case SK::Sphere:   return RK::Sphere;
        case SK::Cuboid:   return RK::Cuboid;
        case SK::Cylinder: return RK::Cylinder;
        case SK::Capsule:  return RK::Capsule;
        case SK::Plane:    return RK::Plane;
    }
    return RK::Sphere;
}

finalLab::render::RenderInstance
make_render_instance(const finalLab::scene::Object& o, ColorMode mode, const finalLab::scene::Scene& s) {
    finalLab::render::RenderInstance ri;
    ri.shape     = to_render_shape(o.shape.kind);
    ri.container = (o.collision_type == finalLab::scene::CollisionType::Container);
    ri.model     = glm::mat4(1.0f);
    ri.color     = object_color(o, mode, s);
    return ri;
}

std::vector<finalLab::render::RenderInstance>
scene_to_render_instances(const finalLab::scene::Scene& s, ColorMode mode) {
    std::vector<finalLab::render::RenderInstance> out;
    out.reserve(s.objects.size());
    for (const auto& o : s.objects) out.push_back(make_render_instance(o, mode, s));
    return out;
}

glm::mat4 make_projection(const finalLab::scene::Camera& c, VkExtent2D extent) {
    float aspect = static_cast<float>(extent.width) / std::max(1u, extent.height);
    glm::mat4 proj;
    if (c.kind == finalLab::scene::CameraKind::Orthographic) {
        float half_h = c.orthographic.size * 0.5f;
        float half_w = half_h * aspect;
        proj = glm::ortho(-half_w, half_w, -half_h, half_h, c.orthographic.z_near, c.orthographic.z_far);
    } else {
        proj = glm::perspective(c.perspective.fov_rad, aspect, c.perspective.z_near, c.perspective.z_far);
    }
    proj[1][1] *= -1.0f;
    return proj;
}

finalLab::scene::Camera make_fallback_camera() {
    finalLab::scene::Camera c;
    c.name = "fallback";
    c.transform.position    = glm::vec3(0.0f, 10.0f, 16.0f);
    c.transform.orientation = glm::angleAxis(glm::radians(-25.0f), glm::vec3(1, 0, 0));
    c.kind = finalLab::scene::CameraKind::Perspective;
    c.perspective.fov_rad = glm::radians(60.0f);
    return c;
}

} // namespace

int main(int argc, char** argv) try {
    using namespace finalLab;

    SetThreadAffinityMask(GetCurrentThread(), sim::cores_to_mask(1, 1));

    int  arg_peer = 0;
    std::filesystem::path arg_peers_file;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--peer" && i + 1 < argc)        arg_peer = std::atoi(argv[++i]);
        else if (a == "--peers" && i + 1 < argc)  arg_peers_file = argv[++i];
    }

    net::PeerConfig peer_config;
    if (arg_peer > 0) {
        std::filesystem::path pf = arg_peers_file.empty() ? (exe_dir() / "peers.txt") : arg_peers_file;
        peer_config = net::load_peer_config(pf, arg_peer);
        std::printf("networked: peer %d / %d (peers from %s)\n",
                    peer_config.my_peer_id, peer_config.total_peers(), pf.string().c_str());
    } else {
        peer_config = net::single_peer_config();
        std::printf("single-peer mode (pass --peer N to enable networking)\n");
    }

    auto scene_files = scan_scene_files();
    std::vector<std::string> scene_names;
    int active_scene = 0;
    for (size_t i = 0; i < scene_files.size(); ++i) {
        scene_names.push_back(scene_files[i].stem().string());
        if (scene_files[i].stem() == "basic") active_scene = static_cast<int>(i);
    }

    auto load_scene_at = [&](int idx) -> scene::Scene {
        scene::Scene s;
        if (idx >= 0 && idx < static_cast<int>(scene_files.size())) {
            try { s = scene::load_scene_from_file(scene_files[idx]); }
            catch (const std::exception& e) { std::fprintf(stderr, "scene load failed: %s\n", e.what()); }
        }
        if (s.cameras.empty()) s.cameras.push_back(make_fallback_camera());
        return s;
    };

    scene::Scene scene = load_scene_at(active_scene);

    core::Window window(kWidth, kHeight, "finalLab - Networked Physics Simulator");
    render::VulkanContext    ctx(window, kEnableValidation);
    render::Renderer         renderer(ctx, window);
    render::InstanceRenderer instancer(ctx, renderer.color_format(), renderer.depth_format());
    render::ImGuiLayer       imgui(ctx, window.handle(), renderer.color_format());
    window.set_resize_callback([&](int, int) { renderer.notify_resized(); });

    net::NetworkRuntime net_runtime;
    if (peer_config.networked()) {
        net_runtime.start(peer_config, sim::cores_to_mask(2, 3));
    }

    sim::SimRuntime sim_runtime;
    sim_runtime.set_my_peer_id(peer_config.my_peer_id);
    if (peer_config.networked()) sim_runtime.attach_network(&net_runtime);
    sim_runtime.load_scene(scene);
    sim_runtime.start(sim::cores_to_mask(4, 8));

    ColorMode color_mode = ColorMode::ByOwner;
    auto      instances  = scene_to_render_instances(scene, color_mode);
    int       my_scene_version = 0;

    core::FlyCamera fly;
    int  camera_index = 0;
    int  last_seeded  = -1;
    int  last_scene   = active_scene;
    int  last_color   = static_cast<int>(color_mode);
    int  inspector_idx = 0;

    int  sim_hz_ui    = 240;
    int  render_hz_ui = 60;
    bool ui_paused    = false;
    bool ui_gravity_on = true;
    glm::vec3 ui_gravity = { 0.0f, -9.81f, 0.0f };
    int  ui_solver_iters = 6;
    float ui_position_correction = 0.5f;
    float ui_penetration_slop    = 0.001f;
    physics::FlockParams ui_flock{};
    bool  ui_net_smooth_enabled = true;
    float ui_net_correction_rate = 8.0f;
    int   ui_sim_latency_ms = 0;
    int   ui_sim_jitter_ms  = 0;
    int   ui_sim_loss_pct   = 0;

    sim_runtime.set_sim_hz(sim_hz_ui);
    sim_runtime.set_flock_params(ui_flock);
    sim_runtime.set_net_smoothing(ui_net_smooth_enabled, ui_net_correction_rate);

    sim::SnapshotData snap;
    auto last_render = std::chrono::steady_clock::now();
    auto last_tick   = std::chrono::steady_clock::now();

    std::vector<std::string> body_label_storage;

    while (!window.should_close()) {
        window.poll_events();

        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - last_tick).count();
        last_tick = now;

        if (active_scene != last_scene) {
            scene = load_scene_at(active_scene);
            sim_runtime.load_scene(scene);
            instances = scene_to_render_instances(scene, color_mode);
            camera_index  = 0;
            last_seeded   = -1;
            last_scene    = active_scene;
            last_color    = static_cast<int>(color_mode);
            inspector_idx = 0;
        }
        if (static_cast<int>(color_mode) != last_color) {
            instances = scene_to_render_instances(scene, color_mode);
            last_color = static_cast<int>(color_mode);
        }

        sim_runtime.drain_snapshot(snap);

        if (snap.scene_version != my_scene_version && snap.scene_version > 0) {
            scene = load_scene_at(active_scene);
            instances = scene_to_render_instances(scene, color_mode);
            my_scene_version = snap.scene_version;
        }

        for (auto& obj : snap.spawned_since_last) {
            instances.push_back(make_render_instance(obj, color_mode, scene));
            scene.objects.push_back(std::move(obj));
        }

        for (const auto& bs : snap.bodies) {
            int oi = bs.scene_object_index;
            if (oi >= 0 && oi < static_cast<int>(instances.size())) {
                instances[oi].model = bs.model;
            }
        }

        if (camera_index != last_seeded) {
            const auto& c = scene.cameras[camera_index];
            fly.seed(c.transform.position, c.transform.orientation);
            last_seeded = camera_index;
        }
        fly.update(window.handle(), dt);

        imgui.begin_frame();
        ImGui::Begin("finalLab");
        ImGui::Text("finalLab — networked rigid-body sim + flocking");
        ImGui::Text("Peer %d of %d   |   net thread: cores 2-3   |   sim thread: cores 4+",
                    peer_config.my_peer_id, peer_config.total_peers());
        ImGui::Text("Render: %.1f FPS  |  Sim target: %d Hz  |  Sim actual: %d Hz",
                    ImGui::GetIO().Framerate, sim_hz_ui, snap.sim_hz_actual);

        if (ImGui::CollapsingHeader("Global", ImGuiTreeNodeFlags_DefaultOpen)) {
            std::vector<const char*> snames;
            for (const auto& n : scene_names) snames.push_back(n.c_str());
            if (!snames.empty()) {
                ImGui::Combo("Scene", &active_scene, snames.data(), static_cast<int>(snames.size()));
            }
            ImGui::Text("Loaded: %s   Objects: %zu",
                        scene.name.c_str(), scene.objects.size());
            ImGui::TextDisabled("(global control — Phase 6 broadcasts swap to peers)");
        }

        if (ImGui::CollapsingHeader("Local", ImGuiTreeNodeFlags_DefaultOpen)) {
            int cm = static_cast<int>(color_mode);
            if (ImGui::Combo("Colour mode", &cm, kColorModeNames, IM_ARRAYSIZE(kColorModeNames))) {
                color_mode = static_cast<ColorMode>(cm);
            }
            std::vector<const char*> cam_names;
            for (const auto& c : scene.cameras) cam_names.push_back(c.name.c_str());
            if (ImGui::Combo("Camera", &camera_index, cam_names.data(), static_cast<int>(cam_names.size()))) {
                last_seeded = -1;
            }
            if (ImGui::Button("Reset to authored pose")) last_seeded = -1;
            ImGui::SliderFloat("Move speed", &fly.move_speed, 0.5f, 40.0f, "%.1f m/s");
            ImGui::SliderFloat("Mouse sens", &fly.mouse_sensitivity, 0.0005f, 0.01f, "%.4f");
            ImGui::TextDisabled("hold RMB: look + WASD/QE, shift = fast");

            glm::vec3 p = fly.position();
            ImGui::Text("pos: (%.1f, %.1f, %.1f)  yaw: %.1f  pitch: %.1f",
                        p.x, p.y, p.z,
                        glm::degrees(fly.yaw_rad()), glm::degrees(fly.pitch_rad()));
        }

        if (ImGui::CollapsingHeader("Threading + Hz", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::SliderInt("Sim Hz", &sim_hz_ui, 30, 2000)) sim_runtime.set_sim_hz(sim_hz_ui);
            ImGui::SliderInt("Render Hz", &render_hz_ui, 15, 240);
            ImGui::Text("Sim steps last frame: %d", snap.steps_since_last_drain);
            ImGui::TextDisabled("vis core 1 | net cores 2-3 | sim cores 4-8");
        }

        if (ImGui::CollapsingHeader("Network", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (peer_config.networked()) {
                ImGui::Text("Peer %d of %d (this PC)", peer_config.my_peer_id, peer_config.total_peers());
                for (const auto& p : peer_config.peers) {
                    if (p.peer_id == peer_config.my_peer_id) continue;
                    ImGui::BulletText("peer %d -> %s:%u", p.peer_id, p.address.c_str(), unsigned(p.port));
                }
                ImGui::Text("Tx: %d pkts, %d KB    Rx: %d pkts, %d KB",
                            net_runtime.packets_sent(), net_runtime.bytes_sent() / 1024,
                            net_runtime.packets_received(), net_runtime.bytes_received() / 1024);
                ImGui::Text("Dropped: %d (sim-loss)   %d (stale/out-of-order)",
                            net_runtime.packets_dropped_in_sim(), net_runtime.packets_dropped_stale());

                bool smooth_dirty = false;
                smooth_dirty |= ImGui::Checkbox("Smoothing on", &ui_net_smooth_enabled);
                smooth_dirty |= ImGui::SliderFloat("Correction rate", &ui_net_correction_rate, 0.5f, 30.0f, "%.1f");
                if (smooth_dirty) sim_runtime.set_net_smoothing(ui_net_smooth_enabled, ui_net_correction_rate);

                ImGui::Separator();
                ImGui::Text("In-app QoS shaper (worst-case spec target: 100ms +/- 50ms, 20%% loss)");
                if (ImGui::SliderInt("Sim latency (ms)", &ui_sim_latency_ms, 0, 300))
                    net_runtime.set_sim_latency_ms(ui_sim_latency_ms);
                if (ImGui::SliderInt("Sim jitter (ms)", &ui_sim_jitter_ms, 0, 100))
                    net_runtime.set_sim_jitter_ms(ui_sim_jitter_ms);
                if (ImGui::SliderInt("Sim loss (%)", &ui_sim_loss_pct, 0, 50))
                    net_runtime.set_sim_loss_pct(ui_sim_loss_pct);
            } else {
                ImGui::TextDisabled("(single-peer — re-launch with --peer N to enable networking)");
            }
        }

        if (ImGui::CollapsingHeader("Physics", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Text("Sim time: %.2f s", snap.total_time);
            ImGui::Text("Active bodies: %zu   Contacts/frame: %zu",
                        snap.body_count, snap.contact_count);
            if (ImGui::Checkbox("Gravity on", &ui_gravity_on)) sim_runtime.set_gravity_on(ui_gravity_on);
            if (ImGui::DragFloat3("Gravity (m/s^2)", &ui_gravity.x, 0.1f, -50.0f, 50.0f, "%.2f"))
                sim_runtime.set_gravity(ui_gravity);
            if (ImGui::Checkbox("Paused", &ui_paused)) sim_runtime.set_paused(ui_paused);
            ImGui::SameLine();
            if (ImGui::Button("Step"))  sim_runtime.single_step();
            ImGui::SameLine();
            if (ImGui::Button("Reset")) {
                sim_runtime.reset();
                inspector_idx = 0;
            }
            if (ImGui::SliderInt("Solver iterations", &ui_solver_iters, 1, 20))
                sim_runtime.set_solver_iterations(ui_solver_iters);
            if (ImGui::SliderFloat("Position correction %", &ui_position_correction, 0.0f, 1.0f, "%.2f"))
                sim_runtime.set_position_correction(ui_position_correction);
        }

        if (ImGui::CollapsingHeader("Flocking", ImGuiTreeNodeFlags_DefaultOpen)) {
            bool dirty = false;
            dirty |= ImGui::Checkbox("Enabled", &ui_flock.enabled);
            const char* combo_modes[] = { "Truncated sum", "Prioritised dither" };
            int cm = static_cast<int>(ui_flock.combine_mode);
            if (ImGui::Combo("Combine", &cm, combo_modes, IM_ARRAYSIZE(combo_modes))) {
                ui_flock.combine_mode = static_cast<physics::CombineMode>(cm);
                dirty = true;
            }
            dirty |= ImGui::SliderFloat("Max speed",      &ui_flock.max_speed,         0.5f, 30.0f, "%.2f");
            dirty |= ImGui::SliderFloat("Max force",      &ui_flock.max_force,         0.5f, 50.0f, "%.2f");
            dirty |= ImGui::SliderFloat("Neighbour r",    &ui_flock.neighbour_radius,  0.5f, 20.0f, "%.2f");
            dirty |= ImGui::SliderFloat("Avoidance r",    &ui_flock.avoidance_radius,  0.5f, 20.0f, "%.2f");
            dirty |= ImGui::SliderFloat("w cohesion",     &ui_flock.cohesion_weight,   0.0f,  5.0f, "%.2f");
            dirty |= ImGui::SliderFloat("w alignment",    &ui_flock.alignment_weight,  0.0f,  5.0f, "%.2f");
            dirty |= ImGui::SliderFloat("w separation",   &ui_flock.separation_weight, 0.0f,  5.0f, "%.2f");
            dirty |= ImGui::SliderFloat("w avoidance",    &ui_flock.avoidance_weight,  0.0f,  5.0f, "%.2f");
            if (dirty) sim_runtime.set_flock_params(ui_flock);
            ImGui::TextDisabled("Reynolds steering: cohesion + alignment + separation + obstacle avoidance");

            ImGui::Separator();
            ImGui::Text("Spatial segmentation (live benchmark)");
            const char* sm_names[] = { "None (O(n^2))", "Uniform Grid", "Octree" };
            int sm_idx = static_cast<int>(snap.spatial_mode);
            if (ImGui::Combo("Mode", &sm_idx, sm_names, IM_ARRAYSIZE(sm_names))) {
                sim_runtime.set_spatial_mode(static_cast<physics::SpatialMode>(sm_idx));
            }
            ImGui::Text("Build:    %lld us / step", static_cast<long long>(snap.spatial_stats.build_us));
            ImGui::Text("Queries:  %lld us total over %d calls",
                        static_cast<long long>(snap.spatial_stats.query_us_total), snap.spatial_stats.query_count);
            ImGui::Text("Memory:   %.2f KB",
                        static_cast<double>(snap.spatial_stats.memory_bytes) / 1024.0);
            ImGui::TextDisabled("Switch modes live; build/query times scale with boid count");
        }

        if (ImGui::CollapsingHeader("Inspector")) {
            if (snap.bodies.empty()) {
                ImGui::TextDisabled("(no bodies)");
            } else {
                body_label_storage.clear();
                body_label_storage.reserve(snap.bodies.size());
                for (size_t i = 0; i < snap.bodies.size(); ++i) {
                    int oi = snap.bodies[i].scene_object_index;
                    body_label_storage.push_back("[" + std::to_string(i) + "] " +
                        ((oi >= 0 && oi < (int)scene.objects.size()) ? scene.objects[oi].name : "?"));
                }
                std::vector<const char*> body_names;
                for (auto& s : body_label_storage) body_names.push_back(s.c_str());

                inspector_idx = std::clamp(inspector_idx, 0, static_cast<int>(snap.bodies.size()) - 1);
                ImGui::Combo("Body", &inspector_idx, body_names.data(), static_cast<int>(body_names.size()));

                const auto& b = snap.bodies[inspector_idx];
                int oi = b.scene_object_index;
                ImGui::Text("Mass: %.4g    1/m: %.4g", b.inverse_mass > 0 ? 1.0f / b.inverse_mass : 0.0f, b.inverse_mass);
                ImGui::Text("Position: (%.2f, %.2f, %.2f)", b.position.x, b.position.y, b.position.z);
                ImGui::Text("Quat: (w=%.3f, x=%.3f, y=%.3f, z=%.3f)",
                            b.orientation.w, b.orientation.x, b.orientation.y, b.orientation.z);
                ImGui::Text("Linear vel:  (%.2f, %.2f, %.2f)",
                            b.linear_velocity.x, b.linear_velocity.y, b.linear_velocity.z);
                glm::vec3 ang_deg = glm::degrees(b.angular_velocity_rad);
                ImGui::Text("Angular vel: (%.1f, %.1f, %.1f) deg/s",
                            ang_deg.x, ang_deg.y, ang_deg.z);
                if (oi >= 0 && oi < (int)scene.objects.size()) {
                    ImGui::Text("Material: %s", scene.objects[oi].material.c_str());
                }
                ImGui::TextDisabled("(read-only in Phase 5; mutation needs command queue)");
            }
        }
        ImGui::End();

        render::FrameContext frame;
        if (renderer.begin_frame(frame)) {
            const scene::Camera& cam = scene.cameras[camera_index];
            glm::mat4 view_proj = make_projection(cam, frame.extent) * fly.view_matrix();
            instancer.draw(frame.frame_index, frame.cmd, view_proj, instances);
            imgui.render(frame.cmd);
            renderer.end_frame(frame);
        }

        if (render_hz_ui > 0) {
            auto target = last_render + std::chrono::microseconds(1'000'000 / render_hz_ui);
            std::this_thread::sleep_until(target);
        }
        last_render = std::chrono::steady_clock::now();
    }

    sim_runtime.stop();
    if (peer_config.networked()) net_runtime.stop();
    return 0;
} catch (const std::exception& e) {
    std::fprintf(stderr, "fatal: %s\n", e.what());
    return 1;
}
