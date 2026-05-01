#pragma once

#include "scene/scene_types.hpp"

#include <filesystem>

namespace finalLab::scene {

Scene load_scene_from_file(const std::filesystem::path& path);

} // namespace finalLab::scene
