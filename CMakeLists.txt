cmake_minimum_required(VERSION 3.25)

project(finalLab LANGUAGES CXX)

if(NOT WIN32)
    message(FATAL_ERROR "finalLab targets Windows only (Winsock 2, demo on RBB-335).")
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Vulkan REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(flatbuffers CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/FlatBuffers.cmake)
finallab_generate_flatbuffers(
    SCHEMA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/schema"
    OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated"
    OUTPUT_VAR FBS_GENERATED_HEADERS
)

find_program(GLSLANG_VALIDATOR
    NAMES glslangValidator glslang
    HINTS "$ENV{VULKAN_SDK}/Bin" "$ENV{VULKAN_SDK}/bin"
    REQUIRED)

set(SHADER_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/shaders)
set(SHADER_OUT_DIR $<TARGET_FILE_DIR:finalLab>/shaders)

file(GLOB SHADER_FILES CONFIGURE_DEPENDS
    "${SHADER_SRC_DIR}/*.vert"
    "${SHADER_SRC_DIR}/*.frag"
    "${SHADER_SRC_DIR}/*.comp")

set(COMPILED_SHADERS "")
foreach(SHADER ${SHADER_FILES})
    get_filename_component(SHADER_NAME ${SHADER} NAME)
    set(SPV "${CMAKE_CURRENT_BINARY_DIR}/shaders/${SHADER_NAME}.spv")
    add_custom_command(
        OUTPUT ${SPV}
        COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/shaders"
        COMMAND ${GLSLANG_VALIDATOR} -V -o ${SPV} ${SHADER}
        DEPENDS ${SHADER}
        COMMENT "glslang ${SHADER_NAME}"
        VERBATIM)
    list(APPEND COMPILED_SHADERS ${SPV})
endforeach()

add_custom_target(finalLab_shaders ALL DEPENDS ${COMPILED_SHADERS})

set(SCENE_JSON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/assets/scenes)
set(SCENE_BIN_DIR  ${CMAKE_CURRENT_BINARY_DIR}/assets/scenes)
file(GLOB SCENE_JSON_FILES CONFIGURE_DEPENDS "${SCENE_JSON_DIR}/*.json")

set(COMPILED_SCENES "")
foreach(SCENE_JSON ${SCENE_JSON_FILES})
    get_filename_component(SCENE_NAME_WE ${SCENE_JSON} NAME_WE)
    set(SCENE_BIN "${SCENE_BIN_DIR}/${SCENE_NAME_WE}.scene")
    add_custom_command(
        OUTPUT ${SCENE_BIN}
        COMMAND ${CMAKE_COMMAND} -E make_directory "${SCENE_BIN_DIR}"
        COMMAND $<TARGET_FILE:flatbuffers::flatc> --binary
                -o "${SCENE_BIN_DIR}"
                "${CMAKE_CURRENT_SOURCE_DIR}/schema/scene.fbs"
                "${SCENE_JSON}"
        DEPENDS ${SCENE_JSON} "${CMAKE_CURRENT_SOURCE_DIR}/schema/scene.fbs"
        COMMENT "flatc --binary ${SCENE_NAME_WE}.json"
        VERBATIM)
    list(APPEND COMPILED_SCENES ${SCENE_BIN})
endforeach()

add_custom_target(finalLab_scenes ALL DEPENDS ${COMPILED_SCENES})

set(FINALLAB_SOURCES
    src/main.cpp
    src/core/window.cpp
    src/core/fly_camera.cpp
    src/render/vulkan_context.cpp
    src/render/renderer.cpp
    src/render/imgui_layer.cpp
    src/render/buffer.cpp
    src/render/mesh.cpp
    src/render/instance_renderer.cpp
    src/scene/scene_loader.cpp
    src/scene/animation.cpp
    src/physics/physics_world.cpp
    src/physics/collision.cpp
    src/physics/spawner.cpp
    src/physics/flocking.cpp
    src/physics/spatial_index.cpp
    src/sim/sim_runtime.cpp
    src/net/peer_config.cpp
    src/net/network_runtime.cpp
)

add_executable(finalLab
    ${FINALLAB_SOURCES}
    ${FBS_GENERATED_HEADERS}
)

add_dependencies(finalLab finalLab_shaders finalLab_scenes)

add_custom_command(TARGET finalLab POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${SHADER_OUT_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_CURRENT_BINARY_DIR}/shaders" ${SHADER_OUT_DIR}
    COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:finalLab>/assets/scenes"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${SCENE_BIN_DIR}" "$<TARGET_FILE_DIR:finalLab>/assets/scenes"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/peers.txt" "$<TARGET_FILE_DIR:finalLab>/peers.txt"
    COMMENT "Copying shaders + scenes + peers.txt next to finalLab.exe")

target_include_directories(finalLab PRIVATE
    src
    ${CMAKE_CURRENT_BINARY_DIR}/generated
)

target_compile_definitions(finalLab PRIVATE
    WIN32_LEAN_AND_MEAN
    NOMINMAX
    GLFW_INCLUDE_VULKAN
)

target_link_libraries(finalLab PRIVATE
    Vulkan::Vulkan
    glfw
    glm::glm
    flatbuffers::flatbuffers
    imgui::imgui
    ws2_32
)

if(MSVC)
    target_compile_options(finalLab PRIVATE /W4 /permissive- /Zc:__cplusplus)
endif()
