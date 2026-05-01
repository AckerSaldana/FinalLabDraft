#pragma once

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include <cstdint>
#include <string>
#include <variant>
#include <vector>

namespace finalLab::scene {

enum class ShapeKind : uint8_t { Sphere, Cuboid, Cylinder, Capsule, Plane };
enum class CollisionType : uint8_t { Solid, Container };
enum class BehaviourKind : uint8_t { Static, Simulated, Animated, Boid };
enum class OwnerId : uint8_t { One = 0, Two = 1, Three = 2, Four = 3 };
enum class SpawnerOwnerKind : uint8_t { One = 0, Two = 1, Three = 2, Four = 3, Sequential = 4 };
enum class EasingType : uint8_t { Linear, Smoothstep };
enum class PathMode : uint8_t { Stop, Loop, Reverse };
enum class CameraKind : uint8_t { Perspective, Orthographic };
enum class SpawnKind : uint8_t { SingleBurst, Repeating };
enum class SpawnLocationKind : uint8_t { Fixed, RandomBox, RandomSphere };

struct Transform {
    glm::vec3 position{0.0f};
    glm::quat orientation{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 scale{1.0f};
};

struct ShapeSphere   { float radius = 0.5f; };
struct ShapeCuboid   { glm::vec3 size{1.0f}; };
struct ShapeCylinder { float radius = 0.5f; float height = 1.0f; };
struct ShapeCapsule  { float radius = 0.5f; float height = 1.0f; };
struct ShapePlane    { glm::vec3 normal{0.0f, 1.0f, 0.0f}; };

struct Shape {
    ShapeKind     kind = ShapeKind::Sphere;
    ShapeSphere   sphere;
    ShapeCuboid   cuboid;
    ShapeCylinder cylinder;
    ShapeCapsule  capsule;
    ShapePlane    plane;
};

struct StaticObject {};

struct SimulatedObject {
    glm::vec3 initial_linear_velocity{0.0f};
    glm::vec3 initial_angular_velocity_rad{0.0f};
    OwnerId   owner = OwnerId::One;
};

struct Waypoint {
    glm::vec3 position{0.0f};
    glm::quat orientation{1.0f, 0.0f, 0.0f, 0.0f};
    float     time = 0.0f;
};

struct AnimatedObject {
    std::vector<Waypoint> waypoints;
    float      total_duration = 0.0f;
    EasingType easing         = EasingType::Linear;
    PathMode   path_mode      = PathMode::Stop;
};

struct BoidObject {
    glm::vec3 initial_linear_velocity{0.0f};
    glm::vec3 initial_angular_velocity_rad{0.0f};
    OwnerId   owner = OwnerId::One;
};

struct Behaviour {
    BehaviourKind   kind = BehaviourKind::Static;
    StaticObject    stat;
    SimulatedObject sim;
    AnimatedObject  anim;
    BoidObject      boid;
};

struct Object {
    std::string   name;
    Transform     transform;
    std::string   material;
    Shape         shape;
    Behaviour     behaviour;
    CollisionType collision_type = CollisionType::Solid;
    float         mass           = 0.0f;
};

struct PerspectiveCamera {
    float fov_rad = 1.0471975f;
    float z_near  = 0.1f;
    float z_far   = 200.0f;
};

struct OrthographicCamera {
    float size   = 10.0f;
    float z_near = 0.1f;
    float z_far  = 200.0f;
};

struct Camera {
    std::string        name;
    Transform          transform;
    CameraKind         kind = CameraKind::Perspective;
    PerspectiveCamera  perspective;
    OrthographicCamera orthographic;
};

struct Material {
    std::string name;
    float       density = 1000.0f;
};

struct MaterialInteraction {
    std::string material_a;
    std::string material_b;
    float       restitution      = 0.5f;
    float       static_friction  = 0.5f;
    float       dynamic_friction = 0.3f;
};

struct FloatRange { float min = 0.0f; float max = 0.0f; };
struct Vec3Range  { glm::vec3 min{0.0f}; glm::vec3 max{0.0f}; };

struct FixedLocation  { Transform transform; };
struct RandomBoxLoc   { glm::vec3 min{0.0f}; glm::vec3 max{0.0f}; };
struct RandomSphereLoc{ glm::vec3 center{0.0f}; float radius = 1.0f; };

struct SpawnLocation {
    SpawnLocationKind kind = SpawnLocationKind::Fixed;
    FixedLocation     fixed;
    RandomBoxLoc      box;
    RandomSphereLoc   sphere;
};

struct SingleBurstSpawn { uint32_t count = 0; };
struct RepeatingSpawn   { float interval = 1.0f; uint32_t max_count = 0; };

struct SpawnTiming {
    SpawnKind         kind = SpawnKind::SingleBurst;
    SingleBurstSpawn  burst;
    RepeatingSpawn    repeating;
};

struct BaseSpawner {
    std::string       name;
    float             start_time = 0.0f;
    SpawnTiming       timing;
    SpawnLocation     location;
    Vec3Range         linear_velocity;
    Vec3Range         angular_velocity_rad;
    std::string       material;
    SpawnerOwnerKind  owner = SpawnerOwnerKind::One;
};

struct Spawner {
    ShapeKind   shape_kind     = ShapeKind::Sphere;
    bool        produces_boids = false;
    BaseSpawner base;
    FloatRange  radius_range;
    FloatRange  height_range;
    Vec3Range   size_range;
};

struct Scene {
    std::string                       name;
    std::string                       description;
    bool                              gravity_on = true;
    std::vector<Camera>               cameras;
    std::vector<Object>               objects;
    std::vector<Spawner>              spawners;
    std::vector<Material>             materials;
    std::vector<MaterialInteraction>  interactions;
};

float shape_volume(const Shape& shape, const glm::vec3& scale);

} // namespace finalLab::scene
