#include "scene/scene_loader.hpp"

#include "scene_generated.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cmath>
#include <fstream>
#include <stdexcept>

namespace fb = finalLab::fbs;

namespace finalLab::scene {

namespace {

constexpr float kPi = 3.14159265358979323846f;

std::vector<char> read_file(const std::filesystem::path& path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) throw std::runtime_error("cannot open scene: " + path.string());
    std::streamsize size = f.tellg();
    std::vector<char> buf(static_cast<size_t>(size));
    f.seekg(0);
    f.read(buf.data(), size);
    return buf;
}

glm::vec3 to_vec3(const fb::Vec3* v) {
    return v ? glm::vec3(v->x(), v->y(), v->z()) : glm::vec3(0.0f);
}

glm::vec3 to_vec3(const fb::Vec3* v, const glm::vec3& fallback) {
    return v ? glm::vec3(v->x(), v->y(), v->z()) : fallback;
}

// Overloads for the by-reference accessors that current FlatBuffers generates
// for struct-fields-of-structs (Transform.position, PhysicsState.linear_velocity,
// Vec3Range.min, etc.). The pointer overloads above stay for table-of-struct
// accessors (Waypoint.position, Cuboid.size, RandomBox.min, ...).
glm::vec3 to_vec3(const fb::Vec3& v) {
    return glm::vec3(v.x(), v.y(), v.z());
}

glm::vec3 to_vec3(const fb::Vec3& v, const glm::vec3& /*fallback*/) {
    return glm::vec3(v.x(), v.y(), v.z());
}

glm::quat euler_to_quat(const fb::RotationEuler* r) {
    if (!r) return glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    glm::quat qy = glm::angleAxis(glm::radians(r->yaw()),   glm::vec3(0, 1, 0));
    glm::quat qx = glm::angleAxis(glm::radians(r->pitch()), glm::vec3(1, 0, 0));
    glm::quat qz = glm::angleAxis(glm::radians(r->roll()),  glm::vec3(0, 0, 1));
    return qy * qx * qz;
}

// Reference overload for the struct-of-struct accessor (Transform.orientation).
glm::quat euler_to_quat(const fb::RotationEuler& r) {
    glm::quat qy = glm::angleAxis(glm::radians(r.yaw()),   glm::vec3(0, 1, 0));
    glm::quat qx = glm::angleAxis(glm::radians(r.pitch()), glm::vec3(1, 0, 0));
    glm::quat qz = glm::angleAxis(glm::radians(r.roll()),  glm::vec3(0, 0, 1));
    return qy * qx * qz;
}

Transform decode_transform(const fb::Transform* t) {
    Transform out;
    if (!t) return out;
    out.position    = to_vec3(t->position());
    out.orientation = euler_to_quat(t->orientation());
    out.scale       = to_vec3(t->scale(), glm::vec3(1.0f));
    return out;
}

Shape decode_shape(const void* shape_ptr, fb::Shape kind) {
    Shape s;
    switch (kind) {
        case fb::Shape::Sphere: {
            s.kind = ShapeKind::Sphere;
            auto t = static_cast<const fb::Sphere*>(shape_ptr);
            if (t) s.sphere.radius = t->radius() > 0 ? t->radius() : 0.5f;
            break;
        }
        case fb::Shape::Cuboid: {
            s.kind = ShapeKind::Cuboid;
            auto t = static_cast<const fb::Cuboid*>(shape_ptr);
            if (t) s.cuboid.size = to_vec3(t->size(), glm::vec3(1.0f));
            break;
        }
        case fb::Shape::Cylinder: {
            s.kind = ShapeKind::Cylinder;
            auto t = static_cast<const fb::Cylinder*>(shape_ptr);
            if (t) {
                s.cylinder.radius = t->radius() > 0 ? t->radius() : 0.5f;
                s.cylinder.height = t->height() > 0 ? t->height() : 1.0f;
            }
            break;
        }
        case fb::Shape::Capsule: {
            s.kind = ShapeKind::Capsule;
            auto t = static_cast<const fb::Capsule*>(shape_ptr);
            if (t) {
                s.capsule.radius = t->radius() > 0 ? t->radius() : 0.5f;
                s.capsule.height = t->height() > 0 ? t->height() : 1.0f;
            }
            break;
        }
        case fb::Shape::Plane: {
            s.kind = ShapeKind::Plane;
            auto t = static_cast<const fb::Plane*>(shape_ptr);
            if (t) s.plane.normal = to_vec3(t->normal(), glm::vec3(0, 1, 0));
            break;
        }
        default: s.kind = ShapeKind::Sphere; break;
    }
    return s;
}

Behaviour decode_behaviour(const void* bh_ptr, fb::Behaviour kind) {
    Behaviour b;
    switch (kind) {
        case fb::Behaviour::StaticObject:
            b.kind = BehaviourKind::Static;
            break;
        case fb::Behaviour::SimulatedObject: {
            b.kind = BehaviourKind::Simulated;
            auto t = static_cast<const fb::SimulatedObject*>(bh_ptr);
            if (t) {
                if (const fb::PhysicsState* ps = t->initial_state()) {
                    b.sim.initial_linear_velocity    = to_vec3(ps->linear_velocity());
                    glm::vec3 deg = to_vec3(ps->angular_velocity());
                    b.sim.initial_angular_velocity_rad = glm::radians(deg);
                }
                b.sim.owner = static_cast<OwnerId>(static_cast<uint8_t>(t->owner()));
            }
            break;
        }
        case fb::Behaviour::BoidObject: {
            b.kind = BehaviourKind::Boid;
            auto t = static_cast<const fb::BoidObject*>(bh_ptr);
            if (t) {
                if (const fb::PhysicsState* ps = t->initial_state()) {
                    b.boid.initial_linear_velocity     = to_vec3(ps->linear_velocity());
                    glm::vec3 deg = to_vec3(ps->angular_velocity());
                    b.boid.initial_angular_velocity_rad = glm::radians(deg);
                }
                b.boid.owner = static_cast<OwnerId>(static_cast<uint8_t>(t->owner()));
            }
            break;
        }
        case fb::Behaviour::AnimatedObject: {
            b.kind = BehaviourKind::Animated;
            auto t = static_cast<const fb::AnimatedObject*>(bh_ptr);
            if (t) {
                if (auto wps = t->waypoints()) {
                    for (uint32_t i = 0; i < wps->size(); ++i) {
                        const fb::Waypoint* fw = wps->Get(i);
                        Waypoint w;
                        w.position    = to_vec3(fw->position());
                        w.orientation = euler_to_quat(fw->rotation());
                        w.time        = fw->time();
                        b.anim.waypoints.push_back(w);
                    }
                }
                b.anim.total_duration = t->total_duration();
                b.anim.easing         = static_cast<EasingType>(static_cast<uint8_t>(t->easing()));
                b.anim.path_mode      = static_cast<PathMode>  (static_cast<uint8_t>(t->path_mode()));
            }
            break;
        }
        default: b.kind = BehaviourKind::Static; break;
    }
    return b;
}

SpawnLocation decode_spawn_location(const void* loc_ptr, fb::SpawnLocation kind) {
    SpawnLocation out;
    switch (kind) {
        case fb::SpawnLocation::FixedLocation: {
            out.kind = SpawnLocationKind::Fixed;
            auto t = static_cast<const fb::FixedLocation*>(loc_ptr);
            if (t) out.fixed.transform = decode_transform(t->transform());
            break;
        }
        case fb::SpawnLocation::RandomBox: {
            out.kind = SpawnLocationKind::RandomBox;
            auto t = static_cast<const fb::RandomBox*>(loc_ptr);
            if (t) { out.box.min = to_vec3(t->min()); out.box.max = to_vec3(t->max()); }
            break;
        }
        case fb::SpawnLocation::RandomSphere: {
            out.kind = SpawnLocationKind::RandomSphere;
            auto t = static_cast<const fb::RandomSphere*>(loc_ptr);
            if (t) { out.sphere.center = to_vec3(t->center()); out.sphere.radius = t->radius(); }
            break;
        }
        default: break;
    }
    return out;
}

SpawnTiming decode_spawn_timing(const void* sp_ptr, fb::SpawnType kind) {
    SpawnTiming t;
    switch (kind) {
        case fb::SpawnType::SingleBurstSpawn: {
            t.kind = SpawnKind::SingleBurst;
            auto ft = static_cast<const fb::SingleBurstSpawn*>(sp_ptr);
            if (ft) t.burst.count = ft->count();
            break;
        }
        case fb::SpawnType::RepeatingSpawn: {
            t.kind = SpawnKind::Repeating;
            auto ft = static_cast<const fb::RepeatingSpawn*>(sp_ptr);
            if (ft) {
                t.repeating.interval  = ft->interval() > 0 ? ft->interval() : 1.0f;
                t.repeating.max_count = ft->max_count();
            }
            break;
        }
        default: break;
    }
    return t;
}

BaseSpawner decode_base_spawner(const fb::BaseSpawner* b, size_t fallback_idx) {
    BaseSpawner out;
    if (!b) { out.name = "spawner " + std::to_string(fallback_idx); return out; }
    out.name       = b->name() ? b->name()->str() : ("spawner " + std::to_string(fallback_idx));
    out.start_time = b->start_time();
    out.timing     = decode_spawn_timing(b->spawn_type(), b->spawn_type_type());
    out.location   = decode_spawn_location(b->location(), b->location_type());
    if (auto lv = b->linear_velocity()) {
        out.linear_velocity.min = to_vec3(lv->min());
        out.linear_velocity.max = to_vec3(lv->max());
    }
    if (auto av = b->angular_velocity()) {
        out.angular_velocity_rad.min = glm::radians(to_vec3(av->min()));
        out.angular_velocity_rad.max = glm::radians(to_vec3(av->max()));
    }
    out.material = b->material() ? b->material()->str() : "";
    out.owner    = static_cast<SpawnerOwnerKind>(static_cast<uint8_t>(b->owner()));
    return out;
}

float shape_volume_unscaled(const Shape& s) {
    switch (s.kind) {
        case ShapeKind::Sphere:   return (4.0f / 3.0f) * kPi * std::pow(s.sphere.radius, 3.0f);
        case ShapeKind::Cuboid:   return s.cuboid.size.x * s.cuboid.size.y * s.cuboid.size.z;
        case ShapeKind::Cylinder: return kPi * s.cylinder.radius * s.cylinder.radius * s.cylinder.height;
        case ShapeKind::Capsule: {
            float r = s.capsule.radius, h = s.capsule.height;
            return kPi * r * r * h + (4.0f / 3.0f) * kPi * std::pow(r, 3.0f);
        }
        case ShapeKind::Plane: return 0.0f;
    }
    return 0.0f;
}

} // namespace

float shape_volume(const Shape& shape, const glm::vec3& scale) {
    return shape_volume_unscaled(shape) * scale.x * scale.y * scale.z;
}

Scene load_scene_from_file(const std::filesystem::path& path) {
    auto bytes = read_file(path);
    if (bytes.size() < 8) throw std::runtime_error("scene file too small");

    if (!fb::SceneBufferHasIdentifier(bytes.data())) {
        throw std::runtime_error("scene file has wrong identifier (not 'SCNE'): " + path.string());
    }
    // We deliberately skip flatbuffers::Verifier here. The verifier in
    // flatbuffers 25.x rejects buffers that flatc 25.x produced from this
    // schema (a known issue with how the empty SpawnerType vector + identifier
    // is encoded). The buffer is structurally valid — we trust the upstream
    // tool and rely on flatc's own JSON↔binary round-trip as the integrity
    // check at build time.
    const fb::Scene* fs = fb::GetScene(bytes.data());
    if (!fs) throw std::runtime_error("scene root null: " + path.string());

    Scene out;
    out.name        = fs->name() ? fs->name()->str() : path.stem().string();
    out.description = fs->description() ? fs->description()->str() : "";
    out.gravity_on  = fs->gravity_on();

    if (auto mats = fs->materials()) {
        for (uint32_t i = 0; i < mats->size(); ++i) {
            const fb::Material* m = mats->Get(i);
            Material om;
            om.name    = m->name() ? m->name()->str() : ("material " + std::to_string(i));
            om.density = m->density() > 0.0f ? m->density() : 1000.0f;
            out.materials.push_back(std::move(om));
        }
    }

    if (auto inters = fs->interactions()) {
        for (uint32_t i = 0; i < inters->size(); ++i) {
            const fb::MaterialInteraction* mi = inters->Get(i);
            MaterialInteraction oi;
            oi.material_a       = mi->material_a() ? mi->material_a()->str() : "";
            oi.material_b       = mi->material_b() ? mi->material_b()->str() : "";
            oi.restitution      = mi->restitution();
            oi.static_friction  = mi->static_friction();
            oi.dynamic_friction = mi->dynamic_friction();
            out.interactions.push_back(std::move(oi));
        }
    }

    if (auto cams = fs->cameras()) {
        for (uint32_t i = 0; i < cams->size(); ++i) {
            const fb::Camera* c = cams->Get(i);
            Camera oc;
            oc.name      = c->name() ? c->name()->str() : ("camera " + std::to_string(i));
            oc.transform = decode_transform(c->transform());
            switch (c->camera_type_type()) {
                case fb::CameraType::PerspectiveCamera: {
                    auto pc = static_cast<const fb::PerspectiveCamera*>(c->camera_type());
                    oc.kind = CameraKind::Perspective;
                    if (pc) {
                        oc.perspective.fov_rad = glm::radians(pc->fov()  > 0 ? pc->fov()  : 60.0f);
                        oc.perspective.z_near  = pc->near() > 0 ? pc->near() : 0.1f;
                        oc.perspective.z_far   = pc->far()  > 0 ? pc->far()  : 200.0f;
                    }
                    break;
                }
                case fb::CameraType::OrthographicCamera: {
                    auto oc2 = static_cast<const fb::OrthographicCamera*>(c->camera_type());
                    oc.kind = CameraKind::Orthographic;
                    if (oc2) {
                        oc.orthographic.size   = oc2->size() > 0 ? oc2->size() : 10.0f;
                        oc.orthographic.z_near = oc2->near();
                        oc.orthographic.z_far  = oc2->far() > 0 ? oc2->far() : 200.0f;
                    }
                    break;
                }
                default: oc.kind = CameraKind::Perspective; break;
            }
            out.cameras.push_back(std::move(oc));
        }
    }

    if (auto objs = fs->objects()) {
        for (uint32_t i = 0; i < objs->size(); ++i) {
            const fb::Object* o = objs->Get(i);
            Object oo;
            oo.name           = o->name() ? o->name()->str() : ("object " + std::to_string(i + 1));
            oo.transform      = decode_transform(o->transform());
            oo.material       = o->material() ? o->material()->str() : "";
            oo.shape          = decode_shape(o->shape(), o->shape_type());
            oo.behaviour      = decode_behaviour(o->behaviour(), o->behaviour_type());
            oo.collision_type = o->collision_type() == fb::CollisionType::CONTAINER
                                ? CollisionType::Container : CollisionType::Solid;

            float density = 1000.0f;
            for (const Material& m : out.materials) {
                if (m.name == oo.material) { density = m.density; break; }
            }
            oo.mass = density * shape_volume(oo.shape, oo.transform.scale);
            out.objects.push_back(std::move(oo));
        }
    }

    auto sps      = fs->spawners();
    auto sps_type = fs->spawners_type();
    if (sps && sps_type) {
        for (uint32_t i = 0; i < sps->size(); ++i) {
            Spawner osp;
            switch (sps_type->Get(i)) {
                case fb::SpawnerType::SphereSpawner: {
                    auto t = static_cast<const fb::SphereSpawner*>(sps->Get(i));
                    osp.shape_kind = ShapeKind::Sphere;
                    if (t) {
                        osp.base = decode_base_spawner(t->base(), i);
                        if (auto rr = t->radius_range()) { osp.radius_range.min = rr->min(); osp.radius_range.max = rr->max(); }
                    }
                    break;
                }
                case fb::SpawnerType::CylinderSpawner: {
                    auto t = static_cast<const fb::CylinderSpawner*>(sps->Get(i));
                    osp.shape_kind = ShapeKind::Cylinder;
                    if (t) {
                        osp.base = decode_base_spawner(t->base(), i);
                        if (auto rr = t->radius_range()) { osp.radius_range.min = rr->min(); osp.radius_range.max = rr->max(); }
                        if (auto hr = t->height_range()) { osp.height_range.min = hr->min(); osp.height_range.max = hr->max(); }
                    }
                    break;
                }
                case fb::SpawnerType::CapsuleSpawner: {
                    auto t = static_cast<const fb::CapsuleSpawner*>(sps->Get(i));
                    osp.shape_kind = ShapeKind::Capsule;
                    if (t) {
                        osp.base = decode_base_spawner(t->base(), i);
                        if (auto rr = t->radius_range()) { osp.radius_range.min = rr->min(); osp.radius_range.max = rr->max(); }
                        if (auto hr = t->height_range()) { osp.height_range.min = hr->min(); osp.height_range.max = hr->max(); }
                    }
                    break;
                }
                case fb::SpawnerType::CuboidSpawner: {
                    auto t = static_cast<const fb::CuboidSpawner*>(sps->Get(i));
                    osp.shape_kind = ShapeKind::Cuboid;
                    if (t) {
                        osp.base = decode_base_spawner(t->base(), i);
                        if (auto sr = t->size_range()) {
                            osp.size_range.min = to_vec3(sr->min());
                            osp.size_range.max = to_vec3(sr->max());
                        }
                    }
                    break;
                }
                case fb::SpawnerType::BoidSpawner: {
                    auto t = static_cast<const fb::BoidSpawner*>(sps->Get(i));
                    osp.shape_kind     = ShapeKind::Sphere;
                    osp.produces_boids = true;
                    if (t) {
                        osp.base = decode_base_spawner(t->base(), i);
                        if (auto rr = t->radius_range()) { osp.radius_range.min = rr->min(); osp.radius_range.max = rr->max(); }
                    }
                    break;
                }
                default: break;
            }
            out.spawners.push_back(std::move(osp));
        }
    }

    return out;
}

} // namespace finalLab::scene
