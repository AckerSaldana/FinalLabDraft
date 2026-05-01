#pragma once

#include <cstdint>

namespace finalLab::net {

enum class MsgType : uint8_t {
    State     = 1,
    SceneSwap = 2,
    BodyEdit  = 3,
    Spawn     = 4,
};

#pragma pack(push, 1)
struct PacketHeader {
    uint8_t   msg_type;
    uint8_t   sender_peer_id;
    uint16_t  sequence;
    uint16_t  entry_count;
    uint16_t  reserved;
};

struct StateEntry {
    uint32_t  body_id;
    float     px, py, pz;
    float     qw, qx, qy, qz;
    float     lvx, lvy, lvz;
    float     avx, avy, avz;
};

constexpr size_t kSceneNameMax = 56;
struct SceneSwapEntry {
    uint32_t  version;                    // monotonic; receivers ignore < latest
    char      name[kSceneNameMax];        // null-padded scene stem
};

// Body edit broadcast — applied verbatim by every peer; the body's owner is
// authoritative going forward but the edit is one-shot so any peer can issue it.
struct BodyEditEntry {
    uint32_t  body_id;
    float     px, py, pz;
    float     qw, qx, qy, qz;
    float     lvx, lvy, lvz;
    float     avx, avy, avz;
};

// Spawn broadcast — owner peer creates the object, payload describes a runtime
// scene::Object compactly enough for receivers to instantiate it identically.
enum class SpawnShapeKind : uint8_t { Sphere = 0, Cuboid = 1, Cylinder = 2, Capsule = 3, Plane = 4 };
enum class SpawnBehaviourKind : uint8_t { Simulated = 0, Boid = 1 };

struct SpawnEntry {
    uint32_t  body_id;            // canonical scene_object_index assigned by owner
    uint8_t   owner;              // 0..3 → OwnerId One..Four
    uint8_t   shape_kind;         // SpawnShapeKind
    uint8_t   behaviour_kind;     // SpawnBehaviourKind
    uint8_t   reserved;
    float     px, py, pz;
    float     qw, qx, qy, qz;
    float     sx, sy, sz;
    float     lvx, lvy, lvz;
    float     avx, avy, avz;
    float     dim0, dim1, dim2;   // shape-specific: sphere=(r,_,_), cyl/cap=(r,h,_), cuboid=(x,y,z), plane=(nx,ny,nz)
    char      material[32];       // null-padded
};
#pragma pack(pop)

constexpr size_t kMaxStateEntriesPerPacket = 24;
constexpr size_t kPacketBufferSize         = 1500;
constexpr size_t kMaxBodyEditsPerPacket    = 24;
constexpr size_t kMaxSpawnsPerPacket       = 6;   // ~190 B each, 6 keeps us under MTU

} // namespace finalLab::net
