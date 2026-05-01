#pragma once

#include <cstdint>

namespace finalLab::net {

enum class MsgType : uint8_t {
    State = 1,
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
#pragma pack(pop)

constexpr size_t kMaxStateEntriesPerPacket = 24;
constexpr size_t kPacketBufferSize         = 1500;

} // namespace finalLab::net
