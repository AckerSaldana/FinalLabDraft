#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace finalLab::net {

struct PeerEndpoint {
    int         peer_id = 0;
    std::string address;
    uint16_t    port    = 0;
};

struct PeerConfig {
    int                       my_peer_id = 1;
    std::vector<PeerEndpoint> peers;

    bool networked() const { return peers.size() >= 2; }
    int  total_peers() const { return static_cast<int>(peers.size()); }
};

PeerConfig load_peer_config(const std::filesystem::path& peers_file, int my_peer_id);
PeerConfig single_peer_config();

} // namespace finalLab::net
