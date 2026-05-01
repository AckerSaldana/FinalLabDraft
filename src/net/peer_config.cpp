#include "net/peer_config.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace finalLab::net {

PeerConfig load_peer_config(const std::filesystem::path& peers_file, int my_peer_id) {
    PeerConfig cfg;
    cfg.my_peer_id = my_peer_id;

    std::ifstream f(peers_file);
    if (!f) throw std::runtime_error("cannot open peers file: " + peers_file.string());

    std::string line;
    int line_no = 0;
    while (std::getline(f, line)) {
        ++line_no;
        size_t lead = line.find_first_not_of(" \t\r\n");
        if (lead == std::string::npos) continue;
        if (line[lead] == '#') continue;

        size_t colon = line.find(':', lead);
        if (colon == std::string::npos) {
            throw std::runtime_error("malformed peers line " + std::to_string(line_no) + ": " + line);
        }
        PeerEndpoint pe;
        pe.peer_id = static_cast<int>(cfg.peers.size()) + 1;
        pe.address = line.substr(lead, colon - lead);

        size_t end = line.find_first_of(" \t\r\n#", colon + 1);
        std::string port_str = (end == std::string::npos)
                               ? line.substr(colon + 1)
                               : line.substr(colon + 1, end - colon - 1);
        pe.port = static_cast<uint16_t>(std::stoul(port_str));

        cfg.peers.push_back(std::move(pe));
    }

    if (cfg.peers.empty()) throw std::runtime_error("peers file empty: " + peers_file.string());
    if (my_peer_id < 1 || my_peer_id > static_cast<int>(cfg.peers.size())) {
        throw std::runtime_error("--peer " + std::to_string(my_peer_id) + " out of range (have "
                                 + std::to_string(cfg.peers.size()) + " peers)");
    }
    return cfg;
}

PeerConfig single_peer_config() {
    PeerConfig cfg;
    cfg.my_peer_id = 1;
    PeerEndpoint pe;
    pe.peer_id = 1;
    pe.address = "127.0.0.1";
    pe.port    = 0;
    cfg.peers.push_back(pe);
    return cfg;
}

} // namespace finalLab::net
