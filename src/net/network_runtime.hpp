#pragma once

#include "net/peer_config.hpp"
#include "net/protocol.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

namespace finalLab::net {

struct InboundState {
    uint8_t    sender_peer_id = 0;
    StateEntry entry{};
};

struct InboundSceneSwap {
    uint8_t        sender_peer_id = 0;
    SceneSwapEntry entry{};
};

struct InboundBodyEdit {
    uint8_t       sender_peer_id = 0;
    BodyEditEntry entry{};
};

struct InboundSpawn {
    uint8_t    sender_peer_id = 0;
    SpawnEntry entry{};
};

class NetworkRuntime {
public:
    NetworkRuntime();
    ~NetworkRuntime();

    NetworkRuntime(const NetworkRuntime&) = delete;
    NetworkRuntime& operator=(const NetworkRuntime&) = delete;

    void start(PeerConfig cfg, uint64_t affinity_mask);
    void stop();

    void queue_outbound(std::vector<StateEntry> entries);
    void drain_inbound(std::vector<InboundState>& out);

    // Control-channel: small typed payloads broadcast to every peer.
    void broadcast_scene_swap(const std::string& scene_name, uint32_t version);
    void broadcast_body_edit(const BodyEditEntry& edit);
    void broadcast_spawn(const SpawnEntry& spawn);
    void drain_inbound_scene_swaps(std::vector<InboundSceneSwap>& out);
    void drain_inbound_body_edits(std::vector<InboundBodyEdit>& out);
    void drain_inbound_spawns(std::vector<InboundSpawn>& out);

    int  packets_sent()      const { return packets_sent_.load(); }
    int  packets_received()  const { return packets_received_.load(); }
    int  packets_dropped_in_sim() const { return packets_dropped_sim_.load(); }
    int  packets_dropped_stale()  const { return packets_dropped_stale_.load(); }
    int  bytes_sent()        const { return bytes_sent_.load(); }
    int  bytes_received()    const { return bytes_received_.load(); }

    void set_sim_latency_ms(int ms)  { sim_latency_ms_.store(ms); }
    void set_sim_jitter_ms(int ms)   { sim_jitter_ms_.store(ms); }
    void set_sim_loss_pct(int pct)   { sim_loss_pct_.store(pct); }
    void set_net_hz(int hz)          { net_hz_.store(hz < 1 ? 1 : (hz > 2000 ? 2000 : hz)); }
    int  net_hz_target() const       { return net_hz_.load(); }
    int  net_hz_actual() const       { return net_hz_actual_.load(); }

    bool is_running() const { return running_.load(); }
    int  my_peer_id() const { return config_.my_peer_id; }
    int  total_peers() const { return config_.total_peers(); }

private:
    void thread_main();
    void send_to_peers(const std::vector<StateEntry>& entries);
    void recv_pending();
    void send_raw_to_peers(const char* buf, size_t len);   // shared by control msgs

    std::thread          thread_;
    std::atomic<bool>    running_{false};
    uint64_t             affinity_mask_ = 0;
    PeerConfig           config_;

    intptr_t             socket_ = -1;

    std::mutex                   outbound_mutex_;
    std::vector<StateEntry>      outbound_pending_;

    std::mutex                   inbound_mutex_;
    std::vector<InboundState>    inbound_pending_;

    std::mutex                       inbound_control_mutex_;
    std::vector<InboundSceneSwap>    inbound_scene_swaps_;
    std::vector<InboundBodyEdit>     inbound_body_edits_;
    std::vector<InboundSpawn>        inbound_spawns_;

    std::atomic<uint16_t>        send_sequence_{0};
    std::atomic<int>             packets_sent_{0};
    std::atomic<int>             packets_received_{0};
    std::atomic<int>             packets_dropped_sim_{0};
    std::atomic<int>             packets_dropped_stale_{0};
    std::atomic<int>             bytes_sent_{0};
    std::atomic<int>             bytes_received_{0};

    std::atomic<int>             sim_latency_ms_{0};
    std::atomic<int>             sim_jitter_ms_{0};
    std::atomic<int>             sim_loss_pct_{0};
    std::atomic<int>             net_hz_{1000};       // UI-controlled poll rate
    std::atomic<int>             net_hz_actual_{0};   // measured from loop iterations

    std::array<uint16_t, 16>     last_seq_per_peer_{};
    std::array<bool, 16>         seen_peer_{};

    struct PendingSend {
        std::chrono::steady_clock::time_point ready_at;
        std::vector<char>                     bytes;
        uint32_t                              dest_addr_be = 0;
        uint16_t                              dest_port_be = 0;
    };
    std::vector<PendingSend>     pending_sends_;
    std::mt19937                 sim_rng_{0xC1A551C};
};

} // namespace finalLab::net
