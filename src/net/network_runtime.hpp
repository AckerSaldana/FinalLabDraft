#pragma once

#include "net/peer_config.hpp"
#include "net/protocol.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <random>
#include <thread>
#include <vector>

namespace finalLab::net {

struct InboundState {
    uint8_t    sender_peer_id = 0;
    StateEntry entry{};
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

    int  packets_sent()      const { return packets_sent_.load(); }
    int  packets_received()  const { return packets_received_.load(); }
    int  packets_dropped_in_sim() const { return packets_dropped_sim_.load(); }
    int  packets_dropped_stale()  const { return packets_dropped_stale_.load(); }
    int  bytes_sent()        const { return bytes_sent_.load(); }
    int  bytes_received()    const { return bytes_received_.load(); }

    void set_sim_latency_ms(int ms)  { sim_latency_ms_.store(ms); }
    void set_sim_jitter_ms(int ms)   { sim_jitter_ms_.store(ms); }
    void set_sim_loss_pct(int pct)   { sim_loss_pct_.store(pct); }

    bool is_running() const { return running_.load(); }
    int  my_peer_id() const { return config_.my_peer_id; }
    int  total_peers() const { return config_.total_peers(); }

private:
    void thread_main();
    void send_to_peers(const std::vector<StateEntry>& entries);
    void recv_pending();

    std::thread          thread_;
    std::atomic<bool>    running_{false};
    uint64_t             affinity_mask_ = 0;
    PeerConfig           config_;

    intptr_t             socket_ = -1;

    std::mutex                   outbound_mutex_;
    std::vector<StateEntry>      outbound_pending_;

    std::mutex                   inbound_mutex_;
    std::vector<InboundState>    inbound_pending_;

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
