#include "net/network_runtime.hpp"

#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <thread>

namespace finalLab::net {

namespace {

bool g_winsock_initialized = false;

void ensure_winsock() {
    if (g_winsock_initialized) return;
    WSADATA wsa{};
    int rc = WSAStartup(MAKEWORD(2, 2), &wsa);
    if (rc != 0) throw std::runtime_error("WSAStartup failed: " + std::to_string(rc));
    g_winsock_initialized = true;
}

} // namespace

NetworkRuntime::NetworkRuntime() = default;

NetworkRuntime::~NetworkRuntime() {
    stop();
}

void NetworkRuntime::start(PeerConfig cfg, uint64_t affinity_mask) {
    if (running_.load()) return;
    config_         = std::move(cfg);
    affinity_mask_  = affinity_mask;

    if (!config_.networked()) return;

    ensure_winsock();

    SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s == INVALID_SOCKET) throw std::runtime_error("socket() failed");

    u_long non_blocking = 1;
    if (ioctlsocket(s, FIONBIO, &non_blocking) != 0) {
        closesocket(s);
        throw std::runtime_error("ioctlsocket FIONBIO failed");
    }

    const PeerEndpoint& me = config_.peers[config_.my_peer_id - 1];

    sockaddr_in bind_addr{};
    bind_addr.sin_family      = AF_INET;
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind_addr.sin_port        = htons(me.port);
    if (bind(s, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr)) == SOCKET_ERROR) {
        int e = WSAGetLastError();
        closesocket(s);
        throw std::runtime_error("bind() failed on port " + std::to_string(me.port)
                                 + " (WSA " + std::to_string(e) + ")");
    }

    socket_ = static_cast<intptr_t>(s);
    running_.store(true);
    thread_ = std::thread([this] { thread_main(); });
}

void NetworkRuntime::stop() {
    if (!running_.load()) {
        if (socket_ != -1) {
            closesocket(static_cast<SOCKET>(socket_));
            socket_ = -1;
        }
        return;
    }
    running_.store(false);
    if (thread_.joinable()) thread_.join();
    if (socket_ != -1) {
        closesocket(static_cast<SOCKET>(socket_));
        socket_ = -1;
    }
}

void NetworkRuntime::queue_outbound(std::vector<StateEntry> entries) {
    if (!running_.load()) return;
    std::scoped_lock lock(outbound_mutex_);
    outbound_pending_.insert(outbound_pending_.end(),
                             std::make_move_iterator(entries.begin()),
                             std::make_move_iterator(entries.end()));
}

void NetworkRuntime::drain_inbound(std::vector<InboundState>& out) {
    std::scoped_lock lock(inbound_mutex_);
    out = std::move(inbound_pending_);
    inbound_pending_.clear();
}

void NetworkRuntime::send_to_peers(const std::vector<StateEntry>& entries) {
    if (entries.empty()) return;

    int loss_pct = sim_loss_pct_.load();
    int latency  = sim_latency_ms_.load();
    int jitter   = sim_jitter_ms_.load();
    auto now     = std::chrono::steady_clock::now();

    char buf[kPacketBufferSize];

    size_t pos = 0;
    while (pos < entries.size()) {
        size_t batch = std::min(entries.size() - pos, kMaxStateEntriesPerPacket);

        PacketHeader hdr{};
        hdr.msg_type        = static_cast<uint8_t>(MsgType::State);
        hdr.sender_peer_id  = static_cast<uint8_t>(config_.my_peer_id);
        hdr.sequence        = send_sequence_.fetch_add(1);
        hdr.entry_count     = static_cast<uint16_t>(batch);
        hdr.reserved        = 0;

        std::memcpy(buf, &hdr, sizeof(hdr));
        std::memcpy(buf + sizeof(hdr), entries.data() + pos, batch * sizeof(StateEntry));
        size_t total = sizeof(hdr) + batch * sizeof(StateEntry);

        for (const auto& peer : config_.peers) {
            if (peer.peer_id == config_.my_peer_id) continue;

            if (loss_pct > 0) {
                std::uniform_int_distribution<int> roll(1, 100);
                if (roll(sim_rng_) <= loss_pct) {
                    packets_dropped_sim_.fetch_add(1);
                    continue;
                }
            }

            int delay_ms = latency;
            if (jitter > 0) {
                std::uniform_int_distribution<int> j(-jitter, jitter);
                delay_ms = std::max(0, latency + j(sim_rng_));
            }

            sockaddr_in to{};
            to.sin_family = AF_INET;
            to.sin_port   = htons(peer.port);
            inet_pton(AF_INET, peer.address.c_str(), &to.sin_addr);

            if (delay_ms <= 0) {
                int sent = sendto(static_cast<SOCKET>(socket_), buf, static_cast<int>(total), 0,
                                  reinterpret_cast<sockaddr*>(&to), sizeof(to));
                if (sent > 0) {
                    bytes_sent_.fetch_add(sent);
                    packets_sent_.fetch_add(1);
                }
            } else {
                PendingSend ps;
                ps.ready_at      = now + std::chrono::milliseconds(delay_ms);
                ps.bytes.assign(buf, buf + total);
                ps.dest_addr_be  = to.sin_addr.s_addr;
                ps.dest_port_be  = to.sin_port;
                pending_sends_.push_back(std::move(ps));
            }
        }
        pos += batch;
    }
}

void NetworkRuntime::recv_pending() {
    char buf[kPacketBufferSize];
    SOCKET s = static_cast<SOCKET>(socket_);
    sockaddr_in from{};
    int from_len = sizeof(from);

    while (true) {
        int n = recvfrom(s, buf, kPacketBufferSize, 0,
                         reinterpret_cast<sockaddr*>(&from), &from_len);
        if (n == SOCKET_ERROR) {
            int e = WSAGetLastError();
            if (e == WSAEWOULDBLOCK) break;
            break;
        }
        if (n < static_cast<int>(sizeof(PacketHeader))) continue;

        PacketHeader hdr{};
        std::memcpy(&hdr, buf, sizeof(hdr));
        if (hdr.msg_type != static_cast<uint8_t>(MsgType::State)) continue;

        size_t expected = sizeof(PacketHeader) + hdr.entry_count * sizeof(StateEntry);
        if (static_cast<size_t>(n) < expected) continue;

        uint8_t sender = hdr.sender_peer_id;
        if (sender == 0 || sender >= last_seq_per_peer_.size()) continue;
        if (seen_peer_[sender]) {
            int16_t delta = static_cast<int16_t>(hdr.sequence - last_seq_per_peer_[sender]);
            if (delta <= 0) {
                packets_dropped_stale_.fetch_add(1);
                continue;
            }
        }
        seen_peer_[sender]        = true;
        last_seq_per_peer_[sender] = hdr.sequence;

        std::vector<InboundState> staged;
        staged.reserve(hdr.entry_count);
        for (uint16_t i = 0; i < hdr.entry_count; ++i) {
            InboundState in;
            in.sender_peer_id = sender;
            std::memcpy(&in.entry, buf + sizeof(PacketHeader) + i * sizeof(StateEntry), sizeof(StateEntry));
            staged.push_back(in);
        }
        bytes_received_.fetch_add(n);
        packets_received_.fetch_add(1);

        std::scoped_lock lock(inbound_mutex_);
        inbound_pending_.insert(inbound_pending_.end(),
                                std::make_move_iterator(staged.begin()),
                                std::make_move_iterator(staged.end()));
    }
}

void NetworkRuntime::thread_main() {
    if (affinity_mask_ != 0) {
        SetThreadAffinityMask(GetCurrentThread(), static_cast<DWORD_PTR>(affinity_mask_));
    }

    while (running_.load()) {
        recv_pending();

        auto now = std::chrono::steady_clock::now();
        SOCKET s = static_cast<SOCKET>(socket_);
        for (auto it = pending_sends_.begin(); it != pending_sends_.end(); ) {
            if (it->ready_at <= now) {
                sockaddr_in to{};
                to.sin_family = AF_INET;
                to.sin_addr.s_addr = it->dest_addr_be;
                to.sin_port        = it->dest_port_be;
                int sent = sendto(s, it->bytes.data(), static_cast<int>(it->bytes.size()), 0,
                                  reinterpret_cast<sockaddr*>(&to), sizeof(to));
                if (sent > 0) {
                    bytes_sent_.fetch_add(sent);
                    packets_sent_.fetch_add(1);
                }
                it = pending_sends_.erase(it);
            } else {
                ++it;
            }
        }

        std::vector<StateEntry> to_send;
        {
            std::scoped_lock lock(outbound_mutex_);
            to_send.swap(outbound_pending_);
        }
        send_to_peers(to_send);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

} // namespace finalLab::net
