#pragma once

#include "itransport.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace chassis {

class UdpTransport final : public ITransport {
public:
    explicit UdpTransport(const TransportConfig& config);
    ~UdpTransport() override;

    bool send(const uint8_t* data, std::size_t len) override;
    void setReceiveCallback(std::function<void(const uint8_t*, std::size_t)> cb) override;

    bool        connect()            override;
    void        disconnect()         override;
    bool        isConnected() const  override;
    std::string getLastError() const override;

private:
    TransportConfig config_;

    int              socket_fd_{-1};
    std::atomic<bool> connected_{false};
    std::atomic<bool> running_{false};

    struct sockaddr_in remote_addr_{};
    struct sockaddr_in local_addr_{};

    std::thread receive_thread_;
    std::mutex  send_mutex_;
    mutable std::mutex error_mutex_;

    std::string last_error_;
    std::function<void(const uint8_t*, std::size_t)> receive_cb_;

    void receiveLoop();
    void setError(const std::string& err);
};

} // namespace chassis
