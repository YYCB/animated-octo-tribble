#pragma once

#include "itransport.hpp"

#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace chassis {

class Rs485Transport final : public ITransport {
public:
    explicit Rs485Transport(const TransportConfig& config);
    ~Rs485Transport() override;

    bool send(const uint8_t* data, std::size_t len) override;
    void setReceiveCallback(std::function<void(const uint8_t*, std::size_t)> cb) override;

    bool        connect()            override;
    void        disconnect()         override;
    bool        isConnected() const  override;
    std::string getLastError() const override;

private:
    TransportConfig config_;

    int              fd_{-1};
    std::atomic<bool> connected_{false};
    std::atomic<bool> running_{false};

    std::thread receive_thread_;
    std::mutex  send_mutex_;
    mutable std::mutex error_mutex_;

    std::string last_error_;
    std::function<void(const uint8_t*, std::size_t)> receive_cb_;

    bool configurePort();
    void receiveLoop();
    void setError(const std::string& err);
};

} // namespace chassis
