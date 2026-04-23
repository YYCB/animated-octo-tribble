#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

// C++17 compatible span (see frame_codec.hpp for the full template)
#include "../frame/frame_codec.hpp"

namespace chassis {

struct TransportConfig {
    std::string host;
    uint16_t    port{0};
    std::string device_path;
    uint32_t    baud_rate{115200};
    uint32_t    timeout_ms{1000};
};

enum class TransportType { TCP, UDP, RS485 };

class ITransport {
public:
    virtual ~ITransport() = default;

    virtual bool send(const uint8_t* data, std::size_t len) = 0;
    bool send(const std::vector<uint8_t>& data) {
        return send(data.data(), data.size());
    }
    bool send(Span<const uint8_t> data) {
        return send(data.data(), data.size());
    }

    virtual void setReceiveCallback(std::function<void(const uint8_t*, std::size_t)> cb) = 0;

    virtual bool        connect()                  = 0;
    virtual void        disconnect()               = 0;
    virtual bool        isConnected() const        = 0;
    virtual std::string getLastError() const       = 0;

    static std::unique_ptr<ITransport> create(TransportType type, const TransportConfig& config);
};

} // namespace chassis
