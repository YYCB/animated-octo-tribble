#pragma once

#include "../frame/frame_codec.hpp"
#include "../transport/itransport.hpp"

#include <chrono>
#include <cstdint>
#include <string>

namespace chassis {

class VersionNegotiator {
public:
    enum class State { IDLE, HANDSHAKE_SENT, CONNECTED, FAILED };

    explicit VersionNegotiator(ITransport& transport, uint32_t protocol_version);

    // Sends HANDSHAKE_REQ frame. Returns false if send fails.
    bool sendHandshake();

    // Call when a complete frame arrives from the transport.
    void onFrameReceived(const DecodedFrame& frame);

    bool        isNegotiated()        const;
    uint32_t    getNegotiatedVersion() const;
    std::string getChassisType()       const;
    std::string getRejectReason()      const;
    State       getState()             const;

    // Call periodically. Returns true if timed-out & max retries exceeded.
    bool checkTimeout();

    void reset();

private:
    ITransport&  transport_;
    uint32_t     protocol_version_;
    State        state_{State::IDLE};
    uint32_t     negotiated_version_{0};
    std::string  chassis_type_;
    std::string  reject_reason_;

    int retries_{0};
    static constexpr int    MAX_RETRIES     = 3;
    static constexpr long   TIMEOUT_MS      = 2000;

    std::chrono::steady_clock::time_point last_send_time_{};

    // Serialise / deserialise binary handshake payloads
    static std::vector<uint8_t> buildHandshakeReqPayload(uint32_t protocol_version);
    void handleHandshakeResp(const DecodedFrame& frame);
};

} // namespace chassis
