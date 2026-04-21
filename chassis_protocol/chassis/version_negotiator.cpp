#include "version_negotiator.hpp"

#include <cstring>

namespace chassis {

// Supported command IDs advertised during handshake
static const uint8_t SUPPORTED_CMDS[] = {
    0x01, 0x02, 0x10, 0x11, 0x12, 0x13, 0x20, 0x21, 0x22, 0xFF
};
static const char CONFIG_HASH[] = "chassis_hal_v1";

VersionNegotiator::VersionNegotiator(ITransport& transport, uint32_t protocol_version)
    : transport_(transport), protocol_version_(protocol_version) {}

// ─── Binary handshake payload builders ───────────────────────────────────────
//
// HandshakeRequest payload layout (all little-endian):
//   protocol_version : uint32_t
//   num_cmds         : uint16_t
//   supported_cmds[] : uint8_t * num_cmds
//   config_hash_len  : uint8_t
//   config_hash[]    : char * config_hash_len

std::vector<uint8_t> VersionNegotiator::buildHandshakeReqPayload(uint32_t protocol_version) {
    const uint8_t  num_cmds        = static_cast<uint8_t>(sizeof(SUPPORTED_CMDS));
    const uint8_t  hash_len        = static_cast<uint8_t>(sizeof(CONFIG_HASH) - 1); // exclude NUL

    std::vector<uint8_t> payload;
    payload.reserve(4 + 2 + num_cmds + 1 + hash_len);

    // protocol_version (uint32_t LE)
    payload.push_back(static_cast<uint8_t>(protocol_version & 0xFF));
    payload.push_back(static_cast<uint8_t>((protocol_version >> 8) & 0xFF));
    payload.push_back(static_cast<uint8_t>((protocol_version >> 16) & 0xFF));
    payload.push_back(static_cast<uint8_t>((protocol_version >> 24) & 0xFF));

    // num_cmds (uint16_t LE)
    payload.push_back(static_cast<uint8_t>(num_cmds & 0xFF));
    payload.push_back(static_cast<uint8_t>((num_cmds >> 8) & 0xFF));

    // supported_cmds[]
    for (uint8_t i = 0; i < num_cmds; ++i) {
        payload.push_back(SUPPORTED_CMDS[i]);
    }

    // config_hash_len + config_hash
    payload.push_back(hash_len);
    for (uint8_t i = 0; i < hash_len; ++i) {
        payload.push_back(static_cast<uint8_t>(CONFIG_HASH[i]));
    }

    return payload;
}

// ─── Public interface ─────────────────────────────────────────────────────────

bool VersionNegotiator::sendHandshake() {
    auto payload = buildHandshakeReqPayload(protocol_version_);
    auto frame   = FrameCodec::encode(CmdId::HANDSHAKE_REQ, payload);

    if (!transport_.send(frame.data(), frame.size())) {
        return false;
    }

    state_          = State::HANDSHAKE_SENT;
    last_send_time_ = std::chrono::steady_clock::now();
    return true;
}

void VersionNegotiator::onFrameReceived(const DecodedFrame& frame) {
    if (frame.cmdId() != CmdId::HANDSHAKE_RESP) {
        return;
    }
    if (state_ != State::HANDSHAKE_SENT) {
        return;
    }
    handleHandshakeResp(frame);
}

// HandshakeResponse payload layout (all little-endian):
//   accepted_version   : uint32_t
//   accepted           : uint8_t  (0 = false, 1 = true)
//   chassis_type_len   : uint8_t
//   chassis_type[]     : char * chassis_type_len
//   reject_reason_len  : uint8_t
//   reject_reason[]    : char * reject_reason_len

void VersionNegotiator::handleHandshakeResp(const DecodedFrame& frame) {
    const auto& p = frame.payload;
    std::size_t offset = 0;

    auto readU32 = [&]() -> uint32_t {
        if (offset + 4 > p.size()) return 0;
        uint32_t v = static_cast<uint32_t>(p[offset])
                   | (static_cast<uint32_t>(p[offset+1]) << 8)
                   | (static_cast<uint32_t>(p[offset+2]) << 16)
                   | (static_cast<uint32_t>(p[offset+3]) << 24);
        offset += 4;
        return v;
    };
    auto readU8 = [&]() -> uint8_t {
        if (offset >= p.size()) return 0;
        return p[offset++];
    };
    auto readStr = [&](uint8_t len) -> std::string {
        if (offset + len > p.size()) return {};
        std::string s(reinterpret_cast<const char*>(p.data() + offset), len);
        offset += len;
        return s;
    };

    const uint32_t accepted_version = readU32();
    const bool     accepted         = (readU8() != 0);
    const uint8_t  type_len         = readU8();
    const std::string chassis_type  = readStr(type_len);
    const uint8_t  reason_len       = readU8();
    const std::string reject_reason = readStr(reason_len);

    if (accepted) {
        negotiated_version_ = accepted_version;
        chassis_type_       = chassis_type;
        state_              = State::CONNECTED;
    } else {
        reject_reason_ = reject_reason;
        state_         = State::FAILED;
    }
}

bool VersionNegotiator::isNegotiated() const {
    return state_ == State::CONNECTED;
}

uint32_t VersionNegotiator::getNegotiatedVersion() const {
    return negotiated_version_;
}

std::string VersionNegotiator::getChassisType() const {
    return chassis_type_;
}

std::string VersionNegotiator::getRejectReason() const {
    return reject_reason_;
}

VersionNegotiator::State VersionNegotiator::getState() const {
    return state_;
}

bool VersionNegotiator::checkTimeout() {
    if (state_ != State::HANDSHAKE_SENT) {
        return false;
    }

    const auto now     = std::chrono::steady_clock::now();
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                             now - last_send_time_).count();

    if (elapsed < TIMEOUT_MS) {
        return false;
    }

    // Timed out — retry or fail
    if (retries_ >= MAX_RETRIES) {
        state_ = State::FAILED;
        return true;
    }

    ++retries_;
    // Re-send and update timestamp
    auto payload = buildHandshakeReqPayload(protocol_version_);
    auto frame   = FrameCodec::encode(CmdId::HANDSHAKE_REQ, payload);
    transport_.send(frame.data(), frame.size());
    last_send_time_ = std::chrono::steady_clock::now();
    return false;
}

void VersionNegotiator::reset() {
    state_              = State::IDLE;
    negotiated_version_ = 0;
    chassis_type_.clear();
    retries_            = 0;
    last_send_time_     = {};
}

} // namespace chassis
