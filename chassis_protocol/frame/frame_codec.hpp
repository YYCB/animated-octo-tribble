#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <optional>
#include <vector>

// C++17 compatible span wrapper (std::span requires C++20)
namespace chassis {

template <typename T>
class Span {
public:
    Span() : data_(nullptr), size_(0) {}
    Span(T* data, std::size_t size) : data_(data), size_(size) {}
    Span(const std::vector<typename std::remove_const<T>::type>& v)
        : data_(v.data()), size_(v.size()) {}

    T* data() const { return data_; }
    std::size_t size() const { return size_; }
    bool empty() const { return size_ == 0; }
    T& operator[](std::size_t i) { return data_[i]; }
    const T& operator[](std::size_t i) const { return data_[i]; }
    T* begin() const { return data_; }
    T* end() const { return data_ + size_; }

private:
    T* data_;
    std::size_t size_;
};

} // namespace chassis

namespace chassis {

static constexpr uint16_t FRAME_MAGIC = 0xAA55;
static constexpr uint8_t  MAGIC_BYTE0 = 0xAA;
static constexpr uint8_t  MAGIC_BYTE1 = 0x55;
static constexpr uint8_t  FRAME_VERSION = 0x01;
static constexpr std::size_t HEADER_SIZE = 6; // MAGIC(2)+VERSION(1)+CMD_ID(1)+LEN(2)
static constexpr std::size_t TRAILER_SIZE = 2; // CRC16

enum class CmdId : uint8_t {
    HANDSHAKE_REQ   = 0x01,
    HANDSHAKE_RESP  = 0x02,
    VELOCITY_CMD    = 0x10,  // body-frame velocity → MCU does inverse kinematics
    LIGHT_CMD       = 0x11,
    EMERGENCY_STOP  = 0x12,
    WHEEL_CMD       = 0x13,  // per-wheel velocity (rad/s) → host did inverse kinematics
    CHASSIS_STATUS  = 0x20,
    ODOMETRY        = 0x21,
    BATTERY_INFO    = 0x22,
    ACK             = 0xFF,
};

#pragma pack(push, 1)
struct FrameHeader {
    uint8_t  magic0{MAGIC_BYTE0};
    uint8_t  magic1{MAGIC_BYTE1};
    uint8_t  version{FRAME_VERSION};
    uint8_t  cmd_id{0};
    uint8_t  len_lo{0};
    uint8_t  len_hi{0};

    uint16_t payloadLen() const {
        return static_cast<uint16_t>(len_lo) | (static_cast<uint16_t>(len_hi) << 8);
    }
    void setPayloadLen(uint16_t len) {
        len_lo = static_cast<uint8_t>(len & 0xFF);
        len_hi = static_cast<uint8_t>((len >> 8) & 0xFF);
    }
};
#pragma pack(pop)

static_assert(sizeof(FrameHeader) == HEADER_SIZE, "FrameHeader must be 6 bytes");

struct DecodedFrame {
    FrameHeader          header;
    std::vector<uint8_t> payload;

    CmdId cmdId() const { return static_cast<CmdId>(header.cmd_id); }
};

// ─── FrameCodec ──────────────────────────────────────────────────────────────

class FrameCodec {
public:
    // CRC16-CCITT over [VERSION, CMD_ID, LEN_LO, LEN_HI, PAYLOAD...]
    static uint16_t crc16(const uint8_t* data, std::size_t len);
    static uint16_t crc16(Span<const uint8_t> data) {
        return crc16(data.data(), data.size());
    }

    // Encode: returns full frame bytes
    static std::vector<uint8_t> encode(CmdId cmd_id,
                                       const uint8_t* payload,
                                       std::size_t payload_len,
                                       uint8_t version = FRAME_VERSION);
    static std::vector<uint8_t> encode(CmdId cmd_id,
                                       const std::vector<uint8_t>& payload,
                                       uint8_t version = FRAME_VERSION) {
        return encode(cmd_id, payload.data(), payload.size(), version);
    }

    // Decode: validates magic and CRC, returns DecodedFrame on success
    static std::optional<DecodedFrame> decode(const uint8_t* data, std::size_t len);
    static std::optional<DecodedFrame> decode(const std::vector<uint8_t>& data) {
        return decode(data.data(), data.size());
    }
};

// ─── StreamDecoder ───────────────────────────────────────────────────────────

class StreamDecoder {
public:
    StreamDecoder() { reset(); }

    void feed(uint8_t byte);
    void feed(const uint8_t* data, std::size_t len);
    void feed(Span<const uint8_t> data) { feed(data.data(), data.size()); }

    // Returns the next complete frame if one is ready, else nullopt.
    std::optional<DecodedFrame> poll();

    void reset();

private:
    enum class State {
        SYNC1,
        SYNC2,
        HEADER,   // reading remaining 4 bytes of header
        PAYLOAD,
        CRC,
        COMPLETE,
    };

    State                    state_{State::SYNC1};
    FrameHeader              current_header_{};
    std::vector<uint8_t>     payload_buf_;
    std::array<uint8_t, 2>   crc_buf_{};
    std::size_t              bytes_read_{0};
    std::vector<DecodedFrame> ready_frames_;

    void processState(uint8_t byte);
};

} // namespace chassis
