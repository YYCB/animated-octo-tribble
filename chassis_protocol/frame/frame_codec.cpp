#include "frame_codec.hpp"

#include <cstring>
#include <stdexcept>

namespace chassis {

// ─── CRC16-CCITT ─────────────────────────────────────────────────────────────
// Polynomial 0x1021, initial value 0xFFFF
// Covers: VERSION + CMD_ID + LEN_LO + LEN_HI + PAYLOAD

uint16_t FrameCodec::crc16(const uint8_t* data, std::size_t len) {
    uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            } else {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }
    return crc;
}

// ─── FrameCodec::encode ──────────────────────────────────────────────────────

std::vector<uint8_t> FrameCodec::encode(CmdId cmd_id,
                                         const uint8_t* payload,
                                         std::size_t payload_len,
                                         uint8_t version) {
    // Build the CRC region: [VERSION, CMD_ID, LEN_LO, LEN_HI, PAYLOAD...]
    const uint16_t len16 = static_cast<uint16_t>(payload_len);
    const uint8_t  len_lo = static_cast<uint8_t>(len16 & 0xFF);
    const uint8_t  len_hi = static_cast<uint8_t>((len16 >> 8) & 0xFF);

    std::vector<uint8_t> crc_region;
    crc_region.reserve(4 + payload_len);
    crc_region.push_back(version);
    crc_region.push_back(static_cast<uint8_t>(cmd_id));
    crc_region.push_back(len_lo);
    crc_region.push_back(len_hi);
    for (std::size_t i = 0; i < payload_len; ++i) {
        crc_region.push_back(payload[i]);
    }

    const uint16_t crc = crc16(crc_region.data(), crc_region.size());

    // Full frame: [MAGIC0, MAGIC1, VERSION, CMD_ID, LEN_LO, LEN_HI, PAYLOAD..., CRC_LO, CRC_HI]
    std::vector<uint8_t> frame;
    frame.reserve(HEADER_SIZE + payload_len + TRAILER_SIZE);
    frame.push_back(MAGIC_BYTE0);
    frame.push_back(MAGIC_BYTE1);
    frame.push_back(version);
    frame.push_back(static_cast<uint8_t>(cmd_id));
    frame.push_back(len_lo);
    frame.push_back(len_hi);
    for (std::size_t i = 0; i < payload_len; ++i) {
        frame.push_back(payload[i]);
    }
    frame.push_back(static_cast<uint8_t>(crc & 0xFF));
    frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));

    return frame;
}

// ─── FrameCodec::decode ──────────────────────────────────────────────────────

std::optional<DecodedFrame> FrameCodec::decode(const uint8_t* data, std::size_t len) {
    if (len < HEADER_SIZE + TRAILER_SIZE) {
        return std::nullopt;
    }

    // Validate magic
    if (data[0] != MAGIC_BYTE0 || data[1] != MAGIC_BYTE1) {
        return std::nullopt;
    }

    FrameHeader hdr;
    std::memcpy(&hdr, data, sizeof(FrameHeader));
    const uint16_t payload_len = hdr.payloadLen();

    if (len < HEADER_SIZE + payload_len + TRAILER_SIZE) {
        return std::nullopt;
    }

    // Build crc region for verification
    const uint8_t* crc_region_start = data + 2; // skip magic
    const std::size_t crc_region_len = 4 + payload_len; // VERSION+CMD_ID+LEN_LO+LEN_HI+PAYLOAD

    const uint16_t computed_crc = crc16(crc_region_start, crc_region_len);

    const uint8_t* crc_ptr = data + HEADER_SIZE + payload_len;
    const uint16_t received_crc =
        static_cast<uint16_t>(crc_ptr[0]) | (static_cast<uint16_t>(crc_ptr[1]) << 8);

    if (computed_crc != received_crc) {
        return std::nullopt;
    }

    DecodedFrame frame;
    frame.header = hdr;
    frame.payload.assign(data + HEADER_SIZE, data + HEADER_SIZE + payload_len);
    return frame;
}

// ─── StreamDecoder ───────────────────────────────────────────────────────────

void StreamDecoder::reset() {
    state_     = State::SYNC1;
    bytes_read_ = 0;
    payload_buf_.clear();
    crc_buf_   = {};
    current_header_ = FrameHeader{};
    ready_frames_.clear();
}

void StreamDecoder::feed(const uint8_t* data, std::size_t len) {
    for (std::size_t i = 0; i < len; ++i) {
        processState(data[i]);
    }
}

void StreamDecoder::feed(uint8_t byte) {
    processState(byte);
}

void StreamDecoder::processState(uint8_t byte) {
    switch (state_) {
        case State::SYNC1:
            if (byte == MAGIC_BYTE0) {
                current_header_.magic0 = byte;
                state_ = State::SYNC2;
            }
            break;

        case State::SYNC2:
            if (byte == MAGIC_BYTE1) {
                current_header_.magic1 = byte;
                bytes_read_ = 0;
                state_ = State::HEADER;
            } else if (byte == MAGIC_BYTE0) {
                // Stay in SYNC2 — could be 0xAA 0xAA 0x55 sequence
                current_header_.magic0 = byte;
            } else {
                state_ = State::SYNC1;
            }
            break;

        case State::HEADER: {
            // Remaining header bytes: VERSION, CMD_ID, LEN_LO, LEN_HI (4 bytes)
            uint8_t* hdr_bytes = reinterpret_cast<uint8_t*>(&current_header_);
            hdr_bytes[2 + bytes_read_] = byte;
            ++bytes_read_;
            if (bytes_read_ == 4) {
                const uint16_t plen = current_header_.payloadLen();
                payload_buf_.clear();
                payload_buf_.reserve(plen);
                bytes_read_ = 0;
                if (plen == 0) {
                    state_ = State::CRC;
                } else {
                    state_ = State::PAYLOAD;
                }
            }
            break;
        }

        case State::PAYLOAD:
            payload_buf_.push_back(byte);
            ++bytes_read_;
            if (bytes_read_ == current_header_.payloadLen()) {
                bytes_read_ = 0;
                state_ = State::CRC;
            }
            break;

        case State::CRC:
            crc_buf_[bytes_read_] = byte;
            ++bytes_read_;
            if (bytes_read_ == 2) {
                // Verify CRC
                std::vector<uint8_t> crc_region;
                crc_region.reserve(4 + payload_buf_.size());
                crc_region.push_back(current_header_.version);
                crc_region.push_back(current_header_.cmd_id);
                crc_region.push_back(current_header_.len_lo);
                crc_region.push_back(current_header_.len_hi);
                crc_region.insert(crc_region.end(), payload_buf_.begin(), payload_buf_.end());

                const uint16_t computed = FrameCodec::crc16(crc_region.data(), crc_region.size());
                const uint16_t received =
                    static_cast<uint16_t>(crc_buf_[0]) | (static_cast<uint16_t>(crc_buf_[1]) << 8);

                if (computed == received) {
                    DecodedFrame f;
                    f.header  = current_header_;
                    f.payload = payload_buf_;
                    ready_frames_.push_back(std::move(f));
                }
                // Whether CRC is good or bad, restart looking for next frame
                bytes_read_ = 0;
                payload_buf_.clear();
                state_ = State::SYNC1;
            }
            break;

        case State::COMPLETE:
            // Should not normally be in this state; just reset
            state_ = State::SYNC1;
            break;
    }
}

std::optional<DecodedFrame> StreamDecoder::poll() {
    if (ready_frames_.empty()) {
        return std::nullopt;
    }
    DecodedFrame f = std::move(ready_frames_.front());
    ready_frames_.erase(ready_frames_.begin());
    return f;
}

} // namespace chassis
