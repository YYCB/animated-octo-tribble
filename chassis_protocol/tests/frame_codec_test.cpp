#include <gtest/gtest.h>
#include "../frame/frame_codec.hpp"

using namespace chassis;

// ─── 1. CRC16 known-value test ────────────────────────────────────────────────
// CRC16-CCITT (poly=0x1021, init=0xFFFF) of "123456789" = 0x29B1
TEST(FrameCodecTest, Crc16KnownValue) {
    const char* msg = "123456789";
    const uint16_t expected = 0x29B1;
    const uint16_t computed = FrameCodec::crc16(
        reinterpret_cast<const uint8_t*>(msg), 9);
    EXPECT_EQ(computed, expected);
}

// ─── 2. Encode + decode roundtrip ─────────────────────────────────────────────
TEST(FrameCodecTest, RoundtripVelocityCmd) {
    const std::vector<uint8_t> payload = {0x01, 0x02, 0x03, 0x04,
                                          0x05, 0x06, 0x07, 0x08};
    const auto frame = FrameCodec::encode(CmdId::VELOCITY_CMD, payload);
    const auto result = FrameCodec::decode(frame.data(), frame.size());

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->cmdId(), CmdId::VELOCITY_CMD);
    EXPECT_EQ(result->payload, payload);
    EXPECT_EQ(result->header.version, FRAME_VERSION);
}

TEST(FrameCodecTest, RoundtripChassisStatus) {
    const std::vector<uint8_t> payload(20, 0xAB);
    const auto frame = FrameCodec::encode(CmdId::CHASSIS_STATUS, payload);
    const auto result = FrameCodec::decode(frame.data(), frame.size());

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->cmdId(), CmdId::CHASSIS_STATUS);
    EXPECT_EQ(result->payload, payload);
}

TEST(FrameCodecTest, RoundtripHandshakeReq) {
    const std::vector<uint8_t> payload = {0x01, 0x00, 0x00, 0x00,
                                          0x02, 0x00, 0x10, 0x11};
    const auto frame = FrameCodec::encode(CmdId::HANDSHAKE_REQ, payload);
    const auto result = FrameCodec::decode(frame.data(), frame.size());

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->cmdId(), CmdId::HANDSHAKE_REQ);
    EXPECT_EQ(result->payload, payload);
}

TEST(FrameCodecTest, RoundtripEmptyPayload) {
    const std::vector<uint8_t> payload;
    const auto frame = FrameCodec::encode(CmdId::EMERGENCY_STOP, payload);
    EXPECT_EQ(frame.size(), HEADER_SIZE + TRAILER_SIZE);

    const auto result = FrameCodec::decode(frame.data(), frame.size());
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->cmdId(), CmdId::EMERGENCY_STOP);
    EXPECT_TRUE(result->payload.empty());
}

// ─── 3. decode returns nullopt for wrong magic ────────────────────────────────
TEST(FrameCodecTest, DecodeWrongMagic) {
    std::vector<uint8_t> payload = {0x01, 0x02};
    auto frame = FrameCodec::encode(CmdId::VELOCITY_CMD, payload);
    frame[0] = 0x00; // corrupt first magic byte
    EXPECT_FALSE(FrameCodec::decode(frame.data(), frame.size()).has_value());

    frame[0] = MAGIC_BYTE0;
    frame[1] = 0x00; // corrupt second magic byte
    EXPECT_FALSE(FrameCodec::decode(frame.data(), frame.size()).has_value());
}

// ─── 4. decode returns nullopt for bad CRC ────────────────────────────────────
TEST(FrameCodecTest, DecodeBadCrc) {
    const std::vector<uint8_t> payload = {0xDE, 0xAD, 0xBE, 0xEF};
    auto frame = FrameCodec::encode(CmdId::VELOCITY_CMD, payload);
    // Flip one CRC byte
    frame[frame.size() - 1] ^= 0xFF;
    EXPECT_FALSE(FrameCodec::decode(frame.data(), frame.size()).has_value());
}

// ─── 5. StreamDecoder: single frame byte-by-byte ─────────────────────────────
TEST(StreamDecoderTest, SingleFrameByByte) {
    const std::vector<uint8_t> payload = {0x10, 0x20, 0x30};
    const auto frame = FrameCodec::encode(CmdId::CHASSIS_STATUS, payload);

    StreamDecoder dec;
    for (uint8_t byte : frame) {
        dec.feed(byte);
    }

    const auto result = dec.poll();
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->cmdId(), CmdId::CHASSIS_STATUS);
    EXPECT_EQ(result->payload, payload);

    // No more frames
    EXPECT_FALSE(dec.poll().has_value());
}

// ─── 6. StreamDecoder: two frames concatenated ───────────────────────────────
TEST(StreamDecoderTest, TwoFramesConcatenated) {
    const std::vector<uint8_t> pay1 = {0xAA, 0xBB};
    const std::vector<uint8_t> pay2 = {0x11, 0x22, 0x33, 0x44};

    auto frame1 = FrameCodec::encode(CmdId::VELOCITY_CMD,   pay1);
    auto frame2 = FrameCodec::encode(CmdId::CHASSIS_STATUS, pay2);

    // Concatenate
    std::vector<uint8_t> combined;
    combined.insert(combined.end(), frame1.begin(), frame1.end());
    combined.insert(combined.end(), frame2.begin(), frame2.end());

    StreamDecoder dec;
    dec.feed(combined.data(), combined.size());

    auto r1 = dec.poll();
    ASSERT_TRUE(r1.has_value());
    EXPECT_EQ(r1->cmdId(), CmdId::VELOCITY_CMD);
    EXPECT_EQ(r1->payload, pay1);

    auto r2 = dec.poll();
    ASSERT_TRUE(r2.has_value());
    EXPECT_EQ(r2->cmdId(), CmdId::CHASSIS_STATUS);
    EXPECT_EQ(r2->payload, pay2);

    EXPECT_FALSE(dec.poll().has_value());
}

// ─── 7. StreamDecoder: corrupt sync byte before valid frame ──────────────────
TEST(StreamDecoderTest, CorruptBytesBeforeFrame) {
    const std::vector<uint8_t> payload = {0x01, 0x02};
    const auto frame = FrameCodec::encode(CmdId::HANDSHAKE_REQ, payload);

    // Prepend garbage bytes
    std::vector<uint8_t> data = {0x00, 0xFF, 0x12, 0x34, 0xAA, 0x00, 0xBB};
    data.insert(data.end(), frame.begin(), frame.end());

    StreamDecoder dec;
    dec.feed(data.data(), data.size());

    const auto result = dec.poll();
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->cmdId(), CmdId::HANDSHAKE_REQ);
    EXPECT_EQ(result->payload, payload);
}

// ─── 8. StreamDecoder: partial frame across two feed() calls ─────────────────
TEST(StreamDecoderTest, PartialFrameAcrossTwoFeeds) {
    const std::vector<uint8_t> payload = {0xCA, 0xFE, 0xBA, 0xBE};
    const auto frame = FrameCodec::encode(CmdId::ODOMETRY, payload);

    const std::size_t split = frame.size() / 2;

    StreamDecoder dec;
    dec.feed(frame.data(), split);
    EXPECT_FALSE(dec.poll().has_value()); // incomplete

    dec.feed(frame.data() + split, frame.size() - split);
    const auto result = dec.poll();
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->cmdId(), CmdId::ODOMETRY);
    EXPECT_EQ(result->payload, payload);
}

// ─── 9. StreamDecoder: reset clears state ────────────────────────────────────
TEST(StreamDecoderTest, ResetClearsState) {
    const std::vector<uint8_t> payload = {0x01};
    const auto frame = FrameCodec::encode(CmdId::HANDSHAKE_REQ, payload);

    StreamDecoder dec;
    // Feed half then reset
    dec.feed(frame.data(), frame.size() / 2);
    dec.reset();
    EXPECT_FALSE(dec.poll().has_value());

    // Feed the complete frame after reset
    dec.feed(frame.data(), frame.size());
    const auto result = dec.poll();
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->cmdId(), CmdId::HANDSHAKE_REQ);
}

// ─── 10. Encode: frame length field matches payload ───────────────────────────
TEST(FrameCodecTest, LengthFieldCorrect) {
    const std::vector<uint8_t> payload(256, 0x5A);
    const auto frame = FrameCodec::encode(CmdId::ODOMETRY, payload);

    // LEN field is bytes [4..5] of frame (after MAGIC0, MAGIC1, VERSION, CMD_ID)
    const uint16_t len_lo = frame[4];
    const uint16_t len_hi = frame[5];
    const uint16_t len    = len_lo | (len_hi << 8);
    EXPECT_EQ(len, 256u);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
