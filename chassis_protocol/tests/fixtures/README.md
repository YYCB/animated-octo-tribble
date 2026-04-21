# Fixtures Directory

This directory stores binary capture files used for regression testing of the
chassis communication protocol HAL.

## Binary Frame File Format

Each `.bin` file contains one or more raw chassis protocol frames in the
following layout:

```
[FRAME_LENGTH : 4 bytes, little-endian uint32]
[FRAME_DATA   : FRAME_LENGTH bytes of raw frame (including magic, header, payload, CRC)]
[FRAME_LENGTH : 4 bytes, little-endian uint32]
[FRAME_DATA   : FRAME_LENGTH bytes]
...
```

A single file may contain multiple consecutive entries. Each entry is
self-describing via its length prefix, so files can be concatenated or split
without any other metadata.

## How to Capture Frames

Wrap the active transport with a session recorder that taps the raw byte
stream and writes it to a `.bin` file:

```cpp
transport->setReceiveCallback([&](const uint8_t* data, std::size_t len) {
    // Write 4-byte LE length prefix
    uint32_t frame_len = static_cast<uint32_t>(len);
    fwrite(&frame_len, 4, 1, capture_file);
    fwrite(data, 1, len, capture_file);

    // Continue normal processing
    originalCallback(data, len);
});
```

A `SessionRecorder` utility class (not yet implemented) may be added to
`transport/` to automate this without modifying application code.

## How to Use Fixtures in Tests

```cpp
#include <fstream>
#include <vector>
#include "../frame/frame_codec.hpp"

std::vector<std::vector<uint8_t>> loadFrameFixture(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    std::vector<std::vector<uint8_t>> frames;
    while (f) {
        uint32_t len = 0;
        if (!f.read(reinterpret_cast<char*>(&len), 4)) break;
        std::vector<uint8_t> frame(len);
        if (!f.read(reinterpret_cast<char*>(frame.data()), len)) break;
        frames.push_back(std::move(frame));
    }
    return frames;
}

TEST(FixtureTest, DifferentialOdomRegression) {
    auto frames = loadFrameFixture(
        FIXTURE_DIR "/differential_odometry_2025-01-01.bin");

    chassis::StreamDecoder dec;
    for (const auto& raw : frames) {
        dec.feed(raw.data(), raw.size());
    }

    auto result = dec.poll();
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->cmdId(), chassis::CmdId::ODOMETRY);
    // Compare payload fields against known-good values
}
```

## Naming Convention

Files follow the pattern:

```
{chassis_model}_{cmd_name}_{YYYY-MM-DD}.bin
```

Examples:

| File | Description |
|------|-------------|
| `differential_odometry_2025-01-01.bin` | Odometry frames from a differential-drive robot |
| `mecanum_velocity_cmd_2025-01-01.bin`  | Outbound velocity commands to a mecanum platform |
| `generic_handshake_2025-01-01.bin`     | Full handshake exchange (REQ + RESP) |
| `differential_battery_info_2025-01-01.bin` | Battery status from a differential chassis |

## Regenerating Fixtures

After protocol changes that alter the wire format, regenerate all `.bin` files
by running the chassis node against the real hardware with capture mode enabled
and then promoting the resulting files to this directory.  Commit the new files
along with the protocol change so that CI can detect future regressions.
