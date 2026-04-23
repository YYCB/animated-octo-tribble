# 02 帧格式与编解码

## 1. 帧结构总览

所有字节均**小端序**（Little-Endian），多字节整数低字节在前。

```
┌────────┬────────┬─────────┬──────────┬─────────┬─────────┬────────────┬──────────┐
│ MAGIC0 │ MAGIC1 │ VERSION │  CMD_ID  │  LEN_LO │  LEN_HI │  PAYLOAD…  │ CRC_LO/HI│
│  0xAA  │  0x55  │  uint8  │  uint8   │  uint8  │  uint8  │  N bytes   │  uint16  │
└────────┴────────┴─────────┴──────────┴─────────┴─────────┴────────────┴──────────┘
  Byte 0   Byte 1   Byte 2    Byte 3    Byte 4    Byte 5    Byte 6..N+5   N+6..N+7
```

| 字段 | 大小（字节） | 值/说明 |
|------|------------|--------|
| MAGIC0 | 1 | 固定 `0xAA` |
| MAGIC1 | 1 | 固定 `0x55` |
| VERSION | 1 | 当前 `0x01`，用于向前兼容 |
| CMD_ID | 1 | 见 §2 CMD_ID 表 |
| LEN_LO | 1 | Payload 长度低字节 |
| LEN_HI | 1 | Payload 长度高字节（`LEN = LEN_LO | (LEN_HI << 8)`） |
| PAYLOAD | 0–65535 B | 命令或响应数据 |
| CRC_LO | 1 | CRC16 低字节 |
| CRC_HI | 1 | CRC16 高字节 |

常量定义（`frame/frame_codec.hpp`）：

```cpp
static constexpr uint16_t FRAME_MAGIC   = 0xAA55;
static constexpr uint8_t  MAGIC_BYTE0   = 0xAA;
static constexpr uint8_t  MAGIC_BYTE1   = 0x55;
static constexpr uint8_t  FRAME_VERSION = 0x01;
static constexpr std::size_t HEADER_SIZE  = 6;   // MAGIC(2)+VERSION(1)+CMD_ID(1)+LEN(2)
static constexpr std::size_t TRAILER_SIZE = 2;   // CRC16
```

最小帧大小（空 Payload）= `HEADER_SIZE + TRAILER_SIZE` = **8 字节**。

---

## 2. CMD_ID 表

```cpp
enum class CmdId : uint8_t {
    HANDSHAKE_REQ   = 0x01,  // 上位机 → MCU：发起握手
    HANDSHAKE_RESP  = 0x02,  // MCU → 上位机：握手响应
    VELOCITY_CMD    = 0x10,  // 上位机 → MCU：体坐标速度（MCU 做逆运动学）
    LIGHT_CMD       = 0x11,  // 上位机 → MCU：灯光控制
    EMERGENCY_STOP  = 0x12,  // 上位机 → MCU：急停
    WHEEL_CMD       = 0x13,  // 上位机 → MCU：各轮角速度（上位机做逆运动学）
    CHASSIS_STATUS  = 0x20,  // MCU → 上位机：底盘状态
    ODOMETRY        = 0x21,  // MCU → 上位机：里程计
    BATTERY_INFO    = 0x22,  // MCU → 上位机：电池信息
    ACK             = 0xFF,  // 双向：通用确认
};
```

---

## 3. 各帧 Payload 详细布局

### 3.1 VELOCITY_CMD (0x10) — 36 字节

```
偏移  大小  类型        字段
0     4     uint32_t   seq（帧序号，单调递增）
4     8     double     vx（m/s，体坐标前向）
12    8     double     vy（m/s，体坐标侧向）
20    8     double     omega（rad/s，偏航角速度）
28    8     uint64_t   timestamp_us（上位机单调时钟，微秒）
```

构建代码（`chassis_hal.cpp`）：

```cpp
std::vector<uint8_t> ChassisHal::buildVelocityPayload(double vx, double vy, double omega) {
    std::vector<uint8_t> buf;
    buf.reserve(36);
    appendU32LE(buf, nextSeq());      // 4 B
    appendDouble(buf, vx);            // 8 B
    appendDouble(buf, vy);            // 8 B
    appendDouble(buf, omega);         // 8 B
    appendU64LE(buf, nowUs());        // 8 B
    return buf;                       // 合计 36 B
}
```

### 3.2 WHEEL_CMD (0x13) — 13 + 8×N 字节

```
偏移      大小    类型        字段
0         4       uint32_t   seq
4         8       uint64_t   timestamp_us
12        1       uint8_t    n_wheels（车轮数量，≤255）
13        8×N     double[]   wheel_rads[0..N-1]（rad/s，各轮目标角速度）
```

轮序约定：

| 底盘 | 轮序 |
|------|------|
| 差速（2轮） | [0] 左轮，[1] 右轮 |
| 麦克纳姆（4轮） | [0] FL，[1] FR，[2] RL，[3] RR |

正值 = 正向旋转（与前进方向一致）。

### 3.3 LIGHT_CMD (0x11) — 12 字节

```
偏移  大小  类型    字段
0     1     uint8  mode (0=OFF, 1=SOLID, 2=BLINK, 3=BREATH)
1     1     uint8  r
2     1     uint8  g
3     1     uint8  b
4     8     double blink_hz（仅 BLINK/BREATH 有效）
```

### 3.4 EMERGENCY_STOP (0x12) — 2+N 字节

```
偏移  大小    类型    字段
0     2       uint16 reason_len（小端）
2     N       char[] reason（UTF-8，不含 NUL）
```

### 3.5 HANDSHAKE_REQ (0x01)

```
偏移          大小      类型      字段
0             4         uint32   protocol_version
4             2         uint16   num_cmds（支持的 CMD_ID 数量）
6             N         uint8[]  supported_cmds[num_cmds]
6+N           1         uint8    config_hash_len
7+N           M         char[]   config_hash（ASCII，无 NUL）
```

当前支持的 CMD_ID 列表（`version_negotiator.cpp`）：
```
0x01, 0x02, 0x10, 0x11, 0x12, 0x13, 0x20, 0x21, 0x22, 0xFF
```
config_hash 固定为 `"chassis_hal_v1"`。

### 3.6 HANDSHAKE_RESP (0x02)

```
偏移      大小    类型    字段
0         4       uint32 accepted_version
4         1       uint8  accepted（0=拒绝, 1=接受）
5         1       uint8  chassis_type_len
6         N       char[] chassis_type（如 "differential", "mecanum"）
6+N       1       uint8  reject_reason_len
7+N       M       char[] reject_reason（accepted=0 时有效）
```

### 3.7 CHASSIS_STATUS (0x20)

```
偏移  大小    类型    字段
0     4       uint32 seq
4     8       uint64 timestamp_us
12    4       uint32 error_code（bit15=1 表示急停激活）
16    2       uint16 error_msg_len
18    N       char[] error_msg
```

### 3.8 ODOMETRY (0x21) — 668 字节（固定）

```
偏移  大小  字段
0     4     seq
4     8     timestamp_us
12    8     x (m)
20    8     y (m)
28    8     z (m)
36    8     qx
44    8     qy
52    8     qz
60    8     qw
68    8     vx (m/s)
76    8     vy (m/s)
84    8     omega (rad/s)
92    288   cov_pose[36]   (36×double)
380   288   cov_twist[36]  (36×double)
```

### 3.9 BATTERY_INFO (0x22) — 可变长

```
偏移  大小      字段
0     8         voltage (V)
8     8         current (A，正值=放电)
16    8         percentage (0–100)
24    8         temperature (℃)
32    1         is_charging (0/1)
33    1         num_cells
34    8×N       cell_voltages[num_cells]
```

---

## 4. CRC16-CCITT 算法

**参数**：多项式 `0x1021`，初始值 `0xFFFF`，无输入/输出反转。

**校验覆盖范围**：`VERSION + CMD_ID + LEN_LO + LEN_HI + PAYLOAD`（**不含** MAGIC 两字节）。

```cpp
// frame/frame_codec.cpp
uint16_t FrameCodec::crc16(const uint8_t* data, std::size_t len) {
    uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000)
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            else
                crc = static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}
```

已知值验证：`crc16("123456789", 9) == 0x29B1`（标准 CCITT 测试向量）。

CRC 以小端序追加到帧尾：`frame[N-2] = crc & 0xFF`，`frame[N-1] = (crc >> 8) & 0xFF`。

---

## 5. `FrameCodec`：编码与解码

### 5.1 `encode()`

```cpp
// 签名
static std::vector<uint8_t> encode(CmdId cmd_id,
                                   const uint8_t* payload,
                                   std::size_t payload_len,
                                   uint8_t version = FRAME_VERSION);
```

编码步骤：
1. 构建 CRC 覆盖区：`[VERSION, CMD_ID, LEN_LO, LEN_HI, PAYLOAD…]`
2. 计算 CRC16
3. 组装最终帧：`[MAGIC0, MAGIC1] + CRC区 + [CRC_LO, CRC_HI]`

### 5.2 `decode()`

```cpp
static std::optional<DecodedFrame> decode(const uint8_t* data, std::size_t len);
```

解码步骤（任一失败返回 `nullopt`）：
1. 检查 `len >= 8`
2. 验证 `data[0] == 0xAA && data[1] == 0x55`
3. 解析 `payload_len = LEN_LO | (LEN_HI << 8)`
4. 检查 `len >= 6 + payload_len + 2`
5. 重算 CRC，比对帧尾两字节
6. 返回 `DecodedFrame{header, payload}`

### 5.3 返回结构

```cpp
struct DecodedFrame {
    FrameHeader          header;   // 6字节头，含 version、cmd_id、len
    std::vector<uint8_t> payload;
    CmdId cmdId() const { return static_cast<CmdId>(header.cmd_id); }
};
```

---

## 6. `StreamDecoder`：流式解帧状态机

用于流式传输（TCP/RS485）中把连续字节流切分为完整帧。

### 6.1 状态转移图

```
                      byte == 0xAA
         ┌──────────────────────────────────────────────┐
         │                                              ▼
  ┌──────────┐  0xAA   ┌──────────┐  0x55   ┌──────────────┐
  │  SYNC1   │────────►│  SYNC2   │────────►│    HEADER    │
  └──────────┘         └──────────┘         │ (读4字节版本  │
       ▲                    │                │  CMD_ID LEN) │
       │   其他字节           │ 其他字节       └──────┬───────┘
       │◄───────────────────┘                       │ bytes_read==4
       │                                            │
       │                          payload_len > 0   ▼
       │                            ┌─────────────────────┐
       │                            │      PAYLOAD        │
       │                            │ (读 payload_len 字节)│
       │                            └──────────┬──────────┘
       │                                       │ bytes_read==payload_len
       │                                       ▼ （或 payload_len==0 直接跳）
       │                            ┌─────────────────────┐
       │                            │        CRC          │
       │                            │   (读2字节 CRC)      │
       │                            └──────────┬──────────┘
       │                                       │ bytes_read==2
       │               CRC 不匹配              ▼
       └───────────────────────────── 校验 CRC
                                              │ CRC 匹配
                                              ▼
                                      push DecodedFrame
                                      → ready_frames_
                                      → 回到 SYNC1
```

特殊情况：SYNC2 状态下若收到 0xAA，停留在 SYNC2（支持 `0xAA 0xAA 0x55` 序列）。

### 6.2 关键成员

```cpp
class StreamDecoder {
    enum class State { SYNC1, SYNC2, HEADER, PAYLOAD, CRC, COMPLETE };

    State                     state_{State::SYNC1};
    FrameHeader               current_header_{};
    std::vector<uint8_t>      payload_buf_;
    std::array<uint8_t, 2>    crc_buf_{};
    std::size_t               bytes_read_{0};
    std::vector<DecodedFrame> ready_frames_;   // 已完成帧队列
};
```

### 6.3 使用方式

```cpp
StreamDecoder dec;

// 接收线程中
dec.feed(data_ptr, len);         // 可以是任意字节块

// 主循环或帧处理函数中
while (auto frame = dec.poll()) {
    onFrameDecoded(*frame);
}
```

`poll()` 以 FIFO 顺序返回帧，每次调用弹出一个；返回 `nullopt` 表示当前队列空。

`reset()` 清空全部状态（用于重连后刷新流）。
