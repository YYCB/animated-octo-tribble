# 04 版本协商（握手协议）

## 1. 概述

`VersionNegotiator`（`chassis/version_negotiator.hpp/.cpp`）在每次 Transport 连接成功后执行**一次**握手，确认：

- 双方协议版本兼容
- MCU 底盘类型（differential / mecanum）
- MCU 支持的 CMD_ID 集合

握手完成前，`ChassisHal` 收到的所有帧都交给 `VersionNegotiator::onFrameReceived()` 处理，**不进入**正常的状态/里程计分发流程。

---

## 2. 状态机

```
  ┌─────────────────────────────────────────────────────────┐
  │                         IDLE                            │
  │  sendHandshake() 调用                                    │
  └─────────────────────────────────────────────────────────┘
                              │
                              │ sendHandshake() 成功
                              ▼
  ┌─────────────────────────────────────────────────────────┐
  │                   HANDSHAKE_SENT                        │
  │  等待 HANDSHAKE_RESP，checkTimeout() 管理超时            │
  └─────────────────────────────────────────────────────────┘
           │                              │
           │ accepted=true                │ accepted=false
           ▼                              ▼
  ┌──────────────────┐         ┌─────────────────────────┐
  │    CONNECTED     │         │         FAILED          │
  │  isNegotiated()  │         │  getRejectReason()      │
  │  返回 true        │         │  返回拒绝原因            │
  └──────────────────┘         └─────────────────────────┘

  （HANDSHAKE_SENT 中 checkTimeout() 超时且 retries >= MAX_RETRIES 也会 → FAILED）
```

枚举定义：

```cpp
enum class State { IDLE, HANDSHAKE_SENT, CONNECTED, FAILED };
```

超时常量：

```cpp
static constexpr int  MAX_RETRIES = 3;
static constexpr long TIMEOUT_MS  = 2000;  // 每次握手等待时间
```

---

## 3. `HANDSHAKE_REQ` Payload 构建

函数：`buildHandshakeReqPayload(uint32_t protocol_version)`

```
偏移  大小    值（示例）           说明
0     4       01 00 00 00         protocol_version = 1（小端）
4     2       0A 00               num_cmds = 10
6     10      01 02 10 11         supported_cmds[]
              12 13 20 21
              22 FF
16    1       0E                  config_hash_len = 14
17    14      "chassis_hal_v1"   config_hash
```

发送完成后：
```cpp
state_          = State::HANDSHAKE_SENT;
last_send_time_ = std::chrono::steady_clock::now();
```

---

## 4. `HANDSHAKE_RESP` 解析

函数：`handleHandshakeResp(const DecodedFrame& frame)`

解析过程（顺序读取，小端）：

```cpp
uint32_t accepted_version = readU32();  // 协商版本号
bool     accepted         = readU8() != 0;
uint8_t  type_len         = readU8();
string   chassis_type     = readStr(type_len);
uint8_t  reason_len       = readU8();
string   reject_reason    = readStr(reason_len);
```

处理结果：

| `accepted` | 操作 |
|-----------|------|
| `true` | `negotiated_version_ = accepted_version`; `chassis_type_ = chassis_type`; `state_ = CONNECTED` |
| `false` | `reject_reason_ = reject_reason`; `state_ = FAILED` |

---

## 5. 超时与重试

`checkTimeout()` 由 `ChassisHal::connect()` 中的等待循环每 50ms 调用一次：

```cpp
bool VersionNegotiator::checkTimeout() {
    if (state_ != State::HANDSHAKE_SENT) return false;

    auto elapsed = elapsed_ms(last_send_time_);
    if (elapsed < TIMEOUT_MS) return false;   // 还没超时

    if (retries_ >= MAX_RETRIES) {
        state_ = State::FAILED;
        return true;                           // 通知 HAL 放弃
    }

    ++retries_;
    resendHandshake();                         // 重新发送 REQ
    last_send_time_ = now();
    return false;
}
```

最多重试 3 次，每次间隔 2000ms，共等待最多 **8 秒**。

---

## 6. 与 `ChassisHal` 的集成

### 6.1 对象持有关系

```
ChassisHal
  ├─ std::unique_ptr<ITransport> transport_
  └─ VersionNegotiator negotiator_  ← 持有 transport_ 的引用
```

`VersionNegotiator` 构造时接收 `ITransport&`，其 `sendHandshake()` 直接调用 `transport_.send()`。

### 6.2 帧分流逻辑

```cpp
// ChassisHal::onFrameDecoded()
void ChassisHal::onFrameDecoded(const DecodedFrame& frame) {
    if (!negotiator_.isNegotiated()) {
        negotiator_.onFrameReceived(frame);  // 握手阶段：交给协商器
        return;
    }
    // 正常阶段：按 CMD_ID 分发
    switch (frame.cmdId()) {
        case CmdId::CHASSIS_STATUS: dispatchStatusFrame(frame);   break;
        case CmdId::ODOMETRY:       dispatchOdometryFrame(frame); break;
        case CmdId::BATTERY_INFO:   dispatchBatteryFrame(frame);  break;
        case CmdId::ACK:            /* 忽略 */                    break;
        default:                    break;
    }
}
```

### 6.3 `connect()` 完整流程

```cpp
bool ChassisHal::connect() {
    if (connected_) return true;

    if (!transport_->connect()) return false;

    negotiator_.reset();
    if (!negotiator_.sendHandshake()) {
        transport_->disconnect();
        return false;
    }

    // 等待最多 6 秒
    auto deadline = now() + 6s;
    while (now() < deadline) {
        if (negotiator_.isNegotiated()) break;
        if (negotiator_.checkTimeout()) {   // 超时失败
            transport_->disconnect();
            return false;
        }
        sleep(50ms);
    }

    if (!negotiator_.isNegotiated()) {
        transport_->disconnect();
        return false;
    }

    connected_  = true;
    running_    = true;
    reconnect_thread_ = thread(&ChassisHal::reconnectLoop, this);

    // 从协商器提取设备信息
    info_.protocol_version = negotiator_.getNegotiatedVersion();
    info_.model            = negotiator_.getChassisType();
    return true;
}
```

---

## 7. `CONFIG_HASH` 的作用

当前值固定为 `"chassis_hal_v1"`。MCU 可用此字段：

- 验证上位机 HAL 版本（拒绝旧版 HAL 连接）
- 区分配置文件版本（若协议参数集发生变化则更换哈希）

扩展建议：将 hash 改为 HAL 编译时 git commit SHA 的前8位，可自动追踪版本一致性。

---

## 8. `reset()` 使用时机

重连循环在重建 transport 连接后调用：

```cpp
// ChassisHal::reconnectLoop()
negotiator_.reset();
stream_decoder_.reset();   // 同时清空未完成的帧缓冲
if (!negotiator_.sendHandshake()) { ... }
```

`reset()` 将状态机归回 `IDLE`，清除 `negotiated_version_`、`chassis_type_`、`reject_reason_`、`retries_`。
