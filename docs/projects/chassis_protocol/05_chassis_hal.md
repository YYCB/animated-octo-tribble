# 05 ChassisHal 核心实现

## 1. 类关系

```
IChassisController  (纯虚接口)
        △
        │
   ChassisHal        ← 本文档重点
        △
        │
 ┌──────┴───────┐
 │              │
DifferentialController   MecanumController
```

`ChassisHal` 聚合：
- `std::unique_ptr<ITransport> transport_` — 传输层
- `VersionNegotiator negotiator_` — 握手（持有 `transport_` 引用）
- `StreamDecoder stream_decoder_` — 流式解帧

---

## 2. 成员变量详解

```cpp
class ChassisHal : public IChassisController {
protected:
    // 传输与协商
    std::unique_ptr<ITransport> transport_;
    uint32_t                    protocol_version_;
    uint32_t                    seq_{0};           // 自增帧序号
    VersionNegotiator           negotiator_;
    StreamDecoder               stream_decoder_;

    // 最新状态缓存（由接收线程写，主线程/ROS2线程读）
    ChassisStatus current_status_;    // 受 status_mutex_  保护
    OdometryData  current_odom_;      // 受 odom_mutex_    保护
    BatteryData   current_battery_;   // 受 battery_mutex_ 保护
    ChassisInfo   info_;              // 受 status_mutex_  保护

    // 互斥锁
    mutable std::mutex status_mutex_;
    mutable std::mutex odom_mutex_;
    mutable std::mutex battery_mutex_;
    mutable std::mutex cb_mutex_;     // 保护回调函数指针

    // 异步回调
    std::function<void(const ChassisStatus&)> status_cb_;
    std::function<void(const OdometryData&)>  odom_cb_;
    std::function<void(const BatteryData&)>   battery_cb_;

    // 线程控制
    std::atomic<bool> connected_{false};
    std::atomic<bool> running_{false};
    std::thread       reconnect_thread_;
};
```

---

## 3. 构造与析构

```cpp
ChassisHal::ChassisHal(std::unique_ptr<ITransport> transport,
                        uint32_t protocol_version)
    : transport_(std::move(transport))
    , protocol_version_(protocol_version)
    , negotiator_(*transport_, protocol_version)  // 注意：引用已 move 后的 transport_
{
    // 在 transport 构造完成后立即注册接收回调
    transport_->setReceiveCallback(
        [this](const uint8_t* data, std::size_t len) {
            onDataReceived(data, len);
        });
}

ChassisHal::~ChassisHal() {
    disconnect();   // 确保重连线程退出、transport 断开
}
```

**注意**：`negotiator_` 使用 `*transport_` 的引用，必须在 `transport_` 被 move 进成员之后构造，因此通过成员初始化列表顺序保证（`transport_` 先于 `negotiator_`）。

---

## 4. 序号管理

```cpp
uint32_t ChassisHal::nextSeq() {
    return seq_++;   // 无锁：仅在发送路径（可能来自 ROS2 主线程）调用
}
```

`seq_` 是 `uint32_t` 非原子类型。若多个线程并发调用 `sendVelocity()` / `sendWheelCmd()` 等，需在调用方自行串行化或将 `seq_` 改为 `std::atomic<uint32_t>`。当前架构中 `cmd_vel` 回调由 ROS2 单线程执行器调用，实际为单线程访问。

---

## 5. 下行命令编码（Payload 构建辅助函数）

所有辅助函数均使用小端序字节写入器：

```cpp
static void appendU8 (buf, uint8_t  v)
static void appendU16LE(buf, uint16_t v)
static void appendU32LE(buf, uint32_t v)
static void appendU64LE(buf, uint64_t v)
static void appendDouble(buf, double  v)   // memcpy 到 uint64_t 后 appendU64LE
```

### 5.1 `buildVelocityPayload`

```cpp
// 格式：seq(4) + vx(8) + vy(8) + omega(8) + timestamp_us(8) = 36 B
std::vector<uint8_t> buf; buf.reserve(36);
appendU32LE(buf, nextSeq());
appendDouble(buf, vx); appendDouble(buf, vy); appendDouble(buf, omega);
appendU64LE(buf, nowUs());
```

### 5.2 `buildWheelPayload`

```cpp
// 格式：seq(4) + timestamp_us(8) + n_wheels(1) + wheel_rads[N](8*N)
const uint8_t n = min(wheel_rpms_rads.size(), 255);
buf.reserve(4 + 8 + 1 + 8 * n);
appendU32LE(buf, nextSeq());
appendU64LE(buf, nowUs());
appendU8(buf, n);
for (uint8_t i = 0; i < n; ++i) appendDouble(buf, wheel_rpms_rads[i]);
```

### 5.3 `buildLightPayload`

```cpp
// 格式：mode(1) + r(1) + g(1) + b(1) + blink_hz(8) = 12 B
appendU8(buf, static_cast<uint8_t>(mode));
appendU8(buf, r); appendU8(buf, g); appendU8(buf, b);
appendDouble(buf, hz);
```

### 5.4 `buildStopPayload`

```cpp
// 格式：reason_len(2) + reason(N)
uint16_t len = reason.size();
appendU16LE(buf, len);
for (char c : reason) buf.push_back(c);
```

### 5.5 `nowUs()` 时间戳

```cpp
static uint64_t nowUs() {
    return duration_cast<microseconds>(
        steady_clock::now().time_since_epoch()).count();
}
```

使用 `steady_clock` 避免系统时间跳变。

---

## 6. 上行帧解析

### 6.1 `dispatchStatusFrame`

解析 `CHASSIS_STATUS (0x20)` payload：

```cpp
const uint32_t seq    = readU32LE(p, off);  // 忽略（仅日志用途）
st.timestamp_us = readU64LE(p, off);
st.error_code   = readU32LE(p, off);
uint16_t msg_len = readU16LE(p, off);
st.error_msg.assign(p.data() + off, msg_len);
// bit15 = 急停激活标志
st.is_estop = (st.error_code & 0x8000U) != 0;
```

写入 `current_status_`（加 `status_mutex_`），然后取出 `status_cb_`（加 `cb_mutex_`）并调用。

### 6.2 `dispatchOdometryFrame`

解析 `ODOMETRY (0x21)` payload（668 B 固定大小）：

```
seq(4) + timestamp_us(8) + x/y/z(24) + q{x,y,z,w}(32)
+ vx/vy/omega(24) + cov_pose(288) + cov_twist(288)
```

写入 `current_odom_`（加 `odom_mutex_`），调用 `odom_cb_`。

### 6.3 `dispatchBatteryFrame`

解析 `BATTERY_INFO (0x22)` payload（可变长）：

```
voltage(8) + current(8) + percentage(8) + temperature(8)
+ is_charging(1) + num_cells(1) + cell_voltages(8*N)
```

写入 `current_battery_`（加 `battery_mutex_`），调用 `battery_cb_`。

### 6.4 回调调用模式（防止在锁内调用用户回调）

```cpp
// 取出 callback 指针（持锁），在锁外调用
std::function<void(const ChassisStatus&)> cb;
{
    std::lock_guard<std::mutex> lk(cb_mutex_);
    cb = status_cb_;
}
if (cb) cb(st);   // 锁外调用，避免死锁
```

---

## 7. 断线重连循环

`reconnectLoop()` 运行在独立线程，每 200ms 检查一次：

```
while running_:
    sleep(200ms)
    if transport_ 已连接 AND negotiator_ 已协商:
        backoff_ms = 500
        continue

    // 连接丢失
    connected_ = false

    if transport_ 未连接:
        sleep(backoff_ms)
        backoff_ms = min(backoff_ms * 2, 30000)
        transport_->connect()  → 失败则 continue

    // transport 已连，重跑握手
    negotiator_.reset()
    stream_decoder_.reset()
    negotiator_.sendHandshake()

    等待最多 6s:
        if negotiator_.isNegotiated(): break
        if negotiator_.checkTimeout(): break

    if 协商成功:
        connected_ = true
        backoff_ms = 500
    else:
        transport_->disconnect()
```

**退避策略**：500ms → 1000ms → 2000ms → … → 30000ms（最大 30 秒）。

---

## 8. `disconnect()` 流程

```cpp
void ChassisHal::disconnect() {
    running_.store(false);     // 通知重连线程退出
    connected_.store(false);
    transport_->disconnect();  // 关闭 fd，解除接收线程的 select() 阻塞
    if (reconnect_thread_.joinable())
        reconnect_thread_.join();
}
```

顺序很重要：先设 `running_=false`，再关 transport（唤醒接收线程），最后 join 重连线程。

---

## 9. `getInfo()` 的 `info_` 填充时机

`info_` 在 `connect()` 成功后（握手完成后）填充：

```cpp
std::lock_guard<std::mutex> lk(status_mutex_);
info_.protocol_version = negotiator_.getNegotiatedVersion();
info_.model            = negotiator_.getChassisType();  // 来自 HANDSHAKE_RESP
```

`vendor` 和 `firmware_version` 由子类（`DifferentialController::getInfo()`）覆盖填充。

---

## 10. `IChassisController` getter 一览

| 方法 | 所用锁 | 说明 |
|------|--------|------|
| `getStatus()` | `status_mutex_` | 返回 `current_status_` 副本 |
| `getInfo()` | `status_mutex_` | 返回 `info_` 副本（子类可覆盖） |
| `getOdometry()` | `odom_mutex_` | 返回 `current_odom_` 副本 |
| `getBatteryInfo()` | `battery_mutex_` | 返回 `current_battery_` 副本 |
| `setXxxCallback()` | `cb_mutex_` | 存储回调函数指针 |
