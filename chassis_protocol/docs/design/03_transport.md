# 03 传输层设计

## 1. `ITransport` 抽象接口

文件：`transport/itransport.hpp`

```cpp
class ITransport {
public:
    virtual ~ITransport() = default;

    // 发送原始字节（阻塞，直到全部写出或出错）
    virtual bool send(const uint8_t* data, std::size_t len) = 0;
    bool send(const std::vector<uint8_t>& data);   // 便利重载
    bool send(Span<const uint8_t> data);            // 便利重载

    // 注册接收回调（接收线程调用，在接收线程上下文中执行）
    virtual void setReceiveCallback(
        std::function<void(const uint8_t*, std::size_t)> cb) = 0;

    virtual bool        connect()            = 0;
    virtual void        disconnect()         = 0;
    virtual bool        isConnected() const  = 0;
    virtual std::string getLastError() const = 0;

    // 工厂方法（itransport.cpp）
    static std::unique_ptr<ITransport> create(
        TransportType type, const TransportConfig& config);
};
```

### `TransportConfig` 结构体

```cpp
struct TransportConfig {
    std::string host;               // TCP/UDP 远端主机名或 IP
    uint16_t    port{0};            // TCP/UDP 端口号
    std::string device_path;        // RS485 设备路径，如 "/dev/ttyUSB0"
    uint32_t    baud_rate{115200};  // 串口波特率
    uint32_t    timeout_ms{1000};   // 连接/读取超时（毫秒）
};
```

### `TransportType` 枚举

```cpp
enum class TransportType { TCP, UDP, RS485 };
```

### 工厂方法

```cpp
// itransport.cpp
std::unique_ptr<ITransport> ITransport::create(
        TransportType type, const TransportConfig& config) {
    switch (type) {
        case TransportType::TCP:   return std::make_unique<TcpTransport>(config);
        case TransportType::UDP:   return std::make_unique<UdpTransport>(config);
        case TransportType::RS485: return std::make_unique<Rs485Transport>(config);
        default: throw std::invalid_argument("Unknown TransportType");
    }
}
```

---

## 2. `TcpTransport`

### 2.1 设计要点

- **全双工**：独立接收线程，发送通过 `send_mutex_` 串行化。
- **TCP_NODELAY**：禁用 Nagle 算法，减小单帧延迟。
- **连接超时**：通过 `SO_SNDTIMEO` / `SO_RCVTIMEO` 设置，默认 `timeout_ms`。
- **接收轮询**：`select()` 100ms 超时，`running_` 为 false 时退出。
- **自动重连**（限 TCP）：接收线程内若 `connected_ == false` 则以指数退避（500ms→16s）重连。

### 2.2 连接过程

```
connect()
  └─ connectSocket()
       ├─ getaddrinfo(host, port)
       ├─ socket(AF_INET, SOCK_STREAM)
       ├─ setsockopt(TCP_NODELAY, SO_REUSEADDR, SO_SNDTIMEO, SO_RCVTIMEO)
       ├─ connect(fd, addr)
       └─ socket_fd_ = fd
  └─ connected_ = true
  └─ running_   = true
  └─ std::thread(&TcpTransport::receiveLoop, this)
```

### 2.3 发送流程

```cpp
bool TcpTransport::send(const uint8_t* data, std::size_t len) {
    std::lock_guard<std::mutex> lock(send_mutex_);
    std::size_t total = 0;
    while (total < len) {
        ssize_t n = ::write(socket_fd_, data + total, len - total);
        if (n < 0) {
            if (errno == EINTR) continue;
            connected_.store(false);
            return false;
        }
        total += n;
    }
    return true;
}
```

关键：发送循环保证全部字节写出（处理 short write）。

### 2.4 接收线程（`receiveLoop`）

```
while (running_):
    if not connected_:
        sleep(backoff_ms); backoff_ms = min(backoff_ms*2, 16000)
        connectSocket() → connected_ = true; backoff_ms = 500
        continue

    select(fd, 100ms 超时)
    if timeout: continue
    recv(fd, buf, 4096)
    if n == 0: peer 断开 → connected_ = false; close(fd)
    if n  > 0: receive_cb_(buf, n)
```

### 2.5 关键常量

```cpp
static constexpr int  RECV_BUFFER_SIZE       = 4096;
static constexpr int  MAX_RECONNECT_ATTEMPTS = 5;   // 未直接用于限制，退避由 HAL 控制
static constexpr long INITIAL_BACKOFF_MS     = 500;
static constexpr long MAX_BACKOFF_MS         = 16000;
```

---

## 3. `UdpTransport`

### 3.1 设计要点

- **无连接**：`connect()` 只创建 socket、解析远端地址、bind 本地随机端口。
- **不支持自动重连**：UDP 本身无连接，断链判断不适用；`connected_` 仅跟踪 socket 是否打开。
- **接收**：`recvfrom()` 获取任意源的数据报，不做源地址过滤（MCU 可能从任意端口回复）。

### 3.2 连接过程

```
connect()
  ├─ socket(AF_INET, SOCK_DGRAM)
  ├─ getaddrinfo(host, port) → remote_addr_
  ├─ bind(INADDR_ANY, port=0)   // OS 分配本地端口
  ├─ connected_ = true
  └─ std::thread(&UdpTransport::receiveLoop, this)
```

### 3.3 发送

```cpp
::sendto(socket_fd_, data, len, 0, &remote_addr_, sizeof(remote_addr_))
```

UDP 单次 `sendto` 就能发出全帧（最大帧 ≈ 65535B，远小于 UDP 上限）。

### 3.4 接收线程

```
while (running_):
    select(fd, 100ms)
    recvfrom(fd, buf, 65535, &sender_addr)
    if n > 0: receive_cb_(buf, n)
```

### 3.5 适用场景

UDP 适用于高频里程计（≥50Hz）且可容忍偶发丢包的场景。命令类帧（急停、握手）建议用 TCP 保证可靠。

---

## 4. `Rs485Transport`

### 4.1 设计要点

- **半双工**：RS-485 总线同一时刻只能一端发送。发送前调 `tcflush(TCIFLUSH)` 清空接收缓冲，发送后调 `tcdrain()` 等待物理发送完成再释放总线。
- **串口配置**：8N1（8位数据，无奇偶，1位停止），无硬件流控，非阻塞读（VMIN=0, VTIME=1）。
- **不支持自动重连**：串口断开（设备文件消失）时接收线程退出，由 `ChassisHal` 重连循环负责重建。

### 4.2 连接过程

```
connect()
  ├─ open(device_path, O_RDWR | O_NOCTTY | O_NONBLOCK)
  ├─ configurePort()
  │   ├─ tcgetattr(fd, &tty)
  │   ├─ cfsetispeed / cfsetospeed (baudrate)
  │   ├─ cfmakeraw()   // raw模式
  │   ├─ 8N1 配置: PARENB=0, CSTOPB=0, CS8
  │   ├─ 无硬件流控: CRTSCTS=0
  │   ├─ VMIN=0, VTIME=1
  │   ├─ tcsetattr(TCSANOW)
  │   └─ tcflush(TCIOFLUSH)
  ├─ connected_ = true
  └─ std::thread(&Rs485Transport::receiveLoop, this)
```

### 4.3 发送（半双工保障）

```cpp
bool Rs485Transport::send(const uint8_t* data, std::size_t len) {
    std::lock_guard<std::mutex> lock(send_mutex_);
    ::tcflush(fd_, TCIFLUSH);          // 清空输入缓冲，防止 echo 干扰
    // write loop (处理 short write)
    ::tcdrain(fd_);                    // 等待所有字节物理发送完毕
    return true;
}
```

### 4.4 支持的波特率

```
9600 / 19200 / 38400 / 57600 / 115200（默认） /
230400 / 460800 / 921600
```

其他值自动降级到 B115200。

### 4.5 适用场景

RS-485 适用于工业现场通信、长距离（≤1200m）、多节点总线拓扑。单主多从时 MCU 地址寻址需应用层实现（当前协议为单主单从）。

---

## 5. 三种传输对比

| 特性 | TCP | UDP | RS485 |
|------|-----|-----|-------|
| 可靠性 | 有（TCP 保证） | 无（丢包不重传） | 有（物理层可靠） |
| 延迟 | 低（1ms 级 LAN） | 极低（0.1ms 级） | 低（取决于波特率） |
| 自动重连 | 接收线程内退避重连 | 无（socket 不重建） | 无（由 HAL 重连循环） |
| 最大帧大小 | 65535 B（受 LEN 字段限制） | 65535 B | 65535 B |
| 全/半双工 | 全双工 | 全双工 | 半双工 |
| 接收缓冲 | 4096 B / 次 | 65535 B / 数据报 | 512 B / 次 |
| 适用场景 | 机器人局域网 | 高频里程计 | 工业串口 MCU |

---

## 6. 接收回调的线程安全性

`receive_cb_` 在接收线程上下文中调用。`ChassisHal::onDataReceived()` 是被注册的回调，其内部操作：

1. `stream_decoder_.feed()` — 仅由接收线程调用，无锁（单生产者）
2. `stream_decoder_.poll()` — 同上
3. `onFrameDecoded()` → `dispatchStatusFrame()` 等 — 操作 `current_status_` 等共享状态时加对应 `std::mutex`

因此**接收线程**与**ROS2 主线程**（通过 `getStatus()` 等 getter 读状态）之间通过互斥锁同步。
