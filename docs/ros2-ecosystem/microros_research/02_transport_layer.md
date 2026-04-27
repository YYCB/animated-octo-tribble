# 传输层深度指南

## 一、传输层在 micro-ROS 栈中的位置

```
Micro XRCE-DDS Client
         ↕
┌────────────────────────────────────────────────┐
│  Transport Layer API                           │
│  uxr_set_serial_transport() /                  │
│  uxr_set_udp_transport() /                     │
│  uxr_set_custom_transport()                    │
└────────────────────────────────────────────────┘
         ↕
物理介质（UART / UDP / USB CDC / SPI / CAN）
```

micro-ROS 传输层的设计原则是**完全解耦**：底层传输实现与 XRCE 协议栈之间仅通过四个函数指针交互（`open` / `close` / `write` / `read`），任何物理介质只需实现这四个回调即可接入。

---

## 二、Serial / UART Transport

### 2.1 硬件配置要求

Serial 传输是 micro-ROS 最常用的传输方式，在 RS485 和 TTL UART 上均可工作。

| 参数 | 推荐值 | 说明 |
|------|-------|------|
| 波特率 | 921600 bps | 最大理论吞吐 ~92 KB/s，适合 ≤ 500 Hz JointState |
| 数据位 | 8 | XRCE 协议不依赖特定数据位 |
| 停止位 | 1 | |
| 校验 | None（XRCE 自带 CRC） | |
| 硬件流控 | RTS/CTS（可选） | 高频发送时建议启用防止丢帧 |
| DMA | **强烈推荐** | 无 DMA 时 CPU 占用率在 500 Hz 下可高达 30% |

### 2.2 STM32 DMA 配置示例

```c
// STM32H7 HAL 配置（CubeMX 生成基础 + micro-ROS 适配）
// micro-ROS 使用的 UART 实例（需在 CubeMX 中启用 DMA 接收）
extern UART_HandleTypeDef huart2;

// micro-ROS Serial 传输初始化（FreeRTOS 任务中调用）
bool cubemx_transport_open(struct uxrCustomTransport * transport) {
    // UART DMA 接收：使用空闲中断（IDLE Line Detection）
    // 实现变长帧接收，无需固定包长
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2,
        uart_rx_buffer, sizeof(uart_rx_buffer));
    return true;
}

bool cubemx_transport_close(struct uxrCustomTransport * transport) {
    HAL_UART_Abort(&huart2);
    return true;
}

size_t cubemx_transport_write(struct uxrCustomTransport * transport,
                               const uint8_t * buf, size_t len, uint8_t * err) {
    // DMA 发送，非阻塞
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart2,
        (uint8_t *)buf, len);
    *err = (status != HAL_OK) ? 1 : 0;
    return (status == HAL_OK) ? len : 0;
}

size_t cubemx_transport_read(struct uxrCustomTransport * transport,
                              uint8_t * buf, size_t len, int timeout, uint8_t * err) {
    // 从 DMA 环形缓冲区读取
    uint32_t deadline = HAL_GetTick() + timeout;
    while (ring_buffer_available(&uart_rx_ring) < len) {
        if (HAL_GetTick() >= deadline) {
            *err = 1;
            return 0;
        }
        taskYIELD();  // 让出 FreeRTOS 调度器
    }
    return ring_buffer_read(&uart_rx_ring, buf, len);
}

// 在 FreeRTOS 任务初始化时设置传输
rmw_uros_set_custom_transport(
    true,  // framing = true（XRCE 自动帧边界检测）
    NULL,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);
```

### 2.3 波特率与消息频率上限

XRCE 协议的 Serial 帧有固定开销（消息头 ~8 字节 + 帧边界 2 字节），实际可用带宽：

```
可用带宽（字节/秒）= 波特率 / 10 × 0.85（协议开销约 15%）

921600 bps → 约 78 KB/s 有效载荷

sensor_msgs/JointState（6 关节）：
  - name: 6 × 12 字节 = 72 字节（含 rosidl 序列开销）
  - position/velocity/effort: 6 × 8 × 3 = 144 字节
  - header: ~20 字节
  - 总计：~240 字节/帧

最大频率：78,000 / 240 ≈ 325 Hz（921600 bps）

若使用 1 Mbps（某些 RS485 模块支持）→ 约 350 Hz
若减少关节数或使用更紧凑的自定义消息 → 可达 500 Hz
```

### 2.4 CRC 校验

XRCE 协议自带 CRC-16（CRC_CCITT），在 `UXR_SERIAL_TRANSPORT` 模式下自动启用。配置：

```c
// micro_ros_utilities/config.h（根据项目需求调整）
#define UXR_SERIAL_TRANSPORT_MTU 512  // 最大传输单元（字节）
// 若消息超过 MTU，micro-ROS 自动进行分片（见第五章）
```

与 `chassis_protocol` 的 `frame_codec.cpp`（自定义 CRC-16 帧格式）对比：XRCE 的 CRC 覆盖整个 XRCE 消息（包含会话头），而 chassis_protocol 的 CRC 仅覆盖自定义帧载荷，两者覆盖范围不同。

---

## 三、UDP Transport（WiFi / 有线以太网）

### 3.1 ESP32 WiFi UDP 配置

```c
// ESP32 FreeRTOS + ESP-IDF micro-ROS UDP 传输
// 在 menuconfig 中设置 Agent IP（或运行时从 NVS 读取）
#define AGENT_IP    "192.168.1.100"
#define AGENT_PORT  8888

// micro-ROS 内置 ESP32 UDP 传输（位于 micro_ros_espidf_component）
set_microros_wifi_transports(
    "MySSID",          // WiFi SSID
    "MyPassword",      // WiFi 密码
    AGENT_IP,
    AGENT_PORT);
```

### 3.2 ESP32 WiFi 延迟特性

WiFi 的非确定性是其在控制应用中的主要局限：

| 条件 | 典型延迟 | 最坏延迟 | 原因 |
|------|---------|---------|------|
| 空闲信道，距 AP < 5m | 1–3 ms | 8 ms | 正常传输 |
| 信道竞争（5 个设备） | 3–10 ms | 30 ms | CSMA/CA 退避 |
| WiFi 省电模式（DTIM） | 10–100 ms | 200 ms | 唤醒延迟 |
| 2.4 GHz 干扰（微波炉） | 5–50 ms | 200 ms | 重传 |

**建议**：用于 IMU、电池等 ≤ 100 Hz 的非关键传感器。关节控制（≥ 200 Hz）应使用 Serial 或有线以太网。

```c
// 关闭 ESP32 WiFi 省电模式（牺牲功耗换延迟确定性）
esp_wifi_set_ps(WIFI_PS_NONE);
```

### 3.3 有线以太网（W5500 / LAN8720）确定性

通过 SPI 接入的 W5500 以太网芯片提供明显更好的确定性：

| 方案 | 延迟 | 抖动 | 成本 |
|------|------|------|------|
| ESP32 WiFi | 1–30 ms | ± 10 ms | 最低 |
| W5500（SPI 20MHz） | 0.5–2 ms | ± 0.3 ms | 低 |
| LAN8720（RMII） | 0.3–1 ms | ± 0.1 ms | 中（需 MCU RMII 支持） |
| ENC28J60（SPI 8MHz） | 1–5 ms | ± 0.5 ms | 极低（但速度慢） |

```c
// STM32H7 + W5500 micro-ROS UDP 传输
// 基于 ioLibrary_Driver + FreeRTOS
set_microros_wifi_transports_custom(
    w5500_send_callback,
    w5500_recv_callback,
    agent_ip_bytes,
    AGENT_PORT);
```

---

## 四、USB CDC Transport

### 4.1 STM32 USB CDC 配置

USB CDC（Communication Device Class）提供比串口更高的带宽，且 USB Full Speed（12 Mbps）延迟较低：

```c
// STM32 USB CDC micro-ROS 传输
// 依赖 STM32 USB Device Library + CDC Class

bool usb_cdc_transport_open(struct uxrCustomTransport * t) {
    MX_USB_DEVICE_Init();  // 初始化 USB 设备
    return true;
}

size_t usb_cdc_transport_write(struct uxrCustomTransport * t,
                                const uint8_t * buf, size_t len,
                                uint8_t * err) {
    uint32_t timeout = 100;
    while (CDC_Transmit_FS((uint8_t*)buf, len) == USBD_BUSY) {
        if (--timeout == 0) { *err = 1; return 0; }
        HAL_Delay(1);
    }
    return len;
}
```

### 4.2 Serial vs USB CDC 对比

| 特性 | Serial 921600 bps | USB CDC Full Speed |
|------|------------------|--------------------|
| 最大带宽 | ~92 KB/s | ~1.2 MB/s（理论），实际 ~500 KB/s |
| 延迟 | 0.5–3 ms | 0.2–2 ms |
| 确定性 | 高（无轮询） | 中（USB SOF 1ms 粒度） |
| 线缆长度 | RS485 可达 1200m | USB: ≤ 5m |
| 隔离需求 | RS485 自带差分 | 需要 USB 隔离器（工业场景） |
| MCU 支持 | 几乎所有 MCU | 需要 USB FS/HS 硬件 |
| 调试便利性 | 需要转接器 | 直接插 PC |

**结论**：USB CDC 适合开发调试（直连 PC）；RS485 Serial 适合工业生产环境（长距离、抗干扰）。

---

## 五、Custom Transport API

### 5.1 四个回调函数签名

任何自定义传输介质（SPI、CAN、I2C、蓝牙串口等）只需实现：

```c
// 1. open：初始化传输介质
bool custom_transport_open(struct uxrCustomTransport * transport);

// 2. close：释放资源
bool custom_transport_close(struct uxrCustomTransport * transport);

// 3. write：发送数据（非阻塞或短超时）
//    返回实际发送字节数；err 置 1 表示错误
size_t custom_transport_write(
    struct uxrCustomTransport * transport,
    const uint8_t * buf,
    size_t len,
    uint8_t * err);

// 4. read：接收数据（带超时）
//    timeout 单位：毫秒；返回实际读取字节数
size_t custom_transport_read(
    struct uxrCustomTransport * transport,
    uint8_t * buf,
    size_t len,
    int timeout,
    uint8_t * err);
```

### 5.2 注册自定义传输

```c
// framing 参数：
//   true  = 使用 XRCE Serial 帧格式（含帧头/CRC，适合 UART/SPI 等字节流介质）
//   false = 原始数据包模式（适合 UDP/以太网等天然有边界的介质）
rmw_uros_set_custom_transport(
    true,              // framing
    (void*)&my_config, // transport args（可传递任意配置结构体）
    custom_transport_open,
    custom_transport_close,
    custom_transport_write,
    custom_transport_read);
```

### 5.3 SPI 自定义传输示例（主从模式）

```c
// MCU 作为 SPI Slave，主机（Raspberry Pi）作为 SPI Master
// 通过 GPIO 引脚信号化数据就绪

static SPI_HandleTypeDef *hspi_transport;

bool spi_transport_open(struct uxrCustomTransport * t) {
    hspi_transport = (SPI_HandleTypeDef*)t->args;
    HAL_GPIO_WritePin(DATA_READY_GPIO, DATA_READY_PIN, GPIO_PIN_RESET);
    return true;
}

size_t spi_transport_write(struct uxrCustomTransport * t,
                            const uint8_t * buf, size_t len, uint8_t * err) {
    // 写入 DMA 缓冲区，拉高 DATA_READY GPIO 通知主机
    memcpy(spi_tx_buf, buf, len);
    spi_tx_len = len;
    HAL_GPIO_WritePin(DATA_READY_GPIO, DATA_READY_PIN, GPIO_PIN_SET);

    // 等待主机读取完成（通过另一 GPIO 信号）
    uint32_t deadline = HAL_GetTick() + 10;
    while (HAL_GPIO_ReadPin(MASTER_ACK_GPIO, MASTER_ACK_PIN) == GPIO_PIN_RESET) {
        if (HAL_GetTick() > deadline) { *err = 1; return 0; }
    }
    HAL_GPIO_WritePin(DATA_READY_GPIO, DATA_READY_PIN, GPIO_PIN_RESET);
    return len;
}
```

---

## 六、Fragmentation & Reassembly（分片与重组）

### 6.1 分片触发条件

当单条 ROS 消息序列化后的 CDR 数据大小超过传输层的 **MTU**（Maximum Transmission Unit）时，XRCE Client 自动进行分片：

```
默认 MTU（micro_ros_utilities）：
  Serial：512 字节（可配置至 4096）
  UDP：   1024 字节（可配置至 65507，但 WiFi 环境建议 ≤ 1400）

分片触发示例：
  sensor_msgs/PointCloud2（640×480，XYZI）≈ 3.7 MB → 7200+ 分片（Serial，不实用）
  sensor_msgs/CompressedImage（JPEG 100×100）≈ 2 KB → 4 分片（Serial 512B MTU）
  sensor_msgs/Image（320×240 RGB8）≈ 230 KB → 460 分片（Serial，不实用）
```

### 6.2 分片配置

```c
// 调整 MTU 大小（需重新编译 micro-ROS 静态库）
// 在 extra_packages/micro_ros_utilities/include/micro_ros_utilities/config.h 中：
#define UXR_SERIAL_TRANSPORT_MTU     2048   // Serial MTU 调大
#define UXR_UDP_TRANSPORT_MTU        1400   // UDP MTU（以太网最大帧安全值）

// 分片缓冲区配置（RELIABLE 流必须有足够的重传缓冲）
#define UXR_CONFIG_MAX_OUTPUT_RELIABLE_STREAMS    2
#define UXR_CONFIG_MAX_INPUT_RELIABLE_STREAMS     2
#define UXR_CONFIG_SERIAL_TRANSPORT_MTU           2048
```

### 6.3 实用建议

| 消息类型 | 数据量 | 建议方案 |
|---------|-------|---------|
| JointState（6 关节） | ~240 B | 直接传输，无需分片 |
| IMU（sensor_msgs/Imu） | ~128 B | 直接传输 |
| 压缩图像缩略图（64×64 JPEG） | ~1–5 KB | 调大 MTU 后分片可行，但频率应 ≤ 10 Hz |
| 点云 / 全分辨率图像 | 数百 KB | **不推荐**通过 micro-ROS 传输，应使用以太网直接传输 |

---

## 七、Discovery vs Static Agent 配置

### 7.1 Static Agent（推荐生产环境）

MCU 端硬编码 Agent 的 IP/端口，启动即直连，无需发现协议：

```c
// UDP 模式：硬编码 Agent 地址
set_microros_wifi_transports(SSID, PWD, "192.168.1.100", 8888);

// Serial 模式：无 IP，默认连接串口对端（即 Agent）
set_microros_serial_transports(huart2);
```

Agent 端也无需额外配置，直接监听指定端口/设备。

### 7.2 Dynamic Discovery（开发调试方便）

通过 `RMW_UXRCE_TRANSPORT` 环境变量和 mDNS 发现 Agent：

```bash
# 主机端：启动 Agent 并广播 mDNS
ros2 run micro_ros_agent micro_ros_agent udp4 \
    --port 8888 \
    --discovery  # 启用 mDNS 广播（需要 avahi-daemon）
```

MCU 端通过 mDNS 查询 `_microros._udp.local` 发现 Agent IP，适合开发阶段 IP 地址变动频繁的场景。

### 7.3 环境变量配置

```bash
# 配置 Agent 地址（ESP32 可从 NVS 读取，等效于环境变量）
export RMW_UXRCE_DEFAULT_UDP_IP=192.168.1.100
export RMW_UXRCE_DEFAULT_UDP_PORT=8888

# 会话超时（毫秒）：Agent 多久没收到心跳认为 Client 断线
export RMW_UXRCE_SESSION_TIMEOUT_MS=5000  # 默认 5000ms

# 创建实体超时
export RMW_UXRCE_CREATION_DESTROY_TIMEOUT_MS=1000
```

---

## 八、可靠性模式：BEST_EFFORT vs RELIABLE

### 8.1 两种模式的 MCU 资源对比

| 特性 | BEST_EFFORT | RELIABLE |
|------|------------|---------|
| MCU RAM 额外开销 | ~0 | ~MTU × 流数量（重传缓冲） |
| 延迟 | 最低（无 ACK 等待） | 增加 ACK 往返时间 |
| 消息可达性 | 无保证（丢帧不重传） | 保证可达（自动重传） |
| 适用场景 | 高频传感器数据 | 配置命令、服务请求 |
| XRCE 流类型 | `UXR_BEST_EFFORT_STREAM` | `UXR_RELIABLE_STREAM` |

### 8.2 QoS 配置

```c
// BEST_EFFORT Publisher（传感器数据）
rclc_publisher_init(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu/data",
    &rmw_qos_profile_sensor_data);  // depth=5, BEST_EFFORT

// RELIABLE Subscriber（控制命令）
rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "/joint_commands");  // default = RELIABLE, depth=10
```

### 8.3 混合使用策略（推荐）

```c
// 在同一个 session 上使用两条流
uxrStreamId best_effort_stream = uxr_create_output_best_effort_stream(
    &session, be_stream_buf, sizeof(be_stream_buf));

uxrStreamId reliable_stream = uxr_create_output_reliable_stream(
    &session, rel_stream_buf, sizeof(rel_stream_buf),
    RELIABLE_STREAM_DEPTH);  // RELIABLE_STREAM_DEPTH = 重传窗口大小

// IMU 数据 → best_effort_stream
// 关节命令确认 → reliable_stream
```
