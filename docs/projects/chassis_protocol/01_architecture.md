# 01 整体架构

## 1. 分层模型

```
┌────────────────────────────────────────────────────────┐
│  ROS2 层  (namespace: chassis_ros2)                     │
│  chassis_node_main.cpp → ChassisNode                   │
│  订阅 /cmd_vel，发布 /odom /battery_state /diagnostics  │
└────────────────────┬───────────────────────────────────┘
                     │ std::unique_ptr<IChassisController>
┌────────────────────▼───────────────────────────────────┐
│  控制器层  (namespace: chassis)                          │
│  IChassisController  ←──────── 纯抽象接口               │
│    ├── ChassisHal              通用 HAL 基类             │
│    │     ├── DifferentialController                     │
│    │     └── MecanumController                          │
└────────────────────┬───────────────────────────────────┘
                     │ std::unique_ptr<ITransport>
┌────────────────────▼───────────────────────────────────┐
│  传输层  (namespace: chassis)                            │
│  ITransport  ←──────── 纯抽象接口                        │
│    ├── TcpTransport                                     │
│    ├── UdpTransport                                     │
│    └── Rs485Transport                                   │
└────────────────────┬───────────────────────────────────┘
                     │ POSIX socket fd / tty fd
┌────────────────────▼───────────────────────────────────┐
│  物理链路                                                │
│  Ethernet (TCP/UDP) / RS-485 串口                        │
└────────────────────────────────────────────────────────┘
```

### 横切关注点（贯穿多层）

| 组件 | 所在文件 | 职责 |
|------|---------|------|
| `FrameCodec` | `frame/frame_codec.hpp/.cpp` | 帧编解码、CRC16 计算 |
| `StreamDecoder` | `frame/frame_codec.hpp/.cpp` | 流式字节解帧（状态机） |
| `VersionNegotiator` | `chassis/version_negotiator.hpp/.cpp` | 连接握手、版本协商 |

---

## 2. 目录结构

```
chassis_protocol/
├── CMakeLists.txt
├── package.xml
├── proto/
│   └── chassis.proto              # Protobuf schema（纯文档用途）
├── frame/
│   ├── frame_codec.hpp            # 帧格式常量、FrameCodec、StreamDecoder
│   └── frame_codec.cpp
├── transport/
│   ├── itransport.hpp             # ITransport 接口 + TransportConfig + 工厂方法
│   ├── itransport.cpp             # ITransport::create() 工厂实现
│   ├── tcp_transport.{hpp,cpp}
│   ├── udp_transport.{hpp,cpp}
│   └── rs485_transport.{hpp,cpp}
├── chassis/
│   ├── ichassis_controller.hpp    # 数据结构 + IChassisController 接口
│   ├── chassis_hal.{hpp,cpp}      # HAL 基类实现
│   ├── version_negotiator.{hpp,cpp}
│   ├── differential/
│   │   ├── differential_controller.hpp
│   │   └── differential_controller.cpp
│   └── mecanum/
│       ├── mecanum_controller.hpp
│       └── mecanum_controller.cpp
├── ros2_adapter/
│   ├── chassis_node.{hpp,cpp}     # ROS2 节点封装
│   └── chassis_node_main.cpp      # main()
├── tests/
│   ├── frame_codec_test.cpp
│   ├── transport_mock_test.cpp
│   └── fixtures/
└── docs/
    └── design/                    # ← 本文档集
```

---

## 3. 模块依赖关系（编译单元级别）

```
chassis_node_main.cpp
    └─ chassis_protocol_ros2_adapter (chassis_node.cpp)
           └─ chassis_protocol_hal
                  ├─ chassis_hal.cpp
                  ├─ version_negotiator.cpp
                  ├─ differential_controller.cpp
                  ├─ mecanum_controller.cpp
                  └─ chassis_protocol_transport
                         ├─ itransport.cpp   (工厂)
                         ├─ tcp_transport.cpp
                         ├─ udp_transport.cpp
                         ├─ rs485_transport.cpp
                         └─ chassis_protocol_frame
                                └─ frame_codec.cpp
```

CMake 目标：

| 目标名 | 类型 | 主要源文件 |
|--------|------|-----------|
| `chassis_protocol_frame` | STATIC | `frame/frame_codec.cpp` |
| `chassis_protocol_transport` | STATIC | `transport/*.cpp` |
| `chassis_protocol_hal` | STATIC | `chassis/*.cpp` + 子目录 |
| `chassis_protocol_ros2_adapter` | STATIC | `ros2_adapter/chassis_node.cpp` |
| `chassis_node` | EXECUTABLE | `ros2_adapter/chassis_node_main.cpp` |

---

## 4. 核心数据类型速览

### `IChassisController` 接口（`chassis/ichassis_controller.hpp`）

```cpp
// 控制指令
bool sendVelocity(double vx, double vy, double omega);
bool sendLightCmd(LightMode mode, uint8_t r, uint8_t g, uint8_t b, double blink_hz);
bool emergencyStop(const std::string& reason);
bool sendWheelCmd(const std::vector<double>& wheel_rpms_rads); // 默认返回 false

// 状态读取
ChassisStatus  getStatus()      const;
ChassisInfo    getInfo()        const;
OdometryData   getOdometry()    const;
BatteryData    getBatteryInfo() const;

// 异步回调注册
void setStatusCallback(std::function<void(const ChassisStatus&)> cb);
void setOdometryCallback(std::function<void(const OdometryData&)> cb);
void setBatteryCallback(std::function<void(const BatteryData&)> cb);
```

### 枚举类型

```cpp
enum class ChassisType { DIFFERENTIAL, MECANUM, OMNI, UNKNOWN };
enum class LightMode    { OFF, SOLID, BLINK, BREATH };
enum class ControlMode  { BODY_VELOCITY, WHEEL_RPM };
```

### 关键结构体

| 结构体 | 字段 | 用途 |
|--------|------|------|
| `ChassisInfo` | vendor, model, chassis_type, firmware_version, protocol_version | 设备元信息 |
| `ChassisStatus` | timestamp_us, error_code, error_msg, is_estop | 实时状态 |
| `OdometryData` | x/y/z, q{x,y,z,w}, vx/vy/omega, cov_pose[36], cov_twist[36] | 里程计 |
| `BatteryData` | voltage, current, percentage, temperature, is_charging, cell_voltages[] | 电池 |

---

## 5. 线程模型

| 线程 | 创建者 | 职责 |
|------|--------|------|
| 接收线程 | `TcpTransport::connect()` / `UdpTransport::connect()` / `Rs485Transport::connect()` | 阻塞 select()，收到字节后调 `receive_cb_` |
| 重连线程 | `ChassisHal::connect()` | 每 200ms 检测连接，必要时重跑握手 |
| ROS2 主线程 | `rclcpp::spin()` | 处理 /cmd_vel 回调，驱动发布 |

**互斥锁分配**（全部在 `ChassisHal`）：

| 锁 | 保护的资源 |
|----|-----------|
| `status_mutex_` | `current_status_`, `info_` |
| `odom_mutex_` | `current_odom_` |
| `battery_mutex_` | `current_battery_` |
| `cb_mutex_` | `status_cb_`, `odom_cb_`, `battery_cb_` |
| `send_mutex_`（各 Transport） | 串行化 `write()`/`sendto()` 系统调用 |
