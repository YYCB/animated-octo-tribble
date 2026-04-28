# micro-ROS 核心架构

## 一、micro-ROS 软件栈分层

micro-ROS 是 ROS 2 向资源受限嵌入式系统的官方扩展，完整栈如下：

```
┌─────────────────────────────────────────────────────────────────┐
│  应用层（用户代码）                                                │
│  rclc_executor_spin_some() / 用户回调函数                         │
└─────────────────────────────────────────────────────────────────┘
         ↕
┌─────────────────────────────────────────────────────────────────┐
│  rclc（ROS Client Library for C）                                │
│  rcl_node_t / rcl_publisher_t / rcl_subscription_t              │
│  rcl_timer_t / rcl_service_t / rcl_client_t                     │
└─────────────────────────────────────────────────────────────────┘
         ↕
┌─────────────────────────────────────────────────────────────────┐
│  rmw_microxrcedds（ROS Middleware 实现，面向 XRCE-DDS）            │
│  rmw_publisher_t / rmw_subscription_t → uxrObjectId             │
└─────────────────────────────────────────────────────────────────┘
         ↕
┌─────────────────────────────────────────────────────────────────┐
│  Micro XRCE-DDS Client 库（eProsima）                             │
│  uxrSession / uxrDataWriter / uxrDataReader                     │
│  CDR 序列化 / XRCE 协议帧封装                                     │
└─────────────────────────────────────────────────────────────────┘
         ↕ Serial/UDP/USB CDC/Custom
┌─────────────────────────────────────────────────────────────────┐
│  传输层（Transport Layer）                                        │
│  uxrCustomTransport / uxrSerialTransport / uxrUDPTransport      │
└─────────────────────────────────────────────────────────────────┘
         ↕ 物理总线（RS485/UART/WiFi/USB）
┌─────────────────────────────────────────────────────────────────┐
│  micro-ROS Agent（主机端进程，eProsima）                           │
│  Micro XRCE-DDS Agent ←→ DDS DataWriter/DataReader 桥接          │
└─────────────────────────────────────────────────────────────────┘
         ↕ DDS（CycloneDDS / FastDDS）
┌─────────────────────────────────────────────────────────────────┐
│  标准 ROS 2 节点（主机端）                                         │
│  ros2_control / Nav2 / MoveIt 2 / rosbag2                       │
└─────────────────────────────────────────────────────────────────┘
```

---

## 二、rclc vs rclcpp：API 详细对比

### 2.1 Node 创建

**rclcpp（主机端 C++）**：
```cpp
// rclcpp 方式
auto node = rclcpp::Node::make_shared("joint_controller");
```

**rclc（MCU 端 C）**：
```c
// rclc 方式 — 全部使用栈分配或静态分配，无 new/malloc
rcl_allocator_t allocator = rcl_get_default_allocator();
rclc_support_t support;
rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
rcl_init_options_init(&init_options, allocator);

rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

rcl_node_t node = rcl_get_zero_initialized_node();
rclc_node_init_default(&node, "joint_controller", "robot1", &support);
```

### 2.2 Publisher / Subscriber

| 特性 | rclcpp | rclc |
|------|--------|------|
| 语言 | C++17 | C99/C11 |
| 内存 | heap（std::make_shared） | 静态/栈（用户分配） |
| 类型安全 | 模板编译期检查 | void* 运行时消息指针 |
| QoS | QoS 对象 | rmw_qos_profile_t |
| 回调 | std::function / lambda | 函数指针 |
| Executor | rclcpp::Executor（多种） | rclc_executor_t（单一） |

```c
// rclc Publisher
sensor_msgs__msg__JointState joint_state_msg;
sensor_msgs__msg__JointState__init(&joint_state_msg);

rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();
rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/joint_states");

// rclc Subscriber（需提前分配消息内存）
std_msgs__msg__Float64MultiArray cmd_msg;
std_msgs__msg__Float64MultiArray__init(&cmd_msg);

rcl_subscription_t subscriber = rcl_get_zero_initialized_subscription();
rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "/joint_commands");
```

### 2.3 Executor

rclc_executor 是 micro-ROS 的核心调度器，设计为**单线程事件循环**，适配 RTOS 任务模型：

```c
rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
// handles 参数：需要管理的实体总数（subscription + timer + service 之和）
rclc_executor_init(&executor, &support.context, 3, &allocator);

// 注册 subscription
rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &cmd_msg,
    &joint_command_callback,
    ON_NEW_DATA);  // 或 ALWAYS（每 spin 都调用）

// 注册 timer
rclc_executor_add_timer(&executor, &timer);

// 主循环：timeout_ns = 100ms，在 FreeRTOS task 中调用
while (true) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    vTaskDelay(pdMS_TO_TICKS(1));  // 让出 CPU 给其他任务
}
```

### 2.4 Timer

```c
rcl_timer_t timer = rcl_get_zero_initialized_timer();
// 500 Hz = 2,000,000 ns 周期
rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(2), publish_joint_states);
```

### 2.5 Service / Client

```c
// Service 端（MCU 上运行，响应主机的配置请求）
rcl_service_t service = rcl_get_zero_initialized_service();
rclc_service_init_default(
    &service, &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
    "/enable_torque");

rclc_executor_add_service(
    &executor, &service,
    &request_msg, &response_msg,
    enable_torque_callback);
```

### 2.6 Parameter Server

```c
// rclc 参数服务器（支持 bool/int/double/string/byte_array）
rclc_parameter_server_t param_server;
rclc_parameter_server_init_default(&param_server, &node, &executor);

// 添加参数
rclc_add_parameter(&param_server, "kp", RCLC_PARAMETER_DOUBLE);
rclc_add_parameter(&param_server, "ki", RCLC_PARAMETER_DOUBLE);
rclc_add_parameter(&param_server, "enabled", RCLC_PARAMETER_BOOL);

// 设置默认值
rclc_parameter_set_double(&param_server, "kp", 1.0);
rclc_parameter_set_bool(&param_server, "enabled", true);
```

---

## 三、Micro XRCE-DDS 协议原理

### 3.1 XRCE 设计目标

XRCE（**eX**tremely **R**esource **C**onstrained **E**nvironments）是 OMG 标准化的轻量 DDS 接入协议，专为以下场景设计：
- Flash < 2 MB，RAM < 256 KB 的 MCU
- 无 TCP/IP 完整栈的场景（Serial/UART）
- 间歇性连接（WiFi 休眠/唤醒）

### 3.2 Client-Agent 通信模型

```
┌─────────────────────────────────┐      ┌────────────────────────────────┐
│  XRCE-DDS Client（MCU）          │      │  XRCE-DDS Agent（主机进程）     │
│                                 │      │                                │
│  uxrSession                     │─────>│  ProxyClient（per session）    │
│  │                              │ XRCE │  │                             │
│  ├─ uxrDataWriter               │<─────│  ├─ DDS DataWriter             │
│  │   └─ ObjectId: 0x0001        │      │  │   └─ Topic: /joint_states   │
│  └─ uxrDataReader               │      │  └─ DDS DataReader             │
│      └─ ObjectId: 0x0002        │      │      └─ Topic: /joint_commands │
└─────────────────────────────────┘      └────────────────────────────────┘
                                                       ↕ DDS（UDP/SHM）
                                         ┌────────────────────────────────┐
                                         │  标准 ROS 2 节点               │
                                         │  ros2_control / Nav2 等        │
                                         └────────────────────────────────┘
```

### 3.3 XRCE 帧结构

每个 XRCE 数据包由**消息头 + 子消息序列**组成：

```
┌──────────────────────────────────────────────────────────┐
│  Message Header（4 字节）                                 │
│  session_id(1) + stream_id(1) + sequence_nr(2)           │
├──────────────────────────────────────────────────────────┤
│  SubMessage 1: WRITE_DATA                                 │
│  ├─ SubMessage Header（4 字节）：id(1) + flags(1) + len(2)│
│  └─ Payload：ObjectId(2) + CDR 序列化数据                │
├──────────────────────────────────────────────────────────┤
│  SubMessage 2: HEARTBEAT（RELIABLE 模式）                  │
│  └─ first_unacked_seq / last_unacked_seq                 │
└──────────────────────────────────────────────────────────┘
```

### 3.4 流（Stream）类型

| 流类型 | stream_id | 语义 | MCU 端 RAM 开销 |
|--------|-----------|------|----------------|
| `UXR_BEST_EFFORT_STREAM` | 0x01–0x7F | 无确认，最低延迟 | 极小（无重传缓冲） |
| `UXR_RELIABLE_STREAM` | 0x81–0xFF | 带 ACK/NACK + 重传 | 需配置 `UXR_CONFIG_MAX_OUTPUT_RELIABLE_STREAMS` 缓冲区 |

> **最佳实践**：高频传感器数据（IMU、关节状态）用 BEST_EFFORT；配置命令、服务请求用 RELIABLE。

---

## 四、micro-ROS Agent 架构

### 4.1 Agent 的角色

micro-ROS Agent（`micro_ros_agent` 包）是主机端运行的**协议桥接进程**，核心职责：

1. **Session 管理**：每个 MCU 连接对应一个 `ProxyClient`，维护 session_id、sequence_number、心跳超时
2. **对象代理**：Client 创建的 DataWriter/DataReader/Topic/Publisher/Subscriber 在 Agent 侧映射为等价的 DDS 实体
3. **DDS 桥接**：将 XRCE WRITE_DATA 转换为 DDS DataWriter::write()；将 DDS DataReader 收到的数据封装为 XRCE READ_DATA 推送给 Client
4. **多客户端复用**：单个 Agent 实例可同时服务多个 MCU（不同 session_id 区分）

### 4.2 启动方式

```bash
# Serial 传输（最常用）
ros2 run micro_ros_agent micro_ros_agent serial \
    --dev /dev/ttyUSB0 \
    --baudrate 921600 \
    -v4  # 详细日志

# UDP 传输（ESP32 WiFi）
ros2 run micro_ros_agent micro_ros_agent udp4 \
    --port 8888 \
    -v4

# USB CDC 传输
ros2 run micro_ros_agent micro_ros_agent serial \
    --dev /dev/ttyACM0 \
    --baudrate 115200

# 自定义传输（通过 Unix domain socket）
ros2 run micro_ros_agent micro_ros_agent custom \
    --transport <custom_transport_lib.so>
```

### 4.3 多 Agent 部署（多 MCU 独立 Agent）

```bash
# MCU 1（关节驱动）— /dev/ttyUSB0，domain 0
ROS_DOMAIN_ID=0 ros2 run micro_ros_agent micro_ros_agent serial \
    --dev /dev/ttyUSB0 --baudrate 921600

# MCU 2（IMU）— /dev/ttyUSB1，同一 domain
ROS_DOMAIN_ID=0 ros2 run micro_ros_agent micro_ros_agent serial \
    --dev /dev/ttyUSB1 --baudrate 115200

# MCU 3（ESP32 WiFi）— UDP
ROS_DOMAIN_ID=0 ros2 run micro_ros_agent micro_ros_agent udp4 --port 8889
```

---

## 五、支持的 RTOS 及集成复杂度

| RTOS | 官方支持等级 | 集成复杂度 | micro-ROS 初始化模式 | 典型硬件 |
|------|------------|-----------|---------------------|---------|
| **FreeRTOS** | ✅ 官方一级支持 | ★☆☆☆☆（最低） | `micro_ros_freertos_component`；单独 Task | STM32, ESP32, Teensy |
| **Zephyr RTOS** | ✅ 官方一级支持 | ★★☆☆☆ | 模块化集成；`zephyr_module.cmake` | nRF5340, STM32 |
| **NuttX** | ✅ 官方一级支持 | ★★★☆☆ | PX4 生态；`boards/` 配置 | Pixhawk, STM32 |
| **ThreadX** | ⚠️ 社区支持 | ★★★☆☆ | STM32 Azure RTOS | STM32L5/U5 |
| **bare-metal** | ✅ 官方支持 | ★★☆☆☆ | `micro_ros_arduino`（无 RTOS） | Arduino Nano, Pico |

### 5.1 FreeRTOS 集成模式

```c
// FreeRTOS Task 包装 micro-ROS（推荐模式）
void micro_ros_task(void *arg) {
    // 设置传输（在任务内初始化，确保 FreeRTOS heap 就绪）
    set_microros_serial_transports(huart2);  // STM32 HAL UART

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);

    // ... 节点、发布器、订阅器初始化 ...

    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    // ... 注册实体 ...

    // 循环：spin_some 让出 CPU 给其他 FreeRTOS tasks
    for (;;) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        usleep(10000);  // 10ms，对应 ~100Hz 上限
    }
}

// 在 main() 中启动
xTaskCreate(micro_ros_task, "micro_ros_task",
            16384,   // stack 大小（字节），micro-ROS 需要较大栈
            NULL, 5, NULL);
```

### 5.2 Zephyr 集成模式

```c
// Zephyr 线程包装
K_THREAD_DEFINE(uros_thread_id, 8192,
                micro_ros_zephyr_thread, NULL, NULL, NULL,
                5, 0, 1000);  // 优先级 5，延迟 1s 后启动

// CMakeLists.txt 添加
// find_package(microros)
// target_link_libraries(app PRIVATE microros)
```

---

## 六、支持硬件资源对比

| 平台 | Flash（micro-ROS 占用） | RAM（运行时最小） | 典型发布频率上限 | 推荐应用 |
|------|----------------------|----------------|----------------|---------|
| STM32F4（168MHz/1MB/192KB） | ~450 KB | ~48 KB | 500 Hz | 入门关节驱动 |
| STM32H7（480MHz/2MB/1MB） | ~500 KB | ~64 KB | 1 kHz | 高性能关节驱动 |
| ESP32-S3（240MHz/8MB/512KB） | ~1.2 MB（含 WiFi 栈） | ~128 KB | 200 Hz（WiFi），1 kHz（Serial） | IMU/WiFi 节点 |
| Teensy 4.1（600MHz/1MB+QSPI） | ~600 KB | ~80 KB | 1 kHz | 原型验证 |
| RPi Pico（133MHz/2MB/264KB） | ~400 KB | ~40 KB | 200 Hz | 低成本节点 |
| nRF5340（128MHz/1MB/512KB） | ~450 KB | ~56 KB | 500 Hz | 无线节点（Zephyr） |

---

## 七、消息定义：rosidl 生成的 C 结构体

### 7.1 标准消息（预编译进 micro-ROS 库）

micro-ROS 静态库预编译了 ROS 2 标准消息包（`std_msgs`、`geometry_msgs`、`sensor_msgs` 等），直接 include 即可：

```c
#include <sensor_msgs/msg/joint_state.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float64_multi_array.h>

// 消息初始化（分配内部序列内存）
sensor_msgs__msg__JointState joint_state;
sensor_msgs__msg__JointState__init(&joint_state);

// 设置字符串字段（名称数组）
rosidl_runtime_c__String__Sequence__init(&joint_state.name, 6);
rosidl_runtime_c__String__assign(&joint_state.name.data[0], "joint1");

// 设置数值数组
rosidl_runtime_c__double__Sequence__init(&joint_state.position, 6);
joint_state.position.data[0] = 1.5708;  // π/2 rad

// 用完释放
sensor_msgs__msg__JointState__fini(&joint_state);
```

### 7.2 自定义消息编译进固件

自定义消息（如项目特有的 `chassis_msgs/msg/MotorStatus`）需通过 `extra_packages` 机制重新编译 micro-ROS 静态库。详见 [06_firmware_build.md](./06_firmware_build.md)。

---

## 八、开发路径对比

### 8.1 micro_ros_arduino（最快入门）

```
优点：
  - Arduino IDE 直接使用，无需 CMake
  - 预编译库，5 分钟上手
  - Teensy 4.1 / Arduino Portenta 原生支持

缺点：
  - 自定义消息需重新编译整个库（耗时 30–60 min）
  - 受 Arduino 内存模型限制
  - 不适合生产环境
```

### 8.2 micro_ros_platformio（中间路径）

```
优点：
  - PlatformIO 工程管理，CMake 底层
  - 支持 ESP32、STM32、Teensy
  - library.json 管理依赖

缺点：
  - 自定义消息同样需要重新编译库
  - ESP-IDF 兼容性有时需手动处理
```

### 8.3 micro_ros_setup（生产环境推荐）

```
优点：
  - 完整 CMake/colcon 构建链
  - 自定义消息直接通过 extra_packages 集成
  - 支持所有 RTOS 和平台
  - CI/CD 友好

缺点：
  - 初始设置复杂（需要 ROS 2 完整环境）
  - 编译时间长（首次 20–40 min）
```

详细构建流程见 [06_firmware_build.md](./06_firmware_build.md)。
