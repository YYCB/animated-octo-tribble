# micro-ROS + ros2_control 分布式硬件接口

## 一、架构模式概述

micro-ROS 与 ros2_control 的集成代表了**分布式硬件接口**架构的核心模式：将传统单机 `SystemInterface` 的 `read()/write()` 调用从本地共享内存扩展为跨网络的 DDS 发布/订阅。

```
┌─────────────────────────────────────────────────────────────────────────────┐
│  传统 ros2_control 架构（单机）                                               │
│                                                                             │
│  controller_manager::update() ──> SystemInterface::read()                  │
│       [1 kHz RT线程]              [本地内存读取，< 1μs]                      │
│                          <── SystemInterface::write()                       │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│  micro-ROS 分布式架构                                                        │
│                                                                             │
│  主机：controller_manager::update() ──> SystemInterface::read()             │
│              [1 kHz RT线程]               [从 RealtimeBuffer 读最新值]       │
│                                 <── SystemInterface::write()                │
│                                      [向 RealtimeBuffer 写指令]              │
│                                                ↕ 非RT后台线程（DDS发布/订阅）  │
│                                      DDS DataWriter / DataReader            │
│                                                ↕                            │
│                                      micro-ROS Agent（主机进程）             │
│                                                ↕  XRCE-DDS                 │
│  MCU：rclc Publisher (/joint_states) ←─────────┘                            │
│        rclc Subscriber (/joint_commands) ──────┘                            │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 二、`micro_ros_hardware_interface` 实现思路

官方尚无标准的 `micro_ros_hardware_interface` 包，但基于 `ros2_control` 的 `hardware_interface::SystemInterface` 可实现如下架构：

### 2.1 类定义

```cpp
// micro_ros_system_interface.hpp
#pragma once
#include <hardware_interface/system_interface.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>

class MicroRosSystemInterface : public hardware_interface::SystemInterface {
public:
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    std::vector<hardware_interface::StateInterface>
    export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface>
    export_command_interfaces() override;

    hardware_interface::return_type read(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

private:
    // 关节状态双缓冲（RT 安全）
    struct JointStates {
        std::vector<double> position;
        std::vector<double> velocity;
        std::vector<double> effort;
        rclcpp::Time        stamp;
    };
    realtime_tools::RealtimeBuffer<JointStates> joint_state_buffer_;

    // 关节命令双缓冲
    struct JointCommands {
        std::vector<double> data;
    };
    realtime_tools::RealtimeBuffer<JointCommands> joint_cmd_buffer_;

    // 与 controller_manager 共享的状态/命令内存（hardware_interface 标准接口）
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    std::vector<double> hw_efforts_;
    std::vector<double> hw_commands_;

    // DDS 通信（非 RT，在独立线程中运行）
    rclcpp::Node::SharedPtr comm_node_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_pub_;
    rclcpp::executors::SingleThreadedExecutor comm_executor_;
    std::thread comm_thread_;

    size_t num_joints_;
    std::string joint_states_topic_;
    std::string joint_commands_topic_;
};
```

### 2.2 on_init() 实现

```cpp
hardware_interface::CallbackReturn MicroRosSystemInterface::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    num_joints_ = info_.joints.size();
    hw_positions_.resize(num_joints_, 0.0);
    hw_velocities_.resize(num_joints_, 0.0);
    hw_efforts_.resize(num_joints_, 0.0);
    hw_commands_.resize(num_joints_, 0.0);

    // 从 URDF hardware 参数读取 topic 名称
    joint_states_topic_   = info_.hardware_parameters.at("joint_states_topic");
    joint_commands_topic_ = info_.hardware_parameters.at("joint_commands_topic");

    // 初始化 RealtimeBuffer
    JointStates init_states;
    init_states.position.resize(num_joints_, 0.0);
    init_states.velocity.resize(num_joints_, 0.0);
    init_states.effort.resize(num_joints_, 0.0);
    joint_state_buffer_.writeFromNonRT(init_states);

    JointCommands init_cmds;
    init_cmds.data.resize(num_joints_, 0.0);
    joint_cmd_buffer_.writeFromNonRT(init_cmds);

    return hardware_interface::CallbackReturn::SUCCESS;
}
```

### 2.3 on_activate()：启动通信线程

```cpp
hardware_interface::CallbackReturn MicroRosSystemInterface::on_activate(
    const rclcpp_lifecycle::State & /*state*/)
{
    // 创建通信节点（非 RT，与 controller_manager 不同线程）
    comm_node_ = rclcpp::Node::make_shared("micro_ros_hw_comm");

    // 订阅 MCU 发布的关节状态（BEST_EFFORT QoS 匹配 MCU 端）
    rclcpp::QoS sub_qos(rclcpp::KeepLast(10));
    sub_qos.best_effort();
    joint_state_sub_ = comm_node_->create_subscription<sensor_msgs::msg::JointState>(
        joint_states_topic_, sub_qos,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            JointStates states;
            states.position = msg->position;
            states.velocity = msg->velocity;
            states.effort   = msg->effort;
            states.stamp    = msg->header.stamp;
            // 从非 RT 线程写入 RealtimeBuffer（无锁）
            joint_state_buffer_.writeFromNonRT(states);
        });

    // 发布关节命令到 MCU
    rclcpp::QoS pub_qos(rclcpp::KeepLast(1));
    pub_qos.best_effort();
    joint_cmd_pub_ = comm_node_->create_publisher<std_msgs::msg::Float64MultiArray>(
        joint_commands_topic_, pub_qos);

    // 启动通信线程（独立于 controller_manager RT 线程）
    comm_thread_ = std::thread([this]() {
        comm_executor_.add_node(comm_node_);
        comm_executor_.spin();
    });

    return hardware_interface::CallbackReturn::SUCCESS;
}
```

### 2.4 read() / write() 实现（RT 安全）

```cpp
// read()：从 RealtimeBuffer 读取最新的 MCU 状态数据
// 调用上下文：controller_manager RT 线程，1 kHz
hardware_interface::return_type MicroRosSystemInterface::read(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    // readFromRT()：无锁读取，永不阻塞（RealtimeBuffer 核心特性）
    const JointStates * states = joint_state_buffer_.readFromRT();
    if (states && states->position.size() == num_joints_) {
        for (size_t i = 0; i < num_joints_; ++i) {
            hw_positions_[i]  = states->position[i];
            hw_velocities_[i] = states->velocity[i];
            hw_efforts_[i]    = states->effort[i];
        }
    }
    // 若 states 为空（MCU 尚未发布）：保持上次值（ZOH — Zero Order Hold）
    return hardware_interface::return_type::OK;
}

// write()：将控制器命令写入 RealtimeBuffer，由通信线程异步发布
hardware_interface::return_type MicroRosSystemInterface::write(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    JointCommands cmds;
    cmds.data = hw_commands_;
    // writeFromRT()：无锁写入
    joint_cmd_buffer_.writeFromRT(cmds);

    // 通知通信线程发布（使用 RealtimeBuffer 的 readFromNonRT 在通信线程中完成实际发布）
    // 实际发布在 comm_thread_ 的定时器中完成（见下方）
    return hardware_interface::return_type::OK;
}
```

### 2.5 URDF hardware 配置

```xml
<!-- robot_description.urdf.xacro -->
<ros2_control name="micro_ros_arm" type="system">
  <hardware>
    <plugin>micro_ros_hardware_interface/MicroRosSystemInterface</plugin>
    <param name="joint_states_topic">/robot1/joint_states</param>
    <param name="joint_commands_topic">/robot1/joint_commands</param>
    <param name="connection_timeout_ms">5000</param>
    <param name="publish_rate_hz">500</param>
  </hardware>
  <joint name="joint1">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  <!-- joint2 ~ joint6 类似 -->
</ros2_control>
```

---

## 三、端到端延迟分析

### 3.1 延迟分解（Serial 传输，1 kHz 控制环）

```
MCU 硬件采样（编码器读取）
    ↓  ~10–50 μs（SPI/TIM 读取）
rclc Timer 回调（每 2ms）
    ↓  ~5 μs（rcl_publish → CDR 序列化）
XRCE Client 帧封装
    ↓  ~5 μs
Serial DMA 发送（921600 bps，240 字节 JointState）
    ↓  ~2.6 ms（240 × 10 bits / 921600 = 2.6 ms）  ← 主要瓶颈
UART 接收中断 / DMA 完成中断
    ↓  ~0.1 ms
micro-ROS Agent 接收并解析 XRCE 帧
    ↓  ~0.5–1 ms（Agent 处理 + DDS 发布）
DDS DataReader（CycloneDDS）触发回调
    ↓  ~0.1–0.5 ms（SHM 路径）
joint_state_buffer_.writeFromNonRT()
    ↓  ~1 μs
controller_manager::update() → read() → readFromRT()
    ↓  0 μs（无锁，瞬间）
controller 执行（计算控制命令）
─────────────────────────────────────────────────────
总延迟（Serial，921600 bps）：~4–6 ms（典型值）
```

### 3.2 延迟对比表

| 传输方式 | MCU→Agent | Agent→DDS | DDS→read() | 总延迟（典型） | 最大频率 |
|---------|----------|----------|-----------|--------------|---------|
| Serial 921600 bps | 2.6 ms（帧传输） | 0.5 ms | 0.1 ms | **3–5 ms** | **~200 Hz** |
| Serial 3 Mbps（特殊） | 0.65 ms | 0.5 ms | 0.1 ms | **1–2 ms** | **~500 Hz** |
| USB CDC Full Speed | 0.8 ms | 0.5 ms | 0.1 ms | **1.5–3 ms** | **~300 Hz** |
| UDP 有线以太网 | 0.5 ms | 0.5 ms | 0.1 ms | **1–2 ms** | **~500 Hz** |
| UDP WiFi（ESP32） | 2–15 ms | 0.5 ms | 0.1 ms | **3–20 ms** | **~50 Hz** |

> **关键结论**：Serial 921600 bps 最多支持约 200 Hz 的有效控制频率（考虑帧传输时间）。1 kHz ros2_control 控制环需要比传感器发布频率高，配合 ZOH（零阶保持）可正常工作，但控制性能会受限于传感器更新率。

### 3.3 Serial 传输帧时间计算

```
JointState（6 关节）序列化大小估算：
  Header（时间戳 + frame_id）：~30 字节
  name（6个字符串）：~100 字节
  position（6 × 8字节）：48 字节
  velocity（6 × 8字节）：48 字节
  effort（6 × 8字节）：48 字节
  XRCE 帧头：~10 字节
  总计：~284 字节

传输时间（921600 bps，10 bits/字节）：
  T = 284 × 10 / 921600 = 3.08 ms

∴ 921600 bps 下，JointState 最高发布频率 ≈ 1000/3.08 ≈ 324 Hz
  （考虑到 Agent 处理时间，实际建议 ≤ 200 Hz）
```

---

## 四、同步问题与处理策略

### 4.1 控制环频率 vs MCU 发布频率不一致

当 `controller_manager` 以 1 kHz 运行而 MCU 以 200 Hz 发布时，每 1ms 一次的 `read()` 调用中，有 4/5 的时间读取的是重复数据。

**ZOH（Zero Order Hold，零阶保持）**是默认策略：保持最近一次有效数据直到新数据到来。

```cpp
// ZOH 实现（已在 read() 中隐式实现）
// readFromRT() 在无新数据时返回上次的指针，即 ZOH 行为
const JointStates * states = joint_state_buffer_.readFromRT();
// 若 200ms 内无新数据，应触发错误处理
```

**超时检测**：

```cpp
hardware_interface::return_type MicroRosSystemInterface::read(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    const JointStates * states = joint_state_buffer_.readFromRT();

    // 检测数据新鲜度
    double age_ms = (time - states->stamp).seconds() * 1000.0;
    if (age_ms > 50.0) {  // 50ms 超时（MCU 200 Hz → 最大间隔 5ms，50ms 为 10倍容忍）
        RCLCPP_WARN_THROTTLE(
            comm_node_->get_logger(),
            *comm_node_->get_clock(), 1000,  // 限流：1Hz 打印
            "Stale joint states: %.1f ms", age_ms);
        // 可选：触发 hardware_interface 错误状态
        // return hardware_interface::return_type::ERROR;
    }
    // ... 正常读取 ...
}
```

### 4.2 外推（Linear Extrapolation）

对于速度已知的关节，可对位置进行简单外推：

```cpp
if (states && age_ms > 0 && age_ms < 20.0) {  // 仅短时外推（< 20ms）
    double dt = age_ms / 1000.0;
    for (size_t i = 0; i < num_joints_; ++i) {
        hw_positions_[i] = states->position[i] +
                           states->velocity[i] * dt;  // v * dt 外推
        hw_velocities_[i] = states->velocity[i];
    }
}
```

---

## 五、与 chassis_protocol 的对比分析

### 5.1 架构对比

| 维度 | chassis_protocol（现状） | micro-ROS 方案 |
|------|------------------------|---------------|
| **协议** | 自定义帧格式（frame_codec.cpp） | 标准 XRCE-DDS + ROS 2 消息 |
| **接口** | `ChassisHal::read()/write()` 直接返回数据 | `SystemInterface` 通过 RealtimeBuffer 异步通信 |
| **工具链** | 无标准调试工具 | `ros2 topic echo`、`rqt`、Foxglove 直接可视化 |
| **消息定义** | C++ 结构体（项目自定义） | `sensor_msgs/JointState`（ROS 2 标准） |
| **录制** | 需要自定义录制逻辑 | rosbag2 透明录制（MCU topic 通过 Agent 可见） |
| **延迟** | RS485 + frame_codec：约 1–3 ms（取决于波特率） | XRCE Serial：约 3–6 ms（协议开销更大） |
| **实时性** | `ChassisHal` 在 RT 线程中直接调用，无异步 | `SystemInterface::read()` 无锁，但数据来源为异步 DDS |
| **MCU 代码量** | 自定义帧编解码（frame_codec 移植） | rclc + XRCE-DDS（约 500 KB Flash） |

### 5.2 迁移成本矩阵

| 改造项 | 工作量（人天） | 难度 | 风险 |
|--------|------------|------|------|
| MCU 固件：添加 micro-ROS 支持（FreeRTOS 任务 + rclc） | 5–10 | 中 | 中（Flash 约束） |
| MCU 固件：将 frame_codec 替换为 rosidl 消息 | 3–5 | 低 | 低 |
| 主机端：`ChassisNode` 改为 `MicroRosSystemInterface` | 5–8 | 中 | 中（线程模型变化） |
| `DiffDriveController` / `MecanumController` 移至 ros2_control | 3–5 | 低 | 低 |
| 测试验证（延迟、丢包率、重连） | 5–10 | 高 | 高 |
| **合计** | **21–38 人天** | — | — |

---

## 六、EtherCAT + micro-ROS 混合架构

在高性能具身智能系统中，推荐**分层传输架构**：

```
┌─────────────────────────────────────────────────────────────────────────────┐
│  高速关节（EtherCAT 从站，< 1ms 周期）                                         │
│  ├─ 主关节（肩/肘/腕）：EtherCAT + etherlab/IgH                              │
│  │   └─ ethercat_hardware_interface → ros2_control SystemInterface          │
│  └─ 控制循环：1 kHz，确定性 < 100 μs                                          │
│                                                                             │
│  辅助传感器（micro-ROS，非关键路径）                                            │
│  ├─ IMU（200 Hz）：ESP32 → Serial → micro-ROS Agent → /imu                  │
│  ├─ 末端力矩传感器（500 Hz）：STM32 → Serial → /wrench                       │
│  ├─ 手指触觉传感器（50 Hz）：nRF5340 → Serial → /tactile                     │
│  └─ 电池监控（1 Hz）：ESP32 WiFi → /battery_state                            │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 6.1 典型部署配置

```yaml
# ros2_control.yaml — 混合硬件接口配置
controller_manager:
  ros__parameters:
    update_rate: 1000  # Hz，EtherCAT 驱动控制环

    # EtherCAT 高速关节控制器
    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    # micro-ROS 力矩传感器（只读，用于力控回路）
    force_sensor_broadcaster:
      type: force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster

micro_ros_arm:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
    # EtherCAT 接口（主关节）
    hardware_plugin: ethercat_hardware_interface/EthercatSystemInterface
    ethercat_master_idx: 0

force_sensor:
  ros__parameters:
    # micro-ROS 接口（末端力矩）
    hardware_plugin: micro_ros_hardware_interface/MicroRosSystemInterface
    joint_states_topic: /robot1/wrench
    connection_timeout_ms: 2000
```

### 6.2 EtherCAT vs micro-ROS 选型准则

| 场景 | EtherCAT | micro-ROS |
|------|---------|-----------|
| 高频关节控制（≥ 500 Hz，< 1ms 延迟） | ✅ 首选 | ❌ 延迟不满足 |
| 中频关节控制（≤ 200 Hz，5ms 延迟可接受） | ✅ 可用（成本较高） | ✅ 合适 |
| 辅助传感器（IMU/触觉/电池，≤ 100 Hz） | ❌ 过度设计 | ✅ 最合适 |
| 无线节点（ESP32/nRF） | ❌ 不支持无线 | ✅ UDP WiFi |
| ROS 2 标准化工具链兼容 | ⚠️ 需适配层 | ✅ 原生兼容 |
| 从站数量多（> 16 轴） | ✅ 扩展性好 | ⚠️ 多 Agent 管理复杂 |
