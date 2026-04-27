# 与本仓库已有组件的对接

## 一、chassis_protocol 迁移评估

### 1.1 现状分析

本仓库 `chassis_protocol` 包提供了一套**自定义串口协议栈**，架构如下：

```
chassis_protocol 当前架构：
  ros2_adapter/chassis_node.cpp      ← ROS 2 适配层（geometry_msgs/Twist 等）
       ↓
  chassis/chassis_hal.cpp            ← HAL 抽象（版本协商、差速/麦轮控制器）
  chassis/differential/              ← 差速驱动里程计计算
  chassis/mecanum/                   ← 麦轮控制
       ↓
  transport/rs485_transport.cpp      ← RS485 物理传输（POSIX fd + select()）
  transport/tcp_transport.cpp        ← TCP 传输（备用）
  transport/udp_transport.cpp        ← UDP 传输（备用）
       ↓
  frame/frame_codec.cpp              ← 自定义帧格式（帧头/长度/CRC-16/帧尾）
```

### 1.2 迁移方案 A：完全 micro-ROS 化（全面标准化）

**目标**：MCU 端运行 micro-ROS，发布/订阅标准 ROS 2 消息；主机端改为纯 ros2_control SystemInterface。

```
新架构（方案 A）：

MCU（STM32H7 FreeRTOS）：
  rclc Publisher  → /chassis/joint_states （sensor_msgs/JointState）
  rclc Publisher  → /chassis/odom         （nav_msgs/Odometry）
  rclc Subscriber ← /chassis/cmd_vel      （geometry_msgs/Twist）
  rclc Subscriber ← /chassis/joint_cmds   （std_msgs/Float64MultiArray）
       ↓ XRCE-DDS Serial 921600 bps
  micro-ROS Agent（主机）
       ↓ DDS（CycloneDDS）
主机（ROS 2 Humble）：
  MicroRosSystemInterface（SystemInterface 实现）
       ↓
  controller_manager → DiffDriveController / MecanumController
       ↓
  /cmd_vel → Nav2 → ...
```

**改造步骤**：

1. MCU 端移植 `differential_controller.cpp` 逻辑为 FreeRTOS 任务
2. MCU 端使用 rclc 发布 `/joint_states` 和 `/odom`（或直接发布 `/odom`）
3. 删除 `chassis_node.cpp`；新建 `MicroRosSystemInterface`
4. 在 URDF 中声明 `ros2_control` 硬件插件

```cpp
// 主机端：MicroRosSystemInterface（参考 04_ros2_control_integration.md）
// URDF 配置片段
<ros2_control name="chassis_micro_ros" type="system">
  <hardware>
    <plugin>micro_ros_hardware_interface/MicroRosSystemInterface</plugin>
    <param name="joint_states_topic">/chassis/joint_states</param>
    <param name="joint_commands_topic">/chassis/joint_commands</param>
  </hardware>
  <joint name="left_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="right_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

### 1.3 迁移方案 B：最小改造（保留协议层）

**目标**：保留 `chassis_protocol` 的 `frame_codec` 和传输层；仅在主机端将 `ChassisNode` 改造为标准 `ros2_control SystemInterface`，不触碰 MCU 端固件。

```
新架构（方案 B）：

MCU（原始固件，不变）：
  自定义帧协议（frame_codec）通过 RS485 与主机通信
       ↓ RS485 自定义帧协议（不变）
主机（改造 chassis_node.cpp）：
  ChassisHal（保留，读写数据）
       ↓
  ChassisSystemInterface（新增：SystemInterface 包装 ChassisHal）
       ↓
  controller_manager → DiffDriveController
```

```cpp
// 新增：ChassisSystemInterface（最小改造方案）
class ChassisSystemInterface : public hardware_interface::SystemInterface {
public:
    hardware_interface::return_type read(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override
    {
        // 直接调用现有 ChassisHal（无需 micro-ROS）
        auto state = chassis_hal_->read();
        hw_left_wheel_pos_  = state.left_encoder;
        hw_right_wheel_pos_ = state.right_encoder;
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type write(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override
    {
        chassis_hal_->write({hw_left_wheel_cmd_, hw_right_wheel_cmd_});
        return hardware_interface::return_type::OK;
    }

private:
    std::unique_ptr<ChassisHal> chassis_hal_;
    double hw_left_wheel_pos_{0}, hw_right_wheel_pos_{0};
    double hw_left_wheel_cmd_{0}, hw_right_wheel_cmd_{0};
};
```

### 1.4 迁移成本对比矩阵

| 评估维度 | 方案 A（完全 micro-ROS） | 方案 B（最小改造） | 现状（不改造） |
|---------|----------------------|-----------------|--------------|
| **MCU 开发量** | 高（需移植到 FreeRTOS + rclc） | 无 | 无 |
| **主机端开发量** | 高（新建 SystemInterface + 删除 ChassisNode） | 中（包装 ChassisHal） | 无 |
| **总开发量（人天）** | 21–38 天 | 5–10 天 | 0 |
| **实时性** | 中（异步 DDS，3–6ms 延迟） | 高（直接调用，1–3ms） | 高（1–3ms） |
| **可维护性** | 高（ROS 2 标准，社区工具支持） | 中（混合架构，双协议） | 低（完全自定义） |
| **调试难度** | 低（`ros2 topic echo`、Foxglove） | 中（需要自定义调试工具） | 高（需要串口协议分析器） |
| **消息录制** | 原生（rosbag2 自动录制） | 原生（ros2_control 标准接口） | 需要自定义 |
| **多机扩展** | 高（namespace + domain_id） | 中（需改造 ChassisHal） | 低（需大量重构） |
| **Flash 成本** | MCU Flash +500KB（micro-ROS 库） | 无 | 无 |
| **延迟增加** | +2–4ms（XRCE transport） | 无 | 基线 |

**推荐决策**：
- **短期（< 3 个月）**：采用方案 B，快速对接 ros2_control 工具链，验证控制效果
- **中期（3–12 个月）**：在 MCU 固件下一版本时迁移到方案 A，获得完整标准化生态
- **新项目**：直接采用方案 A（micro-ROS 原生设计）

---

## 二、Phase 7 RT 对接

### 2.1 micro-ROS Agent 进程 CPU 隔离

根据 Phase 7 研究（`realtime_research/02_ros2_realtime.md`），`micro_ros_agent` 进程属于**非 RT 进程**，应分配到非 RT CPU 核：

```bash
# CPU 隔离配置（8 核示例，参照 Phase 7 分配方案）
# Core 2-3：controller_manager（RT，SCHED_FIFO prio 90）
# Core 4-5：DDS 线程（较高优先级）
# Core 6：micro-ROS Agent（非 RT，Core 6）
# Core 7：VLA 推理（非 RT，Core 7）

# 将 micro-ROS Agent 绑定到 Core 6
taskset -c 6 ros2 run micro_ros_agent micro_ros_agent \
    serial --dev /dev/ttyUSB0 --baudrate 921600

# 或通过 cpuset cgroup（更严格隔离）
mkdir -p /sys/fs/cgroup/cpuset/micro_ros
echo "6" > /sys/fs/cgroup/cpuset/micro_ros/cpuset.cpus
echo "0" > /sys/fs/cgroup/cpuset/micro_ros/cpuset.mems
echo $AGENT_PID > /sys/fs/cgroup/cpuset/micro_ros/tasks
```

```yaml
# docker-compose.yml（容器化 Agent CPU 隔离）
services:
  micro_ros_agent:
    cpuset: "6"          # 绑定 Core 6
    cpu_shares: 512      # 相对 CPU 权重（SCHED_OTHER）
    restart: unless-stopped
    network_mode: host
    ...
```

### 2.2 Agent 作为 DDS 代理对控制环延迟的影响

```
controller_manager RT 线程（Core 2-3，SCHED_FIFO prio 90）：
  update() → read() → readFromRT() [无锁，< 1μs]
                ↑
  通信线程（Core 6，SCHED_OTHER）：
  DDS DataReader 回调 → writeFromNonRT() [无锁，< 1μs]
                ↑
  micro-ROS Agent（Core 6）：
  接收 Serial 帧 → 解析 XRCE → DDS DataWriter::write()
  延迟：~0.5–2ms（Core 6 非 RT，可能被其他进程抢占）
```

**关键结论**：Agent 运行在非 RT 核上，其处理延迟（0.5–2ms）直接加入 MCU→Controller 的端到端延迟链。由于 `read()` 使用 `RealtimeBuffer::readFromRT()`（ZOH），RT 线程永不因 Agent 延迟而阻塞，但数据新鲜度受影响。

**优化方案**：将 Agent 绑定到与 DDS 接收线程相同的核（Core 4-5），减少跨核唤醒延迟：

```bash
# Agent + DDS 接收线程都在 Core 4-5
taskset -c 4,5 ros2 run micro_ros_agent micro_ros_agent serial ...
```

---

## 三、Phase 6 数据录制对接

### 3.1 micro-ROS Topic 的透明录制

由于 MCU 端 topic 通过 Agent 桥接后成为标准 DDS topic，rosbag2 可**无感知地**录制 MCU 数据：

```bash
# 录制所有 MCU 相关 topic（通过 Agent 可见）
ros2 bag record \
    /robot1/joint_states \          # MCU 发布（500 Hz）
    /robot1/imu/data \              # IMU MCU（200 Hz）
    /robot1/wrench \                # 力矩 MCU（500 Hz）
    /robot1/battery_state \         # 电池 MCU（1 Hz）
    /robot1/joint_commands \        # 主机发送给 MCU 的命令
    --storage mcap \
    --compression-mode file \
    --compression-format zstd \
    -o robot1_session_$(date +%Y%m%d_%H%M%S)
```

### 3.2 高频录制 QoS 配置

MCU 端使用 BEST_EFFORT QoS，rosbag2 订阅者也需匹配：

```yaml
# rosbag2_record_config.yaml
record:
  topics:
    - topic: /robot1/joint_states
      type: sensor_msgs/msg/JointState
      # 匹配 MCU 端的 BEST_EFFORT QoS
      qos_profile:
        reliability: best_effort
        durability: volatile
        depth: 100         # 较大缓冲，避免高频录制丢帧

    - topic: /robot1/imu/data
      type: sensor_msgs/msg/Imu
      qos_profile:
        reliability: best_effort
        durability: volatile
        depth: 200         # 200 Hz，需要更大缓冲
```

```bash
# 使用 QoS 覆盖文件录制
ros2 bag record --qos-profile-overrides-path rosbag2_qos.yaml \
    -a  # 录制所有 topic
```

### 3.3 IMU / 关节状态高频录制的磁盘开销

```
估算（参考 Phase 6 rosbag2_research/07_integration.md 基线）：

JointState（6 关节）@ 500 Hz：
  ~240 B/帧 × 500 Hz = 120 KB/s = 7.2 MB/min

IMU（sensor_msgs/Imu）@ 200 Hz：
  ~112 B/帧 × 200 Hz = 22.4 KB/s = 1.3 MB/min

WrenchStamped @ 500 Hz：
  ~56 B/帧 × 500 Hz = 28 KB/s = 1.7 MB/min

合计（MCU topic）：~10 MB/min ≈ 600 MB/h

加上视觉数据（640×480 压缩 @ 30Hz）：~130 MB/s（Phase 6 基线）
→ MCU topic 相比视觉数据占比极小，可接受
```

---

## 四、整体分布式架构图

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│  云端（数据中心）                                                                  │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │  训练服务器：Isaac Lab / LeRobot                                          │   │
│  │  OTA 分发服务器：GitHub Releases + ESP-IDF OTA / DFU                     │   │
│  │  远程监控：Foxglove Cloud / 自建 rosbag2 存储                            │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
│                    ↕ HTTPS（非实时，按需）                                        │
├─────────────────────────────────────────────────────────────────────────────────┤
│  边缘服务器（机房/本地）                                                           │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │  VLA 推理（Triton / TensorRT）→ /vla_actions                            │   │
│  │  Fleet Manager → /robot_N/goal_pose                                    │   │
│  │  domain_bridge（Domain N → Domain 50）                                  │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
│                    ↕ 有线以太网 DDS（< 1ms）                                      │
├─────────────────────────────────────────────────────────────────────────────────┤
│  机器人主机（Jetson Orin，ROS_DOMAIN_ID=N）                                        │
│  ┌────────────────────────┐  ┌───────────────────────────────────────────┐    │
│  │  RT 核（Core 2-3）      │  │  非 RT 核（Core 4-7）                      │    │
│  │  controller_manager    │  │  ├─ micro-ROS Agent（Core 6）              │    │
│  │   1 kHz，SCHED_FIFO   │  │  │   ├─ Serial Agent → /dev/ttyUSB0       │    │
│  │  MicroRosSystemInterface│  │  │   └─ UDP Agent → :8888               │    │
│  │   read() ← RT Buffer  │  │  ├─ Nav2（Core 7）                         │    │
│  │   write() → RT Buffer │  │  ├─ VLA 客户端（Core 7）                   │    │
│  │  DiffDriveController  │  │  ├─ rosbag2 录制（Core 6）                 │    │
│  │  JTC / ImpedanceCtrl  │  │  └─ DDS 线程（Core 4-5）                  │    │
│  └────────────────────────┘  └───────────────────────────────────────────┘    │
│                    ↕ iceoryx2 SHM（RT 内核间 IPC）        ↕ XRCE-DDS            │
├─────────────────────────────────────────────────────────────────────────────────┤
│  MCU 层（FreeRTOS/Zephyr）                                                       │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────────────────┐  │
│  │ STM32H7          │  │ ESP32-S3         │  │ nRF5340                      │  │
│  │ 关节驱动（6轴）   │  │ IMU + WiFi       │  │ 触觉/力矩传感器               │  │
│  │ FreeRTOS         │  │ FreeRTOS + UDP   │  │ Zephyr + Serial              │  │
│  │ 500 Hz pub       │  │ 200 Hz pub       │  │ 500 Hz pub                   │  │
│  │ Serial 921600bps │  │ UDP WiFi :8888   │  │ Serial 921600bps             │  │
│  └──────────────────┘  └──────────────────┘  └──────────────────────────────┘  │
│                    ↕ RS485 / WiFi / UART                                         │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │  物理硬件层                                                               │   │
│  │  编码器 / PWM 驱动器 / IMU（ICM-42688） / ATI 力矩传感器 / 触觉阵列      │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────────┘
```

---

## 五、选型决策矩阵

### 5.1 micro-ROS vs 自定义串口协议 vs EtherCAT

| 维度 | micro-ROS | 自定义串口协议（chassis_protocol） | EtherCAT |
|------|-----------|--------------------------------|---------|
| **协议标准化** | ✅ DDS/XRCE 国际标准 | ❌ 完全自定义 | ✅ IEC 61158 标准 |
| **最高控制频率** | ~500 Hz（921600 Serial） | ~1 kHz（RS485 轻量帧） | 4–10 kHz |
| **端到端延迟** | 3–6 ms（Serial） | 1–3 ms | < 0.1 ms |
| **延迟确定性** | 中（DDS 非 RT） | 中（POSIX select 非 RT） | ✅ 极高（微秒级抖动） |
| **工具链生态** | ✅ ros2 cli / Foxglove / rosbag2 原生支持 | ❌ 需自建工具 | ⚠️ EtherCAT 专用工具 |
| **MCU Flash 占用** | 中（~500 KB） | 小（< 50 KB） | 从站 IP Core（FPGA/ASIC） |
| **MCU RAM 占用** | 中（32–64 KB） | 小（< 10 KB） | 极小（EtherCAT 从站 IP） |
| **有线/无线** | 两者均支持 | 仅有线 | 仅有线（EtherCAT 不支持无线） |
| **开发难度** | 中（rclc API 相对标准） | 低（自定义可完全控制） | 高（EtherCAT 主从站配置复杂） |
| **成本** | 低（软件方案） | 低（软件方案） | 高（专用 IC/模块） |
| **多机器人扩展** | ✅ namespace + domain_id | 需大量改造 | ✅ 从站链路拓扑 |
| **数据录制** | ✅ 原生 rosbag2 | 需自定义 | 需 EtherCAT → ROS 2 适配层 |
| **场景** | 中频传感器 + 标准化优先 | 快速原型 + 资源极限 | 高频精密控制 |

### 5.2 决策流程图

```
                    需要控制频率 > 1 kHz 或延迟 < 1 ms？
                              ↓
                    ┌────────────────────┐
                    │  是（力控 4kHz等）  │──→ EtherCAT（高速关节）
                    └────────────────────┘    + micro-ROS（辅助传感器）
                    ↓ 否
                    是否需要 ROS 2 标准工具链（rosbag2/Foxglove）？
                              ↓
                    ┌────────────────────┐
                    │  是                │──→ micro-ROS（推荐）
                    └────────────────────┘
                    ↓ 否
                    MCU Flash < 512 KB 或时间紧迫（< 1 个月）？
                              ↓
                    ┌────────────────────┐
                    │  是                │──→ 保留 chassis_protocol
                    └────────────────────┘    + 方案 B 最小改造
                    ↓ 否
                        → micro-ROS 方案 A（完全标准化）
```

### 5.3 本仓库当前阶段推荐

基于 `chassis_protocol` 已有 RS485 物理层（`rs485_transport.cpp`）和 `ChassisHal` 抽象：

| 近期目标 | 推荐方案 | 理由 |
|---------|---------|------|
| 快速验证 ros2_control 集成 | **方案 B**（包装 ChassisHal） | 2 周内可完成，风险最低 |
| 数据录制（Phase 6 数据飞轮） | **方案 B** 已满足（ros2_control 标准接口） | — |
| 多机器人扩展（fleet） | **方案 A**（micro-ROS） | namespace + domain_id 原生支持 |
| 生产部署（> 100 台） | **方案 A** + EtherCAT 混合 | 长期维护成本最低 |
