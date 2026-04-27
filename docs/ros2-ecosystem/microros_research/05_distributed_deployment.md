# 分布式部署架构

## 一、DDS Domain 隔离

### 1.1 ROS_DOMAIN_ID 分区策略

在多机器人或多组件系统中，`ROS_DOMAIN_ID` 是隔离 DDS 通信的首要手段：

```bash
# 取值范围：0–232（受 UDP 多播地址空间限制）
# 默认值：0

# 设置当前 shell 的 Domain
export ROS_DOMAIN_ID=42

# micro-ROS Agent 也需要同一 Domain
ROS_DOMAIN_ID=42 ros2 run micro_ros_agent micro_ros_agent serial \
    --dev /dev/ttyUSB0 --baudrate 921600
```

### 1.2 多机器人 Domain 规划

```
建议规划（车队/集群场景）：

Domain 0：开发调试（ROS_DOMAIN_ID=0，默认，可能冲突）
Domain 1–10：机器人 1–10（每台机器人独占一个 Domain）
  robot_1: ROS_DOMAIN_ID=1
  robot_2: ROS_DOMAIN_ID=2
  ...
Domain 50：监控/Fleet Manager（订阅所有机器人数据，需要 domain_bridge）
Domain 99：仿真环境（与实机完全隔离）
```

> **MCU 端注意**：micro-ROS Client 的 Domain ID 由 Agent 继承，MCU 代码本身不设置 Domain ID。因此同一 Agent 进程连接的所有 MCU 都在同一 Domain 下。

### 1.3 跨域桥（domain_bridge）

当需要跨 Domain 共享数据时（如 Fleet Manager 订阅各机器人的状态），使用 `domain_bridge` 包：

```yaml
# domain_bridge_config.yaml
topics:
  - topic: /joint_states
    type: sensor_msgs/msg/JointState
    from_domain: 1        # robot_1 的 Domain
    to_domain: 50         # Fleet Manager Domain
    queue_depth: 10

  - topic: /odom
    type: nav_msgs/msg/Odometry
    from_domain: 2        # robot_2 的 Domain
    to_domain: 50
    queue_depth: 10

  # 反向：从 Fleet Manager 发送导航目标给各机器人
  - topic: /goal_pose
    type: geometry_msgs/msg/PoseStamped
    from_domain: 50
    to_domain: 1
    queue_depth: 1
```

```bash
# 启动 domain_bridge
ros2 run domain_bridge domain_bridge \
    --config domain_bridge_config.yaml
```

---

## 二、micro-ROS Agent 部署

### 2.1 Docker 容器化部署

```dockerfile
# Dockerfile.micro_ros_agent
FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
    ros-humble-micro-ros-agent \
    && rm -rf /var/lib/apt/lists/*

# 健康检查：确认 Agent 进程存活
HEALTHCHECK --interval=10s --timeout=3s --retries=3 \
    CMD pgrep -x micro_ros_agent || exit 1

ENTRYPOINT ["/ros_entrypoint.sh"]
```

```yaml
# docker-compose.yml — micro-ROS Agent 服务
version: '3.8'

services:
  micro_ros_agent_joint:
    image: micro_ros_agent:humble
    restart: unless-stopped
    network_mode: host          # 必须：DDS 多播需要主机网络
    privileged: true            # 必须：访问 /dev/ttyUSB* 需要权限
    devices:
      - /dev/ttyUSB0:/dev/ttyUSB0   # 关节驱动 MCU
    environment:
      - ROS_DOMAIN_ID=1
    command: >
      ros2 run micro_ros_agent micro_ros_agent
      serial --dev /dev/ttyUSB0 --baudrate 921600

  micro_ros_agent_imu:
    image: micro_ros_agent:humble
    restart: unless-stopped
    network_mode: host
    privileged: true
    devices:
      - /dev/ttyUSB1:/dev/ttyUSB1   # IMU MCU
    environment:
      - ROS_DOMAIN_ID=1
    command: >
      ros2 run micro_ros_agent micro_ros_agent
      serial --dev /dev/ttyUSB1 --baudrate 115200

  micro_ros_agent_wifi:
    image: micro_ros_agent:humble
    restart: unless-stopped
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=1
    command: >
      ros2 run micro_ros_agent micro_ros_agent
      udp4 --port 8888
    ports:
      - "8888:8888/udp"
```

### 2.2 --net=host 的必要性

DDS 发现协议依赖 UDP **多播**（`239.255.0.x`），`--net=host` 确保：
1. Agent 能与主机上的其他 ROS 2 节点通过 DDS 通信（同 Domain）
2. MCU 的 UDP 流量（WiFi）能路由到 Agent 容器
3. Serial 设备（`/dev/ttyUSBx`）可访问

> **安全提示**：`--net=host` 会绕过 Docker 网络隔离。在生产环境中，配合 `sros2`（DDS Security）和防火墙规则限制 DDS 多播范围（见 Phase 11）。

### 2.3 udev 规则（固定设备路径）

```bash
# /etc/udev/rules.d/99-micro-ros.rules
# 根据 USB VID/PID 或序列号固定设备路径

# STM32 Virtual COM Port（关节驱动器 1）
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", \
    ATTRS{serial}=="JOINT_MCU_001", SYMLINK+="ttyJOINT0"

# CP2102 USB-UART（IMU）
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
    SYMLINK+="ttyIMU0"
```

```bash
# 重载 udev 规则
sudo udevadm control --reload-rules && sudo udevadm trigger

# Docker 中使用固定路径
devices:
  - /dev/ttyJOINT0:/dev/ttyUSB0
  - /dev/ttyIMU0:/dev/ttyUSB1
```

### 2.4 多 Agent 负载均衡

对于大量 WiFi MCU 节点，可部署多个 UDP Agent 实例分散负载：

```bash
# Agent 1：端口 8888，处理前 8 个 MCU
ROS_DOMAIN_ID=1 ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 &

# Agent 2：端口 8889，处理后 8 个 MCU
ROS_DOMAIN_ID=1 ros2 run micro_ros_agent micro_ros_agent udp4 --port 8889 &

# MCU 端：根据节点 ID 分配到不同 Agent 端口
// ESP32 固件中（main.cpp）
uint16_t agent_port = (node_id < 8) ? 8888 : 8889;
set_microros_wifi_transports(SSID, PWD, AGENT_IP, agent_port);
```

---

## 三、ROS 2 跨机器通信

### 3.1 同局域网 DDS 多播

默认配置下，同一局域网（且同 `ROS_DOMAIN_ID`）的所有机器自动发现彼此：

```bash
# 机器 A（主控）
export ROS_DOMAIN_ID=1
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0

# 机器 B（备份监控，同局域网）
export ROS_DOMAIN_ID=1
ros2 topic echo /joint_states  # 直接可见机器 A 的 MCU topic
```

### 3.2 NAT 穿透（CycloneDDS Peer Discovery）

当机器间有 NAT 或无多播路由时，配置 CycloneDDS 单播 Peer：

```xml
<!-- cyclonedds.xml — 跨网段单播配置 -->
<CycloneDDS>
  <Domain>
    <Id>1</Id>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>30</MaxAutoParticipantIndex>
      <!-- 禁用多播，改为单播 Peer -->
      <Peers>
        <Peer address="192.168.1.100"/>   <!-- 主控机器 -->
        <Peer address="192.168.1.101"/>   <!-- 备份机器 -->
        <Peer address="10.0.0.50"/>       <!-- 边缘服务器 -->
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

```bash
export CYCLONEDDS_URI=file:///path/to/cyclonedds.xml
```

### 3.3 ROS_LOCALHOST_ONLY

在单机开发环境中，避免 DDS 流量泄露到网络：

```bash
# 仅在本机通信（开发/测试时推荐）
export ROS_LOCALHOST_ONLY=1

# 注意：micro-ROS Agent 使用 Serial 时不受影响（Serial 不走网络）
# 但若 MCU 使用 UDP（WiFi），Agent 需要绑定到局域网 IP：
# 取消 ROS_LOCALHOST_ONLY 或配置 Agent 网卡绑定
```

---

## 四、Fleet Management

### 4.1 多机器人 Namespace 规划

```bash
# 推荐 namespace 结构：/<fleet>/<robot_id>/

# 机器人启动参数（launch 文件）
ros2 launch robot_bringup robot.launch.py \
    namespace:=/fleet_A/robot_1 \
    domain_id:=1

# micro-ROS MCU 端对应 namespace
rclc_node_init_default(&node, "joint_controller",
                        "fleet_A/robot_1",  // namespace
                        &support);
// 发布 topic：/fleet_A/robot_1/joint_states
```

```yaml
# fleet_config.yaml
fleet:
  name: "fleet_A"
  robots:
    - id: "robot_1"
      domain_id: 1
      agent_device: "/dev/ttyJOINT0"
      agent_port_udp: null
      namespace: "/fleet_A/robot_1"

    - id: "robot_2"
      domain_id: 2
      agent_device: null
      agent_port_udp: 8888
      namespace: "/fleet_A/robot_2"
```

### 4.2 远程 Lifecycle 管理

```bash
# 启动所有机器人节点的 lifecycle manager
ros2 launch nav2_lifecycle_manager lifecycle_manager.launch.py \
    node_names:="[
        '/fleet_A/robot_1/joint_controller',
        '/fleet_A/robot_1/imu_node',
        '/fleet_A/robot_2/joint_controller'
    ]" \
    autostart:=true

# 远程状态查询
ros2 lifecycle get /fleet_A/robot_1/joint_controller

# 批量激活
for robot in robot_1 robot_2 robot_3; do
    ros2 lifecycle set /fleet_A/$robot/joint_controller activate
done
```

---

## 五、边缘-云协同架构

### 5.1 三层部署模型

```
┌─────────────────────────────────────────────────────────────────────────────┐
│  云端（数据中心）                                                              │
│  ├─ 训练服务器：LeRobot/Isaac Lab 训练 VLA 模型                               │
│  ├─ OTA 服务器：固件 + ROS 2 节点更新（见 06_firmware_build.md）              │
│  └─ 远程监控：rosbag2 流式上传（按需，非实时）                                 │
│                         ↕ HTTPS / gRPC（非实时，数秒延迟可接受）               │
├─────────────────────────────────────────────────────────────────────────────┤
│  边缘服务器（机房 / 本地）                                                     │
│  ├─ VLA 推理：TensorRT / Triton Server → /vla_actions Topic                  │
│  ├─ Fleet Manager：多机器人调度、任务分配                                      │
│  ├─ 地图服务器：全局地图存储 + 更新                                            │
│  └─ ROS 2 Bridge（rosbridge_suite / DDS Peer）                              │
│                         ↕ 局域网 DDS（有线，< 1ms）                           │
├─────────────────────────────────────────────────────────────────────────────┤
│  机器人本地（Jetson Orin / x86）                                               │
│  ├─ ros2_control（1 kHz 控制环）                                             │
│  ├─ Nav2（本地导航，即使断网也能运行）                                          │
│  ├─ rosbag2 录制（本地 SSD，定期同步到云）                                    │
│  └─ micro-ROS Agent（桥接 MCU 层）                                           │
│                         ↕ XRCE-DDS（Serial/UDP）                            │
└─────────────────────────────────────────────────────────────────────────────┘
                            MCU 层（FreeRTOS/Zephyr）
```

### 5.2 边缘-本地通信配置

```xml
<!-- FastDDS 跨主机 XML 配置（主机端） -->
<profiles>
  <participant profile_name="robot_participant">
    <rtps>
      <builtin>
        <metatrafficUnicastLocatorList>
          <locator>
            <udpv4>
              <address>192.168.1.100</address>  <!-- 本机 IP -->
              <port>7412</port>
            </udpv4>
          </locator>
        </metatrafficUnicastLocatorList>
        <initialPeersList>
          <locator>
            <udpv4>
              <address>192.168.1.50</address>   <!-- 边缘服务器 IP -->
              <port>7412</port>
            </udpv4>
          </locator>
        </initialPeersList>
      </builtin>
    </rtps>
  </participant>
</profiles>
```

---

## 六、网络可靠性与重连策略

### 6.1 WiFi 断连重连

micro-ROS XRCE 协议内置会话超时和重连机制：

```c
// MCU 端：Supervisor 状态机处理断线重连（见 03_rclc_programming.md）
// 关键参数配置
#define AGENT_PING_TIMEOUT_MS  500    // ping 超时
#define AGENT_PING_ATTEMPTS    3      // 重试次数
#define RECONNECT_DELAY_MS     1000   // 重连间隔

// 重连时重新初始化所有 ROS 实体（micro-ROS 要求）
// 原因：Agent 重启后 session ID 失效，需重新建立
```

### 6.2 Agent 断线检测

```bash
# Agent 侧日志（-v4 详细模式）会输出会话超时信息
# [1687234567.123] [INFO] [Agent] Session timeout: client_id=1

# 配置 Agent 会话超时
# 环境变量（Agent 进程）
export RMW_UXRCE_SESSION_TIMEOUT_MS=5000   # 5秒无心跳 → 会话失效
```

### 6.3 Serial 断线检测

```c
// 在 micro-ROS Task 中周期性 ping Agent
// 若 Serial 断线（UART 错误），read() 回调返回 0，session 超时
void micro_ros_task(void *arg) {
    // ... 初始化 ...
    for (;;) {
        // ping_agent：发送 PING 子消息，等待 PONG
        rmw_ret_t ret = rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS,
                                             AGENT_PING_ATTEMPTS);
        if (ret != RMW_RET_OK) {
            // 断线：停止电机，等待重连
            set_motor_enable(false);
            destroy_ros_entities();
            vTaskDelay(pdMS_TO_TICKS(RECONNECT_DELAY_MS));
            continue;
        }
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    }
}
```

---

## 七、时间同步

### 7.1 MCU 时间戳问题

MCU 的本地时钟（`HAL_GetTick()`）与主机系统时钟不同步，导致消息时间戳不准确。

**micro-ROS 时间同步**（通过 Agent 的 NTP-like 机制）：

```c
// 与 Agent 同步时钟（建立连接后调用）
rmw_uros_sync_session(1000);  // timeout_ms = 1000

// 获取同步后的时间（纳秒，对应主机系统时间）
int64_t epoch_ns = rmw_uros_epoch_nanos();
int64_t epoch_ms = rmw_uros_epoch_millis();

// 填充消息时间戳
joint_state.header.stamp.sec     = epoch_ns / 1000000000LL;
joint_state.header.stamp.nanosec = epoch_ns % 1000000000LL;
```

### 7.2 多机器人 PTP 时间同步

当多台机器人需要时间融合（如 `robot_localization` 多机协同定位）时，主机端应配置精确时间同步：

```bash
# 安装 chrony（优于 ntpd）
sudo apt install chrony

# /etc/chrony.conf（机器人本地）
server 192.168.1.1 iburst prefer   # 边缘服务器作为 NTP 源
makestep 1.0 3                     # 允许跳步（初次同步）
rtcsync                            # 同步硬件时钟

# 验证同步状态
chronyc tracking
# Reference ID: 192.168.1.1
# RMS offset: 0.000123456 seconds
# Frequency: 5.234 ppm slow
```

```bash
# 企业级：IEEE 1588 PTP（微秒级同步，需要 PTP 硬件时间戳支持）
sudo apt install linuxptp

# ptp4l（PTP 从时钟）
sudo ptp4l -i eth0 -s -m

# phc2sys（将 PTP 时钟同步到系统时钟）
sudo phc2sys -s eth0 -c CLOCK_REALTIME -w -m
```

### 7.3 robot_localization 多机融合

```yaml
# ekf_global.yaml — 多机器人全局定位（边缘服务器端）
ekf_filter_node_global:
  ros__parameters:
    frequency: 30.0
    # 各机器人的 odom（经过时间同步的 header.stamp）
    odom0: /fleet_A/robot_1/odom
    odom0_config: [true, true, false, ...]
    odom1: /fleet_A/robot_2/odom
    odom1_config: [true, true, false, ...]
    # 全局 GPS（若有）
    pose0: /gps/filtered
    pose0_config: [true, true, false, ...]
```
