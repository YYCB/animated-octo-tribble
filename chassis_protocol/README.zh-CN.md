# chassis_protocol（底盘通信协议 HAL）

> [English README](README.md)

面向差速底盘和麦轮底盘的 ROS2 底盘通信协议硬件抽象层（HAL）。
核心理念：**"协议即代码"**——协议变更即 PR，杜绝文档对齐的反复沟通。

---

## 整体架构

```
┌─────────────────────────────────────────────────────────────┐
│                      ROS2 应用层                             │
│   /cmd_vel  /odom  /battery_state  /chassis_status  /diag   │
└───────────────────────┬─────────────────────────────────────┘
                        │ ROS2 话题
┌───────────────────────▼─────────────────────────────────────┐
│               chassis_ros2::ChassisNode                      │
│           (ros2_adapter/chassis_node.hpp/.cpp)               │
└───────────────────────┬─────────────────────────────────────┘
                        │ IChassisController
┌───────────────────────▼─────────────────────────────────────┐
│   DifferentialController  │  MecanumController              │
│   （夹紧 vy=0）           │  （全向运动）                    │
├───────────────────────────┴─────────────────────────────────┤
│                      ChassisHal                              │
│          帧编解码 / 接收分发 / 断线重连循环                    │
├─────────────────────────────────────────────────────────────┤
│   VersionNegotiator  │  StreamDecoder  │  FrameCodec         │
│   （握手状态机）      │  （字节流同步）  │  （CRC16-CCITT）   │
└──────────────────────┬──────────────────────────────────────┘
                       │ ITransport（传输后端抽象）
        ┌──────────────┼──────────────┐
        ▼              ▼              ▼
   TcpTransport   UdpTransport  Rs485Transport
   （POSIX TCP）  （POSIX UDP）  （termios 8N1）
```

**分层原则**：
- **HAL 层**（ChassisHal 及以下）——纯 C++17，无 ROS 依赖，可单独集成到非 ROS 系统
- **ROS2 适配层**（ChassisNode）——负责 ROS 话题订发、参数读取，不感知传输细节
- **传输层**（ITransport）——依赖注入，切换 TCP/UDP/RS485 对上层零修改

---

## 帧格式

```
[MAGIC:2B=0xAA55][VERSION:1B][CMD_ID:1B][LEN:2B][PAYLOAD:NB][CRC16:2B]
```

| 字段    | 大小 | 说明                                         |
|---------|------|----------------------------------------------|
| MAGIC   | 2B   | 固定魔数 `0xAA 0x55`，用于 RS485 流同步       |
| VERSION | 1B   | 协议大版本（当前：`0x01`）                   |
| CMD_ID  | 1B   | 指令标识（见下表）                           |
| LEN     | 2B   | Payload 长度（小端序）                       |
| PAYLOAD | NB   | 指令数据体                                   |
| CRC16   | 2B   | CRC16-CCITT，校验范围：VERSION+CMD_ID+LEN+PAYLOAD |

### CMD_ID 列表

| ID     | 名称             | 方向         |
|--------|------------------|-------------|
| `0x01` | HANDSHAKE_REQ    | 上位机 → MCU |
| `0x02` | HANDSHAKE_RESP   | MCU → 上位机 |
| `0x10` | VELOCITY_CMD     | 上位机 → MCU |
| `0x11` | LIGHT_CMD        | 上位机 → MCU |
| `0x12` | EMERGENCY_STOP   | 上位机 → MCU |
| `0x20` | CHASSIS_STATUS   | MCU → 上位机 |
| `0x21` | ODOMETRY         | MCU → 上位机 |
| `0x22` | BATTERY_INFO     | MCU → 上位机 |
| `0xFF` | ACK              | 双向         |
| `0x30–0x3F` | *预留：足式指令*  | — |
| `0x40–0x4F` | *预留：足轮混合指令* | — |

---

## 快速开始

### 编译

```bash
cd ~/ros2_ws/src
git clone <repo-url> chassis_protocol
cd ~/ros2_ws
colcon build --packages-select chassis_protocol
source install/setup.bash
```

### 启动节点

TCP 差速底盘：

```bash
ros2 run chassis_protocol chassis_node \
  --ros-args \
  -p transport_type:=tcp \
  -p host:=192.168.1.100 \
  -p port:=8899 \
  -p chassis_type:=differential
```

RS485 麦轮底盘：

```bash
ros2 run chassis_protocol chassis_node \
  --ros-args \
  -p transport_type:=rs485 \
  -p device:=/dev/ttyUSB0 \
  -p baud_rate:=115200 \
  -p chassis_type:=mecanum
```

### 运行测试

```bash
colcon test --packages-select chassis_protocol
colcon test-result --verbose
```

---

## ROS2 参数

| 参数名                | 类型   | 默认值          | 说明                          |
|-----------------------|--------|-----------------|-------------------------------|
| `transport_type`      | string | `"tcp"`         | `tcp`、`udp` 或 `rs485`       |
| `host`                | string | `192.168.1.100` | 远端地址（TCP/UDP）           |
| `port`                | int    | `8899`          | 远端端口（TCP/UDP）           |
| `device`              | string | `/dev/ttyUSB0`  | 串口设备路径（RS485）         |
| `baud_rate`           | int    | `115200`        | 波特率（RS485）               |
| `chassis_type`        | string | `"differential"`| `differential` 或 `mecanum`   |
| `reconnect_interval_s`| double | `5.0`           | 重连检测间隔（秒）            |
| `frame_id`            | string | `"odom"`        | 里程计帧 ID                   |
| `child_frame_id`      | string | `"base_link"`   | 里程计子帧 ID                 |

---

## ROS2 话题

| 话题             | 消息类型                                    | 方向   |
|------------------|--------------------------------------------|--------|
| `/cmd_vel`       | `geometry_msgs/msg/Twist`                  | 订阅   |
| `/odom`          | `nav_msgs/msg/Odometry`                    | 发布   |
| `/battery_state` | `sensor_msgs/msg/BatteryState`             | 发布   |
| `/chassis_status`| `std_msgs/msg/String`（JSON）              | 发布   |
| `/diagnostics`   | `diagnostic_msgs/msg/DiagnosticArray`      | 发布   |

---

## 版本协商握手流程

```
上位机                                       MCU（Server）
   │                                              │
   │──── TCP connect ────────────────────────────►│
   │                                              │
   │──── HANDSHAKE_REQ ──────────────────────────►│
   │  { protocol_version=1,                       │
   │    supported_cmds=[0x10,0x11,0x12],          │
   │    config_hash="<proto MD5>" }               │
   │                                              │
   │◄─── HANDSHAKE_RESP ─────────────────────────│
   │  { accepted_version=1,                       │
   │    chassis_type="differential",              │
   │    config_hash="<proto MD5>",                │
   │    accepted=true }                           │
   │                                              │
   │═══════════ 正常通信（VELOCITY_CMD 等）══════════│
```

- 若 `config_hash` 不一致，双方在日志中打印警告，提前暴露协议版本不对齐问题
- 握手超时（2 秒）最多重试 3 次，失败后标记为 `FAILED` 状态并停止发送

---

## 自动重连

`ChassisHal` 后台线程（`reconnectLoop`）：
- 持续监控 `isConnected()` 和 `isNegotiated()`
- 断线后以指数退避（500ms → 1s → 2s … 最大 30s）自动重连并重新握手
- 重连期间上层调用 `sendVelocity` 会返回 `false`，不会崩溃

---

## 协议变更流程

**改进前**：Word/Excel 文档 → 聊天对齐 → 双方各自手改解析代码 → 联调才知道对不对

**改进后**：

```
修改 proto/chassis.proto
         │
         ▼
     开 Pull Request
         │
         ▼
   双方 Review & Approve
         │
         ▼
  protoc 生成 C++ 代码（generated/cpp/）
  nanopb 生成 MCU 代码（generated/nanopb/）
         │
         ▼
  离线回放测试（tests/fixtures/）
         │
         ▼
      合并上线
```

`.proto` 文件是**唯一事实来源（Single Source of Truth）**，协议变更有 Git 历史可追溯。

---

## 扩展指南

### 新增底盘构型（以足式机器人为例）

```
当前：                          扩展后：
IChassisController              IChassisController
  ├── DifferentialController      ├── DifferentialController
  └── MecanumController           ├── MecanumController
                                  └── ILeggedController (新接口)
                                        ├── QuadrupedController (四足)
                                        ├── BipedController     (双足)
                                        └── LegWheelController  (足轮混合)
```

**步骤：**

1. 在 `proto/chassis.proto` 中添加新消息（`GaitCmd`、`StanceCmd`、`JointCmd`）
2. 在 CMD_ID 空间中分配新 ID（已预留 `0x30–0x4F` 给足式）
3. 新建 `chassis/legged/ILeggedController` 接口，继承 `IChassisController`
4. 实现 `chassis/legged/QuadrupedController`（继承 `ChassisHal`）
5. 在 `ChassisNode::initController()` 中注册新构型
6. **现有代码零修改**

### 新增传输后端

1. 实现 `ITransport` 接口（6 个方法：send / setReceiveCallback / connect / disconnect / isConnected / getLastError）
2. 在 `TransportType` 枚举中添加新值
3. 在 `ITransport::create()` 中添加 `case`

### 控制指令安全收口与优先级

对于需要多来源控制指令的场景（遥控 + 自主导航 + 远程调试），推荐在 `ros2_adapter/` 层引入 `CommandArbiter`：

| 优先级 | 来源               | 话题              | 心跳超时 |
|--------|--------------------|-------------------|---------|
| P0     | 急停（任意来源）   | `/e_stop`         | 永久（需显式解除）|
| P1     | 安全守护（防碰）   | `/safety_cmd_vel` | 50 ms   |
| P2     | 遥控 / 手柄        | `/joy_cmd_vel`    | 200 ms  |
| P3     | 自主导航           | `/nav_cmd_vel`    | 500 ms  |
| P4     | 远程调试           | `/debug_cmd_vel`  | 1 s     |

**仲裁规则：**
- 高优先级立即**抢占**低优先级，不等队列
- 来源超时未续命 → 自动降到下一有效优先级
- 急停只能**显式解除**，防止误触后自动恢复
- 当前生效来源在 `/chassis_status` 中透明上报

这部分**不放在 HAL 层**，因为 HAL 只关心"怎么发"，不关心"该不该发"。

---

## Payload 帧结构

所有多字节整数均为**小端序**。

### VELOCITY_CMD (0x10) — 36 字节

| 偏移 | 大小 | 字段          |
|------|------|---------------|
| 0    | 4    | seq（帧序号） |
| 4    | 8    | vx（m/s）     |
| 12   | 8    | vy（m/s）     |
| 20   | 8    | omega（rad/s）|
| 28   | 8    | timestamp_us  |

### LIGHT_CMD (0x11) — 12 字节

| 偏移 | 大小 | 字段                       |
|------|------|----------------------------|
| 0    | 1    | mode（0=关/1=常亮/2=闪烁/3=呼吸）|
| 1    | 1    | r                          |
| 2    | 1    | g                          |
| 3    | 1    | b                          |
| 4    | 8    | blink_hz（闪烁频率）        |

### ODOMETRY (0x21) — 668 字节

| 偏移 | 大小 | 字段                    |
|------|------|-------------------------|
| 0    | 4    | seq                     |
| 4    | 8    | timestamp_us            |
| 12   | 8×3  | x, y, z（位置）         |
| 36   | 8×4  | qx, qy, qz, qw（姿态） |
| 68   | 8×3  | vx, vy, omega（速度）   |
| 92   | 288  | cov_pose（36×double）   |
| 380  | 288  | cov_twist（36×double）  |

### BATTERY_INFO (0x22) — 变长

| 偏移 | 大小         | 字段          |
|------|--------------|---------------|
| 0    | 8            | voltage       |
| 8    | 8            | current       |
| 16   | 8            | percentage    |
| 24   | 8            | temperature   |
| 32   | 1            | is_charging   |
| 33   | 1            | num_cells     |
| 34   | num_cells×8  | cell_voltages |

---

## 与 MCU 工程师的协作边界

| 契约内容       | 载体                     | 维护方     |
|----------------|--------------------------|-----------|
| 帧格式         | `frame/frame_codec.hpp`  | 双方共建  |
| 消息定义       | `proto/chassis.proto`    | 双方共建（PR Review）|
| MCU 实现规范   | `generated/nanopb/` + README | MCU 侧 |
| 联调测试用例   | `tests/fixtures/*.bin`   | 上位机侧  |
| 协议版本日志   | `CHANGELOG.md`           | 双方共建  |

**MCU 工程师**：只需关心 `chassis.proto` 和帧格式，不需要理解 ROS。  
**上位机工程师**：只需关心 `ITransport` 和 `IChassisController`，不需要理解 MCU 内部。

---

## 许可证

Apache-2.0，详见 `package.xml`。
