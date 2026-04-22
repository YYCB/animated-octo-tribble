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
| `0x13` | WHEEL_CMD        | 上位机 → MCU |
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

### 控制模式概览

协议支持两种下行指令模式，可通过 `setControlMode()` 在运行时切换：

| 模式 | CMD_ID | 分解位置 | 适用场景 |
|------|--------|----------|----------|
| `BODY_VELOCITY` | `0x10` | MCU（默认） | MCU 是自研控制板，具备 FPU；需跨底盘构型统一协议 |
| `WHEEL_RPM`     | `0x13` | 上位机 | MCU 是成品 FOC 控制器（ODrive/VESC）；需在上位机实时调参 |

两种模式在 `IChassisController` 接口层**透明**：上层代码始终调用 `sendVelocity(vx, vy, omega)`，底层控制器根据模式自动选择发哪帧。也可以绕过运动学直接调用 `sendWheelCmd({w0, w1, ...})` 精确控制各轮。

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

### WHEEL_CMD (0x13) — 13 + 8×N 字节（上位机逆运动学模式）

上位机完成逆运动学分解后，以单帧原子方式下发所有车轮的目标角速度。MCU 只需为各轮执行 PID 控速，无需运动学知识。

| 偏移        | 大小    | 字段                                  |
|-------------|---------|---------------------------------------|
| 0           | 4       | seq（帧序号）                          |
| 4           | 8       | timestamp_us                           |
| 12          | 1       | n_wheels（车轮数量）                   |
| 13          | 8×N     | wheel_rads[0..N-1]（各轮 rad/s，小端 double）|

**轮序约定：**

| 底盘类型   | 轮序                        |
|------------|-----------------------------|
| 差速（2轮）| [0] 左轮，[1] 右轮          |
| 麦轮（4轮）| [0] FL，[1] FR，[2] RL，[3] RR |

> **正值 = 前进方向旋转**（差速左轮逆时针从俯视看为前进）。MCU 实现应按此约定转换 PWM 方向。

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

## 架构决策记录（FAQ）

本章节记录开发过程中反复出现的设计问题及其决策依据，供团队成员快速查阅。

---

### Q1：MCU 在 TCP 连接中应当充当服务端（Server）还是客户端（Client）？

**结论：MCU = Server，上位机主动发起连接。**

**理由：**

1. **底盘是独立销售单元**  
   底盘出厂时带固定 IP 和监听端口，任何满足协议的上位机（ROS 机器、调试 PC、云端管理系统）都可以直接连入，无需在 MCU 固件中写死上位机地址。更换上位机或同时接多台上位机时，MCU 固件**零修改**。

2. **上位机具备更强的重连和路由能力**  
   ROS 节点（ChassisHal 的 `reconnectLoop`）可以做指数退避重连、日志记录、话题状态发布；MCU 资源有限，让 MCU 主动重连会显著增加固件复杂度。

3. **物理网络通常以底盘为固定点**  
   移动机器人上，底盘的网络接口地址由路由器 DHCP 保留分配，地址稳定；上位机（板卡）地址相对灵活，由 Client 角色发起连接更自然。

4. **与 RS485 传输层行为对称**  
   RS485 天然没有"谁主动"的概念，上位机通过轮询或主动写来驱动通信。TCP 模型中 MCU=Server 与此语义保持一致：底盘等待指令，上位机主动推送。

**握手流程中的体现（见"版本协商握手流程"章节）：**
```
上位机                    MCU（Server）
  │── TCP connect ──────►│    上位机主动发起
  │── HANDSHAKE_REQ ────►│
  │◄─ HANDSHAKE_RESP ────│
  │═══════ 正常通信 ══════│
```

---

### Q2：MCU 是否应把所有反馈状态塞进一个大帧全量发送？

**结论：不合适。应按数据类型分帧，以各自合理的频率独立发送。**

#### 问题的核心：各类数据更新频率差异极大

| 数据类型         | 合理发送频率  | 原因                             |
|-----------------|-------------|----------------------------------|
| ODOMETRY (0x21) | 50–200 Hz   | 运动控制/导航需要高频位姿反馈     |
| CHASSIS_STATUS (0x20) | 1 Hz 心跳 + 事件触发 | 故障码不是每帧都变  |
| BATTERY_INFO (0x22) | 0.5–2 Hz  | 电压变化极慢，高频发送无意义      |

如果把三者打包成一个大帧以 100 Hz 发送：

- 电池数据每秒被白白发送 100 次，实际有效变化不超过 2 次
- 无故障时每帧都携带 `error_code=0, error_msg=""`，占字节纯属浪费
- 带宽估算：Odometry 668B + Status ~30B + Battery ~50B ≈ 748B，以 100 Hz 发送 = **74.8 KB/s**，超出 RS485 @ 115200 bps 的物理上限（≈14.4 KB/s）**5 倍以上**

#### 分帧发送的优势

**① 延迟更低**  
大帧须等所有子数据都就绪后才能打包：最慢的那个决定整体延迟。分帧后里程计就绪即立刻发出，不阻塞在电池数据上。

**② 丢帧损失更小**  
RS485 / UDP 存在丢包。大帧丢一次 = 所有数据同时缺失；分帧丢一次 = 只缺该类数据，其他正常。

**③ MCU RAM 压力更小**  
大帧需要在内存中将所有数据拼好再发，分帧各自用小 buffer，不产生拼包的内存峰值。

**④ 扩展更友好**  
后续新增传感器数据只需新增 CMD_ID，接收方只注册自己关心的回调，大帧方案会变成"垃圾桶"越来越臃肿。

#### 推荐的发送策略

| 帧类型           | CMD_ID | 发送频率       | 触发方式                    |
|-----------------|--------|--------------|---------------------------|
| ODOMETRY        | `0x21` | 100 Hz       | 定时器（运动控制主循环）      |
| CHASSIS_STATUS  | `0x20` | 1 Hz + 事件  | 定时心跳，状态变化时立即发    |
| BATTERY_INFO    | `0x22` | 1 Hz         | 独立低优先级定时器           |

#### 附：Odometry 帧过大的隐患（668B）

当前 Odometry 帧的 **87% 空间**（576B）被两个 36×double 协方差矩阵占用。大多数底盘 MCU 并不实际计算协方差（全是零或固定值），每帧都发 576B 的无效数据。

**建议方案：**

| 方案 | 说明 | 帧大小 |
|------|------|--------|
| A（推荐）| 去掉协方差，上位机从配置文件静态填充 | **92B（↓86%）** |
| B | 默认发精简帧，上位机发 `CMD_REQUEST_FULL_ODOM` 请求带协方差的完整帧（调试用）| 92B / 668B |
| C | 仅发对角线方差（6×double=48B 替代 288B） | ~140B（↓79%）|

---

### Q3：逆运动学分解（速度 → 各轮 RPM）应放在上位机还是 MCU？

**背景：** 本协议的 `VELOCITY_CMD` 发送车体速度（vx, vy, omega），设计意图是 MCU 在收到后自行完成逆运动学分解，转换为各轮 RPM/转矩指令。但团队当前实际的实现方式是**上位机完成分解，直接向 MCU 发送各轮 RPM / 位置目标**。两种方案均有其适用场景，下面完整记录对比。

---

#### 两种方案的本质区别

```
方案 A（协议设计意图）          方案 B（当前实际做法）
─────────────────────────       ─────────────────────────
上位机                           上位机
  │ vx, vy, omega（车体速度）      │ 逆运动学（在上位机完成）
  │ ──── VELOCITY_CMD ────►       │ w1, w2, w3, w4（各轮 RPM）
MCU                               │ ──── WHEEL_CMD ──────────►
  │ 逆运动学（在 MCU 完成）        MCU
  │ → PWM 驱动各电机              │ 直接输出 PWM / FOC 目标
```

---

#### 各维度对比

| 维度 | 方案 A：MCU 做逆运动学 | 方案 B：上位机做逆运动学 |
|------|----------------------|------------------------|
| **同步性** | ✅ 单帧原子下发，各轮同时生效 | ⚠️ 若分多帧发送，各轮生效时刻可能错位（RS485 尤为明显） |
| **协议通用性** | ✅ VELOCITY_CMD 对差速/麦轮/全向均兼容，切换底盘构型不需改协议 | ❌ 电机数量或布局变化必须同步修改协议帧结构 |
| **调参便利性** | ⚠️ 轮径/轴距等参数需烧录进 MCU 固件 | ✅ 参数在上位机配置文件修改，无需重烧固件 |
| **MCU 算力要求** | ⚠️ 需要浮点运算（现代 STM32 FPU 可轻松满足，古老 8 位 MCU 压力大） | ✅ MCU 只做 PID 环路，无需浮点运动学 |
| **与 FOC 控制器的兼容性** | ❌ ODrive / VESC 等成品 FOC 控制器原生接受 RPM，不接受车体速度，需额外固件层 | ✅ 直接对接成品 FOC 控制器，无需中间层 |
| **可观测性** | ⚠️ 逆运动学在 MCU 内，上位机无法直接看到各轮目标 RPM | ✅ 上位机可以实时记录和可视化每个轮子的目标 RPM |
| **急停安全** | ✅ MCU 本地可以根据速度限幅自主急停 | ⚠️ 急停逻辑必须在上位机实现，MCU 只是执行者 |

---

#### 如何选择

**优先选方案 A（MCU 做逆运动学）** 的场景：
- MCU 是**自研**的底盘控制板，具备 FPU
- 需要**跨传输层**统一协议（RS485 + TCP 共用同一帧格式）
- 后续会接入多种构型（差速、麦轮、足式）的底盘，不想每次改协议

**优先选方案 B（上位机做逆运动学）** 的场景：
- MCU 使用**成品 FOC 控制器**（ODrive / VESC / SimpleFOC），原生接受 RPM 或电流目标
- 需要在上位机**实时调试运动学参数**（轮径、轴距）而不重烧 MCU
- 团队**MCU 侧资源受限**，固件只做电流环/速度环

---

#### 本项目当前状态与建议

本项目上位机侧（`DifferentialController`、`MecanumController`）的 `sendVelocity()` 目前只做**限幅**，不做逆运动学分解；分解逻辑预期由 MCU 实现。

若团队要保持**方案 B（上位机分解）** 的当前实践，建议：

1. 在协议中新增 `WHEEL_CMD (0x13)` 帧，显式携带各轮目标（RPM 或 rad/s），取代将 vx/vy/omega 语义歧义地发给只理解 RPM 的 MCU
2. 在 `DifferentialController` / `MecanumController` 的 `sendVelocity()` 中实现逆运动学，转换后调用 `sendWheelCmd()` 而非 `sendVelocity()`
3. 在 `WHEEL_CMD` 帧中包含**所有电机的目标**（单帧），确保同步性

```
// 建议的 WHEEL_CMD (0x13) payload — 差速 2 轮示例（可扩展到 N 轮）
[seq:4B][timestamp_us:8B][n_wheels:1B][wheel0_rpm:8B][wheel1_rpm:8B]…
```

这样既保留上位机做运动学的灵活性，又通过单帧保证多轮指令的原子性。

---

## 许可证

Apache-2.0，详见 `package.xml`。
