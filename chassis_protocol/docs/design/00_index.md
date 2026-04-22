# chassis_protocol 详细设计文档 — 总目录

> **文档版本**：与代码版本 `0.1.0` 对应  
> **语言**：C++17 / ROS2 Humble  
> **包路径**：`chassis_protocol/`

---

## 文件列表

| 编号 | 文件 | 内容摘要 |
|------|------|---------|
| 01 | [01_architecture.md](01_architecture.md) | 整体分层架构、模块依赖关系、目录结构 |
| 02 | [02_frame_codec.md](02_frame_codec.md) | 二进制帧格式、CRC16-CCITT 算法、`StreamDecoder` 状态机 |
| 03 | [03_transport.md](03_transport.md) | `ITransport` 抽象接口、TCP / UDP / RS485 三种传输实现 |
| 04 | [04_version_negotiator.md](04_version_negotiator.md) | 握手协商协议、二进制 Payload 布局、超时重试状态机 |
| 05 | [05_chassis_hal.md](05_chassis_hal.md) | `ChassisHal` 核心逻辑：连接管理、下行编码、上行解析、断线重连 |
| 06 | [06_controllers.md](06_controllers.md) | `ControlMode` 双模式设计、差速/麦轮逆运动学公式、参数说明 |
| 07 | [07_ros2_adapter.md](07_ros2_adapter.md) | `ChassisNode` ROS2 适配层：参数声明、话题映射、消息转换 |
| 08 | [08_data_flows.md](08_data_flows.md) | 四条关键数据流的完整时序图：连接流程、命令下发、数据上行、断线重连 |

---

## 快速导航

### 我想了解 **帧是怎么编解码的**
→ [02_frame_codec.md](02_frame_codec.md)

### 我想了解 **传输层怎么收发字节**
→ [03_transport.md](03_transport.md)

### 我想了解 **连接时的握手流程**
→ [04_version_negotiator.md](04_version_negotiator.md)

### 我想了解 **`BODY_VELOCITY` vs `WHEEL_RPM` 两种控制模式**
→ [06_controllers.md](06_controllers.md)

### 我想了解 **ROS2 话题、参数配置**
→ [07_ros2_adapter.md](07_ros2_adapter.md)

### 我想通过时序图理解完整流程
→ [08_data_flows.md](08_data_flows.md)

---

## 架构一句话总结

```
ROS2 /cmd_vel → ChassisNode → IChassisController
                                    ↓ (DifferentialController / MecanumController)
                                ChassisHal
                                    ↓ FrameCodec::encode()
                                ITransport (TCP / UDP / RS485)
                                    ↓ POSIX socket / tty fd
                                MCU 底层固件
```
