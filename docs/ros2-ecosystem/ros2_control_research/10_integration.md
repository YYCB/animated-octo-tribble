# ros2_control vs 自研 HAL — 决策矩阵与 chassis_protocol 对照分析

> 本文回答 Phase 1 的核心调研目标：  
> **"何时用 `ros2_control` / 何时像 `chassis_protocol` 那样自研 HAL？"**

---

## 一、`chassis_protocol` 架构回顾

本仓库的 `chassis_protocol` 实现了如下分层架构：

```
Proto（协议定义）
  └── FrameCodec（帧编解码）
        └── ITransport（TCP / UDP / RS485）← 可插拔传输层
              └── ChassisHal（底盘 HAL，含 read/write 逻辑）
                    └── IChassisController（控制器接口）
                          └── ChassisNode（ROS 2 节点，订阅 cmd_vel，发布 odom）
```

关键设计决策（来自源码）：
- 自定义帧协议（非标准 CAN/Modbus），`FrameCodec` 处理分包/校验
- 传输层抽象允许在 TCP/UDP/RS485 间切换（用于调试/部署切换）
- `ChassisNode` 直接订阅 `geometry_msgs/Twist`（cmd_vel），不经过 `ros2_control` 的 controller_manager

---

## 二、两种路线的完整对比矩阵

### 2.1 架构对比

| 维度 | `ros2_control` 路线 | `chassis_protocol` 自研路线 |
|------|--------------------|-----------------------------|
| **标准化程度** | 行业标准接口，MoveIt 2 / Nav2 开箱可用 | 自定义接口，需适配所有上层框架 |
| **控制器复用** | 直接使用 `DiffDriveController` / `JTC` | 完全自研，无法复用生态控制器 |
| **插件体系** | pluginlib，可动态切换控制器/硬件 | 静态编译，切换需修改代码 |
| **实时循环** | CM 管理，支持 kHz 级，可配置优先级 | 自实现，需自己处理定时器精度 |
| **生命周期管理** | LifecycleNode 标准状态机（5 态） | 自定义状态管理 |
| **Sim2Real** | 换一行 URDF（mock→sim→real）| 需实现不同后端 + 条件编译 |
| **诊断集成** | `hardware_state_broadcaster` + `/diagnostics` | 需自实现 |
| **学习曲线** | 较高（需理解 CM/RM/HAL 分层） | 较低（直接开发） |
| **协议自由度** | 高（hardware_interface 内部完全自由） | 高 |
| **代码量** | 较少（控制器复用） | 较多（全部自研） |

### 2.2 性能对比

| 指标 | `ros2_control` | 自研 HAL |
|------|---------------|---------|
| 控制频率上限 | > 1kHz（实测） | 取决于实现 |
| 控制环延迟 | < 1ms（同进程） | 取决于线程模型 |
| 内存模型 | 无拷贝（指针共享） | 取决于实现 |
| 跨节点通信 | 无（同进程 RM） | 取决于实现 |

### 2.3 适用场景

| 场景 | 推荐路线 | 理由 |
|------|---------|------|
| **新项目机械臂** | ros2_control ⭐⭐⭐⭐⭐ | JTC + MoveIt 2 直接可用，生态完整 |
| **新项目移动底盘** | ros2_control ⭐⭐⭐⭐⭐ | DiffDriveController / Mecanum + Nav2 |
| **标准 CAN/EtherCAT 协议** | ros2_control ⭐⭐⭐⭐⭐ | 直接用 `ros2_canopen` / `ethercat_driver` |
| **高度自定义帧协议（如本仓库）** | 混合方案 ⭐⭐⭐⭐ | 自定义传输层 + ros2_control HAL 包装 |
| **遗留系统迁移** | 渐进式迁移 ⭐⭐⭐ | 先包装 hw_interface，不动控制器逻辑 |
| **MCU 侧（micro-ROS 环境）** | 自研 / micro-ROS | ros2_control 不运行在裸机上 |
| **极低延迟（< 100µs）FPGA** | 自研 | CM 调度开销不可接受 |

---

## 三、`chassis_protocol` 改造为 `hardware_interface::SystemInterface` 的可行性评估

### 3.1 映射关系

| `chassis_protocol` | `hardware_interface::SystemInterface` |
|--------------------|--------------------------------------|
| `ChassisHal::init()` | `on_init(HardwareInfo)` |
| `ChassisHal::connect()` | `on_configure()` |
| `ChassisHal::enable()` | `on_activate()` |
| `ChassisHal::disable()` | `on_deactivate()` |
| `ChassisHal::disconnect()` | `on_cleanup()` |
| `ITransport::read()` + 解码 | `read(time, period)` |
| `ITransport::write()` + 编码 | `write(time, period)` |
| 底盘速度状态（odom） | `StateInterface("wheel_left/velocity")` 等 |
| 底盘速度命令（cmd_vel） | `CommandInterface("wheel_left/velocity")` 等 |

### 3.2 改造方案草图

```cpp
class ChassisHardwareInterface : public hardware_interface::SystemInterface {
  // 复用 chassis_protocol 的传输层
  std::unique_ptr<ITransport> transport_;
  std::unique_ptr<FrameCodec> codec_;

  // State/Command 缓冲区
  double left_wheel_velocity_state_{0.0};
  double right_wheel_velocity_state_{0.0};
  double left_wheel_velocity_command_{0.0};
  double right_wheel_velocity_command_{0.0};

  CallbackReturn on_init(const HardwareInfo & info) override {
    // 从 info.hardware_parameters 读取传输类型、端口等参数
    std::string transport_type = info.hardware_parameters.at("transport");
    if (transport_type == "rs485") {
      transport_ = std::make_unique<RS485Transport>(info.hardware_parameters);
    } else if (transport_type == "tcp") {
      transport_ = std::make_unique<TCPTransport>(info.hardware_parameters);
    }
    codec_ = std::make_unique<FrameCodec>();
    return CallbackReturn::SUCCESS;
  }

  std::vector<StateInterface> export_state_interfaces() override {
    return {
      StateInterface("left_wheel",  HW_IF_VELOCITY, &left_wheel_velocity_state_),
      StateInterface("right_wheel", HW_IF_VELOCITY, &right_wheel_velocity_state_),
    };
  }

  std::vector<CommandInterface> export_command_interfaces() override {
    return {
      CommandInterface("left_wheel",  HW_IF_VELOCITY, &left_wheel_velocity_command_),
      CommandInterface("right_wheel", HW_IF_VELOCITY, &right_wheel_velocity_command_),
    };
  }

  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override {
    // 复用 chassis_protocol 的解码逻辑
    auto frame = transport_->receive();
    auto status = codec_->decode(frame);
    left_wheel_velocity_state_  = status.left_wheel_rpm * RPM_TO_RAD_S;
    right_wheel_velocity_state_ = status.right_wheel_rpm * RPM_TO_RAD_S;
    return return_type::OK;
  }

  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override {
    // 复用 chassis_protocol 的编码逻辑
    auto frame = codec_->encode_velocity(
      left_wheel_velocity_command_,
      right_wheel_velocity_command_
    );
    transport_->send(frame);
    return return_type::OK;
  }
};
```

### 3.3 改造后的收益

改造完成后：
- **直接使用 `DiffDriveController`**：不再需要 `ChassisNode` 中自实现的 Twist→轮速转换、里程计计算
- **直接与 Nav2 兼容**：DiffDriveController 的 odom + TF 输出是 Nav2 的标准输入
- **Sim2Real 一键切换**：Gazebo/Isaac 仿真只需换 URDF 中的 `<plugin>` 行
- **MoveIt 2 协同**：若底盘有关节臂，可在同一 CM 下统一管理底盘 + 手臂

### 3.4 改造的代价与风险

| 代价 | 说明 |
|------|------|
| 开发工作量 | 中等（主要是包装，核心通信逻辑复用） |
| 测试工作量 | 需要验证 ros2_control 实时循环与原有通信时序的匹配 |
| 自定义协议适配 | FrameCodec 的分包/超时逻辑需适配 CM 的周期调用模式 |
| 协议异步性 | 若原协议是异步事件驱动（非周期），需引入缓冲层 |

---

## 四、具身整机推荐拓扑

基于以上分析，具身智能整机（底盘 + 机械臂 + 夹爪 + 头部）的推荐架构：

```
┌─────────────────────────────────────────────────────────┐
│              MoveIt 2                  Nav2              │
│  FollowJointTrajectory(arm)       cmd_vel(chassis)       │
└──────────────────────┬─────────────────┬────────────────┘
                       │                 │
           ┌───────────▼─────────────────▼────────────┐
           │            ControllerManager              │
           │  ┌─────────────────────────────────────┐  │
           │  │ JointTrajectoryController (arm+head) │  │
           │  │ DiffDriveController / MecanumDriver  │  │
           │  │ GripperActionController              │  │
           │  │ AdmittanceController (force control) │  │
           │  │ JointStateBroadcaster                │  │
           │  │ IMUSensorBroadcaster                 │  │
           │  │ ForceTorqueSensorBroadcaster         │  │
           │  └─────────────────────────────────────┘  │
           │            ResourceManager                 │
           └───────┬──────────────┬────────────────────┘
                   │              │
          ┌────────▼──────┐ ┌─────▼──────────────┐
          │ ArmSystem     │ │ ChassisSystem       │
          │ (CAN Bus)     │ │ (RS485/CAN)         │
          │ 6 DOF + FT    │ │ 自定义帧协议        │
          └───────────────┘ └────────────────────┘
                   │              │
          实际硬件（EtherCAT/CAN）  实际底盘电机
```

**底盘**：将 `chassis_protocol` 改造为 `SystemInterface`（`ChassisHardwareInterface`），保留自定义帧协议的传输层，上层换用 `DiffDriveController`。

**机械臂**：直接实现 `SystemInterface`，使用 `JointTrajectoryController` + `AdmittanceController` + `JointStateBroadcaster`。

**夹爪**：`GripperActionController`（内置，支持 `control_msgs/GripperCommand` action）。

**传感器（FT / IMU）**：`SensorInterface` + `ForceTorqueSensorBroadcaster` / `IMUSensorBroadcaster`。

---

## 五、结论

| 问题 | 结论 |
|------|------|
| **何时用 ros2_control？** | 新项目 100% 优先用；遗留项目渐进迁移；关键判断：是否需要与 MoveIt 2 / Nav2 / VLA 接入生态 |
| **何时自研 HAL？** | 运行在 MCU/FPGA 上（无 ROS 2 运行时）；延迟要求 < 100µs；极度自定义协议且生态接入不是优先级 |
| **chassis_protocol 怎么办？** | **短期**：维持现状（已稳定运行，改造有风险）；**中期**：在 `SystemInterface` 包装层复用传输/编解码，换掉 `ChassisNode`，获得 Nav2 生态收益 |
| **整机统一管理？** | 用**多 CM 模式**（底盘 100Hz CM + 机械臂 500Hz CM）或**单 CM**（若频率差异不大）；通过 ROS 2 Action/Topic 跨 CM 协调 |
