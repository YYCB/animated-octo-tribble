# ros2_control 源码深度调研 — 总索引

> **调研版本**：ros2_control `humble`（tag `3.x`）  
> **参考仓库**：https://github.com/ros-controls/ros2_control  
> **关联文档**：本仓库 `chassis_protocol/`、`docs/ros2-core/ros2_lifecycle_node_research/`、`docs/ros2-core/ros2_rclcpp_core_research/`

---

## 一、为什么调研 ros2_control

具身智能整机同时包含：
- **底盘**（差速/全向/履带）—— 本仓库已有 `chassis_protocol` 自研 HAL
- **机械臂**（6/7 DOF）—— 关节力矩/位置控制，高频实时
- **末端执行器**（夹爪/吸盘）—— 简单开关或力控
- **头部/腰部**（姿态伺服）—— 低速位置控制

`ros2_control` 提供了一套**统一的 HAL 抽象 + 实时控制器框架**，是 ROS 2 生态操作栈（MoveIt 2）和移动栈（Nav2）的事实标准接口。调研目标是：

> **回答"何时用 `ros2_control` / 何时像 `chassis_protocol` 那样自研 HAL"**，并给出具身整机的统一拓扑建议。

---

## 二、整体架构总览

```
┌─────────────────────────────────────────────────────────────────────┐
│                         应用层 / 规划层                              │
│  MoveIt 2 (FollowJointTrajectory)  /  Nav2 (cmd_vel)                │
└────────────────────────┬────────────────────────────────────────────┘
                         │ ROS 2 Action / Topic
┌────────────────────────▼────────────────────────────────────────────┐
│                    Controller Manager (CM)                           │
│  • 管理所有控制器的生命周期（configure / activate / deactivate）      │
│  • 驱动实时循环：read() → update() → write()                        │
│  • 通过 ResourceManager 统一管理所有 hardware_interface 句柄         │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │  控制器插件（pluginlib）                                      │   │
│  │  JointTrajectoryController / DiffDriveController /           │   │
│  │  ForwardCommandController / ChainableController ...          │   │
│  └──────────────────────────────────────────────────────────────┘   │
└────────────────────────┬────────────────────────────────────────────┘
                         │ LoanedCommandInterface / LoanedStateInterface
┌────────────────────────▼────────────────────────────────────────────┐
│                    Resource Manager                                  │
│  • 持有所有 hardware_interface 实例                                  │
│  • 管理 StateInterface / CommandInterface 的所有权与借用             │
└────────────────────────┬────────────────────────────────────────────┘
                         │ read() / write()
┌────────────────────────▼────────────────────────────────────────────┐
│              Hardware Interface（HAL 层）                            │
│  ┌─────────────────┐ ┌──────────────────┐ ┌───────────────────────┐ │
│  │ SystemInterface │ │ ActuatorInterface│ │  SensorInterface      │ │
│  │（关节 + 传感器  │ │（单驱动器）      │ │（只读传感器）         │ │
│  │  一体）         │ │                  │ │                        │ │
│  └────────┬────────┘ └────────┬─────────┘ └──────────┬────────────┘ │
└───────────┼──────────────────┼───────────────────────┼──────────────┘
            │                  │                        │
   实际硬件通信（串口 / EtherCAT / CAN / Modbus / 自定义协议）
```

**关键设计原则**：
1. **实时循环由 CM 驱动**，固定频率（默认 `update_rate=100Hz`，可配置至 kHz 级）
2. **控制器不直接操作硬件**，只通过 `LoanedCommandInterface` / `LoanedStateInterface` 读写内存中的值
3. **硬件驱动（`hardware_interface`）负责把内存值同步到实际硬件**，在每次循环的 `read()` / `write()` 中完成
4. **`pluginlib` 实现完全动态加载**，控制器和硬件驱动都是运行时插件

---

## 三、仓库包结构

| 包名 | 职责 |
|------|------|
| `controller_manager` | CM 实现、启动 launch、`spawner` / `unspawner` CLI |
| `hardware_interface` | `SystemInterface` / `ActuatorInterface` / `SensorInterface` 基类、`ResourceManager` |
| `controller_interface` | 控制器基类 `ControllerInterface` / `ChainableControllerInterface` |
| `ros2_control_test_assets` | 测试用 URDF 片段 |
| `transmission_interface` | 传动比（Reduction / FourBarLinkage）抽象 |
| `joint_limits` | 关节限位约束 |

> **非本仓库内**（独立仓库）：  
> `ros2_controllers`（内置控制器集合）、`ros2_control_demos`、`gz_ros2_control`、`isaac_ros2_control`

---

## 四、文档目录

| 文件 | 内容 |
|------|------|
| [01_architecture.md](./01_architecture.md) | 包结构、核心数据结构、URDF 到运行时的映射 |
| [02_controller_manager.md](./02_controller_manager.md) | CM 实时循环、`read/update/write` 时序、`update_rate` 配置 |
| [03_hardware_interface.md](./03_hardware_interface.md) | System / Actuator / Sensor 三种接口对比与选型 |
| [04_resource_manager.md](./04_resource_manager.md) | ResourceManager 内存模型、LoanedInterface 所有权 |
| [05_controller_lifecycle.md](./05_controller_lifecycle.md) | 控制器生命周期、ChainableController 级联 |
| [06_builtin_controllers.md](./06_builtin_controllers.md) | JTC / DiffDrive / Mecanum / ForwardCommand 源码导读 |
| [07_urdf_plugin_loading.md](./07_urdf_plugin_loading.md) | `<ros2_control>` tag 解析、pluginlib 加载流程 |
| [08_external_integrations.md](./08_external_integrations.md) | MoveIt 2 / gz_ros2_control / isaac_ros2_control |
| [09_data_flows.md](./09_data_flows.md) | 端到端时序：启动 → 激活 → 稳态循环 → 关机 |
| [10_integration.md](./10_integration.md) | 决策矩阵：ros2_control vs 自研 HAL（chassis_protocol 对照） |

---

## 五、关键调用链速查

### 5.1 实时循环（稳态）
```
ControllerManager::update()                           # timer callback @ update_rate Hz
  └── ResourceManager::read(time, period)             # 调用所有激活 HW 的 read()
        └── MySystemInterface::read()                 # 把硬件寄存器值写入 state_interfaces_
  └── for each active controller:
        controller->update(time, period)              # 控制器计算
          └── joint_position_state_interface_.get_value()  # 读 state
          └── joint_position_command_interface_.set_value() # 写 command
  └── ResourceManager::write(time, period)            # 调用所有激活 HW 的 write()
        └── MySystemInterface::write()                # 把 command_interfaces_ 值发给硬件
```

### 5.2 控制器加载（非实时）
```
ros2 run controller_manager spawner joint_trajectory_controller
  → /controller_manager/load_controller (Service)
  → CM::load_controller()
      → pluginlib::ClassLoader::createSharedInstance("JTC")
      → controller->init(node, urdf, update_rate, ...)
      → CM::configure_controller()
          → controller->on_configure()
          → ResourceManager::claim_command_interfaces()
  → /controller_manager/switch_controller → activate
      → controller->on_activate()
```

### 5.3 URDF → ResourceManager
```
robot_description (URDF string)
  → ResourceManager::load_urdf()
      → parse <ros2_control> tags
      → ClassLoader::createSharedInstance(hardware plugin)
      → hw->on_init(HardwareInfo)         # 从 URDF 解析 joints / sensors / params
      → hw->on_configure()
      → hw->export_state_interfaces()     # 返回 vector<StateInterface>
      → hw->export_command_interfaces()   # 返回 vector<CommandInterface>
      → 存入 resource_map_[name]
```

---

## 六、与本仓库已有工作的关联

| 已有工作 | ros2_control 对应位置 | 关系 |
|---------|----------------------|------|
| `chassis_protocol::ChassisHal` | `hardware_interface::SystemInterface` | 可作为后者的实现 |
| `chassis_protocol::ITransport` | `SystemInterface::read()` / `write()` 内部细节 | 传输层封装类似 |
| `chassis_protocol::ChassisNode` | `controller_manager` + `diff_drive_controller` | 功能重叠，决策见 `10_integration.md` |
| `ros2_lifecycle_node_research` | `ControllerInterface` 继承自 `rclcpp_lifecycle::LifecycleNode` | 生命周期状态机相同 |
| `ros2_rclcpp_core_research` | CM 内部 `rclcpp::TimerBase` 驱动实时循环 | Node/Timer 章节直接适用 |
