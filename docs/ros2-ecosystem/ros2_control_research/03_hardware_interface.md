# hardware_interface — System / Actuator / Sensor 三种抽象深度对比

> 源码路径：`hardware_interface/include/hardware_interface/`  
> 关键文件：`system_interface.hpp` / `actuator_interface.hpp` / `sensor_interface.hpp`

---

## 一、三种接口的定位

| 接口类型 | 对应物理对象 | 是否有 Command | 是否有 State | 典型场景 |
|---------|------------|--------------|------------|---------|
| `SystemInterface` | 完整系统（多关节+传感器一体） | ✅ | ✅ | 机械臂控制板、底盘电机板、IMU+电机复合板 |
| `ActuatorInterface` | 单个驱动器（含传动） | ✅ | ✅ | 单个 BLDC 电机 + 编码器 |
| `SensorInterface` | 纯传感器（只读） | ❌ | ✅ | 力矩传感器、IMU、激光雷达（只读） |

**最常用的是 `SystemInterface`**。在具身智能中，一个 CAN 总线节点通常管理多个关节，天然对应 System。

---

## 二、`SystemInterface` 详解

### 2.1 类定义

```cpp
// hardware_interface/include/hardware_interface/system_interface.hpp
class SystemInterface : public rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface {
public:
  // ── 生命周期回调 ──────────────────────────────────────────
  virtual CallbackReturn on_init(const HardwareInfo & info);
  virtual CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state);
  virtual CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state);
  virtual CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state);
  virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state);
  virtual CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state);
  virtual CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state);

  // ── 接口导出（初始化后调用一次）──────────────────────────
  virtual std::vector<StateInterface> export_state_interfaces() = 0;
  virtual std::vector<CommandInterface> export_command_interfaces() = 0;

  // ── 实时循环回调（每个 CM 周期调用）──────────────────────
  virtual return_type read(const rclcpp::Time & time,
                           const rclcpp::Duration & period) = 0;
  virtual return_type write(const rclcpp::Time & time,
                            const rclcpp::Duration & period) = 0;

protected:
  HardwareInfo info_;   // URDF 解析结果，on_init() 后可用
};
```

### 2.2 实现模板（具身智能机械臂示例）

```cpp
class MyArmSystem : public hardware_interface::SystemInterface {
  // 内部缓冲区
  std::vector<double> position_states_;     // 从硬件读到的位置
  std::vector<double> velocity_states_;
  std::vector<double> effort_states_;
  std::vector<double> position_commands_;   // 控制器写入的目标位置
  std::vector<double> velocity_commands_;

  CANBusDriver can_;  // 自定义通信驱动

public:
  CallbackReturn on_init(const HardwareInfo & info) override {
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
    // 从 info_.hardware_parameters 读取 CAN 接口名、波特率等
    can_interface_ = info_.hardware_parameters.at("can_interface");
    // 分配缓冲区（关节数量）
    size_t n = info_.joints.size();
    position_states_.resize(n, std::numeric_limits<double>::quiet_NaN());
    velocity_states_.resize(n, 0.0);
    effort_states_.resize(n, 0.0);
    position_commands_.resize(n, std::numeric_limits<double>::quiet_NaN());
    velocity_commands_.resize(n, 0.0);
    return CallbackReturn::SUCCESS;
  }

  std::vector<StateInterface> export_state_interfaces() override {
    std::vector<StateInterface> interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      interfaces.emplace_back(info_.joints[i].name, HW_IF_POSITION, &position_states_[i]);
      interfaces.emplace_back(info_.joints[i].name, HW_IF_VELOCITY, &velocity_states_[i]);
      interfaces.emplace_back(info_.joints[i].name, HW_IF_EFFORT,   &effort_states_[i]);
    }
    return interfaces;
  }

  std::vector<CommandInterface> export_command_interfaces() override {
    std::vector<CommandInterface> interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      interfaces.emplace_back(info_.joints[i].name, HW_IF_POSITION, &position_commands_[i]);
      interfaces.emplace_back(info_.joints[i].name, HW_IF_VELOCITY, &velocity_commands_[i]);
    }
    return interfaces;
  }

  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override {
    // 从 CAN 总线读取各关节反馈
    auto feedbacks = can_.read_all_joints();
    for (size_t i = 0; i < feedbacks.size(); ++i) {
      position_states_[i] = feedbacks[i].position;
      velocity_states_[i] = feedbacks[i].velocity;
      effort_states_[i]   = feedbacks[i].current;
    }
    return return_type::OK;
  }

  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override {
    // 把控制器计算的 command 发给硬件
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      can_.send_position_command(i, position_commands_[i]);
    }
    return return_type::OK;
  }
};

// pluginlib 导出宏
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(MyArmSystem, hardware_interface::SystemInterface)
```

---

## 三、`ActuatorInterface` 详解

### 3.1 与 SystemInterface 的区别

`ActuatorInterface` 专为**单个驱动器 + 传动机构**设计：

```cpp
class ActuatorInterface : public rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface {
  // 与 SystemInterface API 完全相同，但语义上是单关节
  virtual std::vector<StateInterface> export_state_interfaces() = 0;
  virtual std::vector<CommandInterface> export_command_interfaces() = 0;
  virtual return_type read(...) = 0;
  virtual return_type write(...) = 0;
};
```

### 3.2 与 `transmission_interface` 配合

`ActuatorInterface` 通常与 `transmission_interface` 配合使用，处理关节空间到驱动器空间的转换：

```
joint_position（关节角度）
    ↕ transmission（减速比 N:1、四杆连杆等）
actuator_position（电机编码器）
```

使用场景：
- 独立控制每个电机（每个电机一个 `ActuatorInterface` 插件）
- 需要精确管理传动比（协作机器人、腱驱手）

**实践建议**：在具身智能中，**优先使用 SystemInterface**，因为：
1. 同一 CAN/串口总线上的多关节更适合一个整体 `read()` / `write()`，效率更高
2. 传动比通常在硬件层处理，不需要 transmission_interface

---

## 四、`SensorInterface` 详解

### 4.1 用途

```cpp
class SensorInterface : public rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface {
  virtual std::vector<StateInterface> export_state_interfaces() = 0;
  // 注意：无 export_command_interfaces()
  virtual return_type read(...) = 0;
  // 注意：无 write()
};
```

### 4.2 典型场景

```cpp
// 六轴力矩传感器
class FTSensorInterface : public hardware_interface::SensorInterface {
  double fx_, fy_, fz_, tx_, ty_, tz_;
  RS485Driver rs485_;

  std::vector<StateInterface> export_state_interfaces() override {
    return {
      StateInterface(info_.sensors[0].name, "force.x",  &fx_),
      StateInterface(info_.sensors[0].name, "force.y",  &fy_),
      StateInterface(info_.sensors[0].name, "force.z",  &fz_),
      StateInterface(info_.sensors[0].name, "torque.x", &tx_),
      StateInterface(info_.sensors[0].name, "torque.y", &ty_),
      StateInterface(info_.sensors[0].name, "torque.z", &tz_),
    };
  }

  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override {
    auto data = rs485_.read_wrench();
    fx_ = data.fx; fy_ = data.fy; fz_ = data.fz;
    tx_ = data.tx; ty_ = data.ty; tz_ = data.tz;
    return return_type::OK;
  }
};
```

---

## 五、三种接口选型决策树

```
需要读取传感器数据？
├── 是：该传感器是否同时需要发送命令（如带标定的智能传感器）？
│     ├── 否 → 使用 SensorInterface
│     └── 是 → 合并到 SystemInterface（将传感器作为 <sensor> 嵌入）
└── 否（纯执行器）：
      有多个关节共享一条通信总线吗？
      ├── 是 → 使用 SystemInterface（统一 read/write 效率更高）
      └── 否（单独一个驱动器）→
            有复杂传动比需要 transmission_interface 管理？
            ├── 是 → 使用 ActuatorInterface + transmission_interface
            └── 否 → 使用 SystemInterface（只有 1 个 joint 也可以）
```

**结论：对于具身智能整机，几乎全部使用 `SystemInterface`，偶尔用 `SensorInterface` 接入独立传感器。`ActuatorInterface` 适合有复杂传动的协作臂或腱驱手。**

---

## 六、自定义 StateInterface / CommandInterface 命名规范

除标准 `position` / `velocity` / `effort`，可以定义任意自定义接口：

```xml
<joint name="gripper">
  <command_interface name="position"/>
  <command_interface name="max_effort"/>     <!-- 自定义：最大夹紧力 -->
  <state_interface name="position"/>
  <state_interface name="current"/>          <!-- 自定义：电流反馈 -->
  <state_interface name="temperature"/>      <!-- 自定义：驱动器温度 -->
</joint>
```

控制器使用时：
```cpp
// 控制器 on_configure() 中
auto [loaned_position, loaned_max_effort] = controller_interface::get_ordered_interfaces(
  command_interfaces_, joint_names_, "position", "max_effort");
```

---

## 七、`mock_components`：无硬件开发利器

`ros2_control` 官方提供 `mock_components`（在 `hardware_interface` 包内），提供：

```xml
<hardware>
  <plugin>mock_components/GenericSystem</plugin>
  <param name="fake_sensor_commands">true</param>
  <param name="state_following_offset">0.0</param>
</hardware>
```

`GenericSystem` 的行为：
- `read()`：直接把 `command_interfaces_` 的值复制到 `state_interfaces_`（模拟完美驱动器）
- `write()`：空操作
- 支持注入初始偏差（测试用）

**在具身智能开发中，mock_components 是"先跑通控制器逻辑、再接真实硬件"的标准流程。**
