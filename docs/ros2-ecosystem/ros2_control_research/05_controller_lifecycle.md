# 控制器生命周期与 ChainableController 级联

> 源码路径：`controller_interface/include/controller_interface/controller_interface.hpp`  
> ChainableController：`controller_interface/include/controller_interface/chainable_controller_interface.hpp`

---

## 一、`ControllerInterface` 继承关系

```
rclcpp_lifecycle::LifecycleNodeInterface   （生命周期回调接口）
           ↑
rclcpp_lifecycle::LifecycleNode            （带生命周期的 Node）
           ↑
controller_interface::ControllerInterfaceBase   （公共基类）
           ↑
    ┌──────┴────────────────────────┐
    │                               │
controller_interface::            controller_interface::
ControllerInterface               ChainableControllerInterface
（普通控制器）                      （可链式控制器）
```

每个控制器本质上是一个**独立的 LifecycleNode**，被 CM 通过 pluginlib 实例化并管理其生命周期。

---

## 二、控制器生命周期状态机

```
               ┌─────────────────────────────┐
               │     UNCONFIGURED             │  ← 插件刚加载完成
               └─────────────┬───────────────┘
                             │ on_configure()
               ┌─────────────▼───────────────┐
               │     INACTIVE                 │  ← 已初始化，未激活
               └──────┬──────────────┬────────┘
          on_activate()│              │on_cleanup()
               ┌───────▼─────────┐   │
               │     ACTIVE       │   │（回到 UNCONFIGURED）
               └───────┬─────────┘   │
         on_deactivate()│             │
               └────────►  INACTIVE  ─┘
```

### 2.1 各生命周期回调的职责

#### `on_configure()` — 声明接口需求（非实时）

```cpp
controller_interface::CallbackReturn
MyController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  // 1. 读取参数
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  interface_type_ = get_node()->get_parameter("interface_type").as_string();

  // 2. 声明需要哪些接口（CM 据此调用 RM::claim_*_interfaces）
  // 注意：这里不是 claim，只是声明
  // claim 由 CM 在 configure_controller() 内调用 RM 完成

  // 3. 创建 ROS 2 实体（publisher/subscriber/tf listener）
  state_pub_ = get_node()->create_publisher<JointState>(...);

  return CallbackReturn::SUCCESS;
}
```

控制器通过覆盖两个纯虚函数声明接口需求：

```cpp
// 声明需要的 command_interfaces
controller_interface::InterfaceConfiguration
MyController::command_interface_configuration() const override
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_) {
    config.names.push_back(joint + "/" + interface_type_);
    // 例如："joint1/position", "joint2/position"
  }
  return config;
}

// 声明需要的 state_interfaces
controller_interface::InterfaceConfiguration
MyController::state_interface_configuration() const override
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_) {
    config.names.push_back(joint + "/position");
    config.names.push_back(joint + "/velocity");
  }
  return config;
}
```

`InterfaceConfiguration::type` 的选项：
| 类型 | 含义 |
|------|------|
| `INDIVIDUAL` | 精确列出所需接口名 |
| `ALL` | 要求所有可用接口（谨慎使用） |
| `NONE` | 不需要任何接口 |

#### `on_activate()` — 实时循环前的最后准备（非实时）

```cpp
controller_interface::CallbackReturn
MyController::on_activate(const rclcpp_lifecycle::State &)
{
  // 此时 command_interfaces_ 和 state_interfaces_ 已被 CM 填充好
  // 可以在这里做初始化，如记录当前位置作为起始点
  start_position_.resize(joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    start_position_[i] = state_interfaces_[i * 2].get_value();  // position
  }

  // 设置初始 command（重要！避免首帧 NaN 导致跳变）
  for (auto & cmd : command_interfaces_) {
    cmd.set_value(0.0);  // 或保持当前位置
  }
  return CallbackReturn::SUCCESS;
}
```

#### `update()` — 实时循环中调用（实时线程！）

```cpp
controller_interface::return_type
MyController::update(const rclcpp::Time & time,
                     const rclcpp::Duration & period)
{
  // 在实时线程中调用，严格禁止：
  //   - 动态内存分配 (new/delete/malloc)
  //   - I/O 操作 (printf/cout/file)
  //   - 锁（std::mutex::lock 有优先级反转风险）
  //   - ROS 2 publish（spin 不在此线程）
  //   - 长时间等待

  // 读取状态
  double current_pos = state_interfaces_[0].get_value();
  double current_vel = state_interfaces_[1].get_value();

  // 控制律计算（PID、轨迹插值等）
  double command = pid_.compute(target_pos_, current_pos, period.seconds());

  // 写入命令
  command_interfaces_[0].set_value(command);

  // 非实时发布（通过 realtime_tools::RealtimePublisher）
  if (rt_publisher_->trylock()) {
    rt_publisher_->msg_.position = current_pos;
    rt_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}
```

#### `on_deactivate()` — 安全停止（非实时）

```cpp
controller_interface::CallbackReturn
MyController::on_deactivate(const rclcpp_lifecycle::State &)
{
  // 将 command interfaces 设为安全值
  for (auto & cmd : command_interfaces_) {
    cmd.set_value(std::numeric_limits<double>::quiet_NaN());
    // NaN 通知下游 hardware_interface 停止输出
  }
  return CallbackReturn::SUCCESS;
}
```

---

## 三、`ChainableControllerInterface` — 控制器级联

### 3.1 概念

`ChainableControllerInterface` 允许一个控制器的**输出成为另一个控制器的输入**（虚拟接口）：

```
                    ┌──────────────────────────┐
                    │  AdmittanceController    │  ← 阻抗/导纳控制
                    │  （ChainableController）  │
                    │  输出：joint position cmd │
                    └────────────┬─────────────┘
                                 │ 虚拟 CommandInterface
                    ┌────────────▼─────────────┐
                    │  JointTrajectoryController│  ← 轨迹执行
                    │  （ChainableController）  │
                    │  输出：joint effort cmd   │
                    └────────────┬─────────────┘
                                 │ 真实 CommandInterface
                    ┌────────────▼─────────────┐
                    │  hardware_interface       │  ← 物理驱动器
                    └──────────────────────────┘
```

### 3.2 关键方法

```cpp
class ChainableControllerInterface : public ControllerInterfaceBase {
public:
  // 导出虚拟接口（供上游控制器 claim）
  virtual std::vector<hardware_interface::CommandInterface>
  export_reference_interfaces() = 0;

  // 声明需要的 reference_interfaces（从上游控制器读）
  virtual bool is_in_chained_mode() const;
  virtual void set_chained_mode(bool chained_mode);

  // update() 内部调用：先从 reference_interfaces_ 读取来自上游的输入
  // 再写入自己的 command_interfaces_
protected:
  std::vector<hardware_interface::LoanedStateInterface> reference_interfaces_;
};
```

### 3.3 典型应用：导纳控制（Admittance Controller）

导纳控制是具身智能接触操作的关键：

```
力矩传感器读数（FT Sensor）
        ↓
AdmittanceController::update()
  1. 读取 ft_sensor state interfaces（力/力矩）
  2. 计算期望轨迹修正量（弹簧阻尼模型）
  3. 写入 reference_interfaces_（虚拟位置 command）
        ↓
JointTrajectoryController::update()
  1. 从 reference_interfaces_ 读取修正后的轨迹目标
  2. 执行轨迹插值 + PID
  3. 写入真实 command_interfaces_（关节力矩/位置）
        ↓
hardware_interface::write()
  发送给电机驱动器
```

### 3.4 ChainableController 的激活顺序

CM 在激活链式控制器时按**反向拓扑顺序**：
1. 先 activate 底层控制器（如 JTC）
2. 再 activate 上层控制器（如 AdmittanceController）

切换时使用 `switch_controller` 的 `STRICT` 模式，CM 会自动推断依赖关系。

---

## 四、控制器与 LifecycleNode 的关系

控制器继承 `LifecycleNode`，但其生命周期**由 CM 管理而非 `ros2 lifecycle set`**：

| 特性 | 普通 LifecycleNode | 控制器 |
|------|-------------------|------|
| 生命周期触发方式 | `ros2 lifecycle set` | CM Service 调用 |
| 节点命名 | 独立 | 由 CM 命名空间管理 |
| 参数命名空间 | 独立 | `controller_manager` 子命名空间 |
| Executor | 自己的 | CM 的 Executor（多线程） |

---

## 五、`realtime_tools` — 实时线程安全工具

ros2_control 配套的 `realtime_tools` 包提供若干实时安全工具：

### 5.1 `RealtimePublisher`

```cpp
#include "realtime_tools/realtime_publisher.hpp"

// 在 on_configure() 中初始化
auto pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
rt_pub_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(pub);

// 在 update() 中（实时线程）
if (rt_pub_->trylock()) {
  rt_pub_->msg_.header.stamp = time;
  rt_pub_->msg_.position = {joint_pos_};
  rt_pub_->unlockAndPublish();
}
```

### 5.2 `RealtimeBox`

线程安全的值容器（实时线程写，非实时线程读，或反之）：

```cpp
realtime_tools::RealtimeBox<std::shared_ptr<JointTrajectory>> trajectory_box_;

// 非实时线程（topic callback）写入
trajectory_box_.set(received_trajectory);

// 实时线程（update()）读取
std::shared_ptr<JointTrajectory> traj;
trajectory_box_.get(traj);
```

### 5.3 `RealtimeBuffer`

更轻量的 lock-free 单值缓冲（一写一读）：

```cpp
realtime_tools::RealtimeBuffer<control_msgs::msg::PidState> pid_state_buffer_;
// 实时线程写：
pid_state_buffer_.writeFromNonRT(state);  // 非实时也可用，名字有误导性
// 非实时读：
auto * state_ptr = pid_state_buffer_.readFromRT();
```
