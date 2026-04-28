# ResourceManager — 接口所有权与 Loaned 内存模型

> 源码路径：`hardware_interface/src/resource_manager.cpp`  
> 关键头文件：`hardware_interface/include/hardware_interface/resource_manager.hpp`

---

## 一、ResourceManager 的职责

`ResourceManager`（RM）是 ros2_control 框架的**内存协调中心**：

1. **加载硬件插件**：解析 URDF，通过 pluginlib 实例化 System/Actuator/Sensor
2. **收集接口**：调用 `export_state_interfaces()` / `export_command_interfaces()`，统一索引
3. **管理接口所有权**：保证每个 `CommandInterface` 在同一时刻最多被一个控制器 claim
4. **驱动生命周期**：管理所有硬件组件的 lifecycle 状态
5. **批量 read/write**：在 CM 的主循环中调用所有激活硬件的 `read()` / `write()`

---

## 二、核心数据结构（内存布局）

```
ResourceManager
│
├── hardware_components_info_    map<string, HardwareComponentInfo>
│   │
│   └── [name] → HardwareComponentInfo {
│         hardware: shared_ptr<SystemInterface>  ← 硬件插件实例
│         info:     HardwareInfo                 ← URDF 解析的元数据
│         state:    lifecycle_state              ← 当前生命周期状态
│         ├── state_interfaces: vector<StateInterface>
│         │     StateInterface { prefix, name, double* value_ptr }
│         │     StateInterface { prefix, name, double* value_ptr }
│         │     ...
│         └── command_interfaces: vector<CommandInterface>
│               CommandInterface { prefix, name, double* value_ptr }
│               ...
│
├── state_interface_map_         map<string, ref_wrapper<StateInterface>>
│   ├── "joint1/position"  → &state_interfaces_[0]
│   ├── "joint1/velocity"  → &state_interfaces_[1]
│   ├── "joint2/position"  → &state_interfaces_[2]
│   └── ...
│
├── command_interface_map_       map<string, ref_wrapper<CommandInterface>>
│   ├── "joint1/position"  → &command_interfaces_[0]
│   ├── "joint1/velocity"  → &command_interfaces_[1]
│   └── ...
│
└── claimed_command_interface_map_   unordered_set<string>
    ├── "joint1/position"   ← 已被某控制器 claim
    └── ...                 ← 未 claim 的不在此集合中
```

**关键设计**：
- `StateInterface` / `CommandInterface` 对象中的 `double* value_ptr_` 指向硬件插件内部的 `double` 变量（如 `position_states_[i]`）
- 没有额外的内存拷贝，控制器和硬件插件通过**指针共享同一块 double**
- `ResourceManager` 仅持有这些对象的引用，不拥有底层 double 内存（由硬件插件拥有）

---

## 三、接口生命周期

### 3.1 Export（初始化阶段，非实时）

```
ControllerManager::init()
  → ResourceManager::load_urdf(urdf)
      → hw->on_init(info)
      → hw->on_configure()
      → auto state_ifaces = hw->export_state_interfaces()
           // 返回 vector<StateInterface>，每个 StateInterface 内的 value_ptr 指向
           // hw 内部的 double 数组（如 position_states_[i]）
      → for iface in state_ifaces:
            state_interface_map_[iface.get_full_name()] = std::ref(iface)
            // 存储引用，注意 iface 必须由 hw 内部稳定持有，不能是局部变量
      → 同理处理 command_interfaces
```

**坑点**：`export_state_interfaces()` 返回 `vector<StateInterface>` 时，这些对象必须被稳定保存在 HardwareComponentInfo 中，而不能是临时变量。ResourceManager 内部会将 `hw_component_info.state_interfaces = hw->export_state_interfaces()` 存下，之后 `state_interface_map_` 存储的引用指向 `hw_component_info.state_interfaces[i]`，后者的 `value_ptr_` 指向 hw 内部的 double。

### 3.2 Claim（控制器配置阶段，非实时）

```cpp
// hardware_interface/src/resource_manager.cpp
std::vector<LoanedCommandInterface>
ResourceManager::claim_command_interfaces(
  const std::vector<std::string> & interfaces)
{
  std::vector<LoanedCommandInterface> loaned;
  for (const auto & name : interfaces) {
    if (claimed_command_interface_map_.count(name)) {
      throw std::runtime_error("Interface " + name + " already claimed!");
    }
    claimed_command_interface_map_.insert(name);
    loaned.emplace_back(command_interface_map_.at(name));
  }
  return loaned;
}
```

控制器在 `on_configure()` 时调用 `get_command_interface_names()` 声明需求，CM 调用 `claim_command_interfaces()` 完成 claim。

### 3.3 Release（控制器 deactivate，非实时）

```cpp
ResourceManager::release_command_interface(const std::string & name)
{
  claimed_command_interface_map_.erase(name);
}
```

控制器 deactivate 时，CM 自动释放其持有的所有 LoanedCommandInterface。

---

## 四、`LoanedStateInterface` vs `LoanedCommandInterface`

### 4.1 `LoanedStateInterface`

```cpp
// hardware_interface/include/hardware_interface/loaned_state_interface.hpp
class LoanedStateInterface {
  std::reference_wrapper<StateInterface> interface_;
public:
  explicit LoanedStateInterface(StateInterface & interface) : interface_(interface) {}

  double get_value() const {
    return interface_.get().get_value();
    // → StateInterface::get_value() → return *value_ptr_
  }

  const std::string & get_name() const { return interface_.get().get_name(); }
  const std::string & get_prefix_name() const { return interface_.get().get_prefix_name(); }
  const std::string & get_interface_name() const { return interface_.get().get_interface_name(); }
};
```

特点：
- **不互斥**：多个控制器可以同时读同一个 `StateInterface`
- `get_value()` 直接解引用 `double*`，零拷贝，O(1)

### 4.2 `LoanedCommandInterface`

```cpp
class LoanedCommandInterface {
  std::reference_wrapper<CommandInterface> interface_;
  // 部分实现中包含互斥保护（取决于版本）
public:
  void set_value(double value) {
    interface_.get().set_value(value);
    // → *value_ptr_ = value  （写入共享内存）
  }

  double get_value() const { return interface_.get().get_value(); }
};
```

特点：
- **互斥**：同一时刻只有一个控制器可以持有（ResourceManager 的 `claimed` set 保证）
- `set_value()` 直接写指针，O(1)

---

## 五、接口所有权冲突的处理

当两个控制器都想控制同一个关节的 `position` command 时：

```
spawner joint_trajectory_controller  # 声明 joint1/position command
spawner forward_command_controller   # 同样声明 joint1/position command

→ CM::configure_controller(forward_command_controller)
    → RM::claim_command_interfaces(["joint1/position"])
    → THROW: "Interface joint1/position already claimed by joint_trajectory_controller"
```

CM 的 `switch_controller` 支持**原子 start/stop**：

```bash
# 原子切换：先停 JTC，再启 FCC
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{start_controllers: ['forward_command_controller'],
    stop_controllers: ['joint_trajectory_controller'],
    strictness: 2}"  # STRICT 模式
```

---

## 六、StateInterface 的并发读安全

在 CM 的实时循环中：
- `read()` 写入 `state_interface` 的 `double*`
- `update()` 读取同一 `double*`

这两个操作在**同一线程**（CM 实时线程）中顺序执行，不存在并发问题。

但如果外部线程（如 ROS 2 topic callback）也尝试读取硬件状态，则需要注意：

```
⚠️ 默认情况下，ros2_control 不对 value_ptr_ 加锁。
   外部线程直接读取 StateInterface 的值可能读到撕裂（tearing）的中间状态。
```

解决方案：
1. 使用 `state_publisher_controller`（官方内置控制器）发布状态到 ROS 2 topic，外部节点订阅 topic
2. 不直接访问 `StateInterface`，只通过 ROS 2 通信读状态

---

## 七、ResourceManager 的硬件生命周期管理

```cpp
// 触发单个硬件组件的生命周期转换
hardware_interface::return_type
ResourceManager::set_component_state(
  const std::string & component_name,
  rclcpp_lifecycle::State & target_state)
{
  auto & hw = hardware_components_info_.at(component_name).hardware;
  // 根据 target_state.id() 调用对应回调
  switch (target_state.id()) {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
      // 可能是 configure 或 deactivate，取决于当前状态
      hw->on_configure / hw->on_deactivate();
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      hw->on_activate();
    ...
  }
}
```

CLI 操作：
```bash
# 查看所有硬件组件状态
ros2 control list_hardware_components

# 手动 activate/deactivate 某个硬件组件
ros2 control set_hardware_component_state my_robot active
ros2 control set_hardware_component_state my_robot inactive
```

---

## 八、GPIO 接口（非关节自定义 IO）

除 joint / sensor，`ros2_control` 还支持 GPIO（通用数字/模拟 IO）：

```xml
<gpio name="digital_io">
  <command_interface name="pin0"/>    <!-- 数字输出 -->
  <command_interface name="pin1"/>
  <state_interface name="pin0"/>      <!-- 数字输入读回 -->
  <state_interface name="analog_in"/> <!-- 模拟输入 -->
</gpio>
```

GPIO 接口与关节接口在 ResourceManager 中使用相同的机制管理，名称格式：`digital_io/pin0`。

**具身智能典型用途**：气动夹爪开关、急停按钮状态、LED 指示灯控制。
