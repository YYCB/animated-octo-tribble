# ControllerManager — 实时调度循环深度剖析

> 源码路径：`controller_manager/src/controller_manager.cpp`  
> 关键头文件：`controller_manager/include/controller_manager/controller_manager.hpp`

---

## 一、ControllerManager 的本质

`ControllerManager`（CM）是 ros2_control 的**运行时心脏**：

- 继承 `rclcpp::Node`（不是 LifecycleNode）
- 对外暴露一组**管理 Service**（`/controller_manager/load_controller`、`/switch_controller` 等）
- 对内驱动一条**固定频率实时循环**：`read → update_controllers → write`
- 是 `ResourceManager` 的唯一持有者

---

## 二、初始化流程

```
ControllerManager 构造函数
  ├── 解析参数：update_rate（默认 100 Hz）、robot_description（URDF）
  ├── ResourceManager::load_urdf(robot_description)
  │     → 解析 <ros2_control> 标签
  │     → pluginlib 加载硬件插件
  │     → 调用 hw->on_init()  /  hw->on_configure()
  │     → 收集所有 StateInterface / CommandInterface
  ├── 注册管理 Service（load / configure / switch / list ...）
  ├── 创建实时定时器：
  │     rclcpp::create_timer(this, update_rate_hz, &CM::read_update_write)
  └── （可选）加载 YAML 中预配置的控制器
```

关键参数（`controller_manager/controller_manager.hpp`）：

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `update_rate` | `int` | 100 | 主循环频率（Hz） |
| `robot_description` | `string` | 必填 | URDF 字符串 |
| `thread_priority` | `int` | 50 | 实时线程优先级（需 `SCHED_FIFO`） |

---

## 三、实时主循环 `read_update_write()`

这是 ros2_control **最核心的代码路径**，理解它就理解了整个框架：

```cpp
// controller_manager/src/controller_manager.cpp （简化）
void ControllerManager::read_update_write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // ── 阶段 1：READ ──────────────────────────────────────────
  auto read_failures = resource_manager_->read(time, period);
  // 遍历所有 ACTIVE 状态的 hardware_interface，调用各自的 read()
  // read() 把物理寄存器值写入 state_interface 的 value_ptr_ 内存

  // ── 阶段 2：UPDATE（控制器计算）───────────────────────────
  auto [controllers_updated, ...] = update_controllers(time, period);
  // 按拓扑顺序（考虑 ChainableController 依赖）遍历 ACTIVE 控制器
  // 每个控制器的 update() 从 loaned state interfaces 读值，
  // 把计算结果写入 loaned command interfaces

  // ── 阶段 3：WRITE ─────────────────────────────────────────
  auto write_failures = resource_manager_->write(time, period);
  // 遍历所有 ACTIVE 状态的 hardware_interface，调用各自的 write()
  // write() 把 command_interface 的 value_ptr_ 内存值发送到物理硬件
}
```

### 3.1 `ResourceManager::read()` 内部

```cpp
// hardware_interface/src/resource_manager.cpp
HardwareReadWriteStatus ResourceManager::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  for (auto & [name, hw_info] : hardware_components_info_) {
    if (hw_info.state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      auto ret = hw_info.hardware->read(time, period);
      if (ret != hardware_interface::return_type::OK) {
        // 错误处理：记录 failed_hardware_names
      }
    }
  }
}
```

### 3.2 `update_controllers()` 内部

```cpp
// 简化
for (auto & controller : rt_controllers_snapshot_) {
  if (is_active(controller)) {
    auto ret = controller->update(time, period);
    // ret == SUCCESS / ERROR / DEACTIVATE
    if (ret == return_type::DEACTIVATE) {
      // 请求异步 deactivate（不在实时线程中执行）
    }
  }
}
```

关键：**`rt_controllers_snapshot_` 是一个不可变快照**，在非实时线程（switch_controller 时）原子切换，避免实时线程访问锁。

### 3.3 `ResourceManager::write()` 内部

与 `read()` 对称，遍历 ACTIVE 硬件并调用 `write()`。

---

## 四、read/write/update 时序图

```
t=0          t=T/update_rate
│            │
▼            ▼
┌────────────┬─────────────────────┬───────────────┐
│  READ      │  UPDATE             │  WRITE        │
│ (all HW)   │ (all controllers)   │ (all HW)      │
│ ~1-2µs     │ ~10-100µs           │ ~1-2µs        │
└────────────┴─────────────────────┴───────────────┘
     │              │                    │
     │         控制律计算                │
     │         (PID/轨迹插值/etc)        │
     ▼                                   ▼
  state_interface[]               command_interface[]
  （共享内存）                     （共享内存）
```

- **READ 优先**：必须先读取最新传感器状态，再计算控制量
- **UPDATE 在中间**：基于最新状态计算，写入 command
- **WRITE 最后**：把计算出的 command 发给硬件

---

## 五、控制器管理 Service 接口

CM 暴露以下 ROS 2 Services（`controller_manager_msgs`）：

| Service | 功能 |
|---------|------|
| `~/load_controller` | 动态加载控制器插件 |
| `~/configure_controller` | 触发 on_configure()，claim interfaces |
| `~/switch_controller` | activate / deactivate 控制器（原子切换） |
| `~/unload_controller` | 卸载插件 |
| `~/list_controllers` | 列出当前所有控制器及状态 |
| `~/list_controller_types` | 列出可用插件 |
| `~/list_hardware_components` | 列出硬件组件状态 |
| `~/list_hardware_interfaces` | 列出所有 state/command 接口 |
| `~/set_hardware_component_state` | 触发硬件组件生命周期转换 |

CLI 工具（`controller_manager` 包内）：

```bash
# 加载并激活控制器
ros2 run controller_manager spawner joint_trajectory_controller \
  --controller-manager /controller_manager \
  --param-file controller_params.yaml

# 停止并卸载
ros2 run controller_manager unspawner joint_trajectory_controller

# 查看当前状态
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

---

## 六、`switch_controller` 的原子切换机制

这是保证实时安全的关键设计：

```
非实时线程（Service callback）:
  1. 请求 switch（start_controllers, stop_controllers 列表）
  2. 触发 configure / activate / deactivate（调用生命周期回调）
  3. 构造新的 rt_controllers_snapshot_（新的 shared_ptr 列表）
  4. 原子替换：
       rt_controllers_snapshot_.store(new_snapshot, memory_order_release)

实时线程（timer callback）:
  auto snapshot = rt_controllers_snapshot_.load(memory_order_acquire)
  // 始终使用一致的快照，不受 switch 影响
```

这个模式来自经典的 **lock-free RCU（Read-Copy-Update）** 思想：
- 实时线程从不持锁
- 非实时线程修改副本，原子发布
- 旧快照的引用计数归零后自动释放

---

## 七、update_rate 与实时性配置

### 7.1 频率设置

```yaml
# controller_manager 节点参数
controller_manager:
  ros__parameters:
    update_rate: 1000  # Hz，建议范围：100~2000
```

### 7.2 实时线程优先级

CM 使用 `rclcpp::TimerBase` 驱动循环，**默认不是实时优先级**。  
要获得 PREEMPT_RT 效果，需要在 CM 启动后设置：

```cpp
// controller_manager 内部（若使用 realtime_tools）
realtime_tools::configure_sched_fifo(thread_priority_);
```

或在启动脚本中：
```bash
# 需要 CAP_SYS_NICE 权限或 root
chrt -f 80 ros2 run controller_manager controller_manager
```

### 7.3 与 hardware_interface 的频率匹配

硬件插件的 `read()` / `write()` 在**每个 CM 循环都被调用**，频率与 `update_rate` 相同。  
若硬件通信频率低于 CM 频率（如串口 200Hz，CM 1000Hz），需在 `read()` 内部处理：

```cpp
return_type MySystem::read(const rclcpp::Time & time, const rclcpp::Duration & period) {
  // 只有 5 次中的 1 次才真正读串口（插值其余帧）
  if (++read_counter_ % 5 == 0) {
    actual_serial_read();
  }
  return return_type::OK;
}
```

---

## 八、错误处理与硬件故障恢复

CM 提供分级错误处理：

```cpp
// controller_manager/src/controller_manager.cpp
enum class HardwareComponentErrorCodes {
  OK,
  DEACTIVATE,   // 请求 deactivate 该硬件
  ERROR,        // 硬件严重错误
};
```

硬件 `read()` / `write()` 返回 `return_type::ERROR` 时：
1. CM 记录错误计数
2. 超过阈值后触发硬件 deactivate（通过非实时线程的异步请求）
3. 控制器依赖该硬件的 interfaces 会失去访问权，控制器也会被 deactivate

---

## 九、多 CM 场景（分布式控制）

一个机器人进程可以运行**多个 CM 实例**，每个管理一组硬件：

```yaml
# 例如：左臂 CM + 右臂 CM + 底盘 CM
left_arm_controller_manager:
  ros__parameters:
    update_rate: 500
right_arm_controller_manager:
  ros__parameters:
    update_rate: 500
chassis_controller_manager:
  ros__parameters:
    update_rate: 100
```

各 CM 独立运行，通过 ROS 2 topic/action 进行跨 CM 协调。这在具身智能整机中是**推荐架构**：保证各子系统的实时性边界清晰。
