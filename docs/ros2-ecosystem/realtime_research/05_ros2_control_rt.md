# ros2_control 实时控制环深化

> 本文档深入分析 `controller_manager` 的 RT 调度时序、`hardware_interface` 的 RT 约束、`ChainableController` 的延迟模型，以及力控 4kHz 配置和硬件接口 RT 实现要点，并给出与本仓库 `chassis_protocol` 的具体改造建议。

---

## 一、controller_manager update() 时序详解

### 1.1 RT 控制循环全流程

```
┌──────────────────────────────────────────────────────────────────────────┐
│                   controller_manager RT 控制循环                           │
│                                                                          │
│  hrtimer 到期（每 1ms @ 1kHz）                                            │
│         ↓                                                                │
│  ControllerManager::update(time, period)                                 │
│    │                                                                     │
│    ├─ [1] ResourceManager::read_all()   ← 从所有 HW 接口读取状态           │
│    │       for hw in hardware_components:                                │
│    │           hw.read(time, period)    ← 无锁、无 malloc、确定性时间      │
│    │                                                                     │
│    ├─ [2] 更新 StateInterface 数组      ← hw→controller 状态传递          │
│    │       hw_positions[0] → state_iface.position.get_value()            │
│    │                                                                     │
│    ├─ [3] 按优先级顺序调用各 Controller  ← 链式控制                        │
│    │       for ctrl in active_controllers (sorted by chain order):       │
│    │           ctrl.update(time, period)                                 │
│    │               ← 控制算法执行（PID/impedance 等）                      │
│    │               ← 通过 CommandInterface 写出                          │
│    │                                                                     │
│    ├─ [4] ResourceManager::write_all()  ← 将命令写入所有 HW 接口          │
│    │       for hw in hardware_components:                                │
│    │           hw.write(time, period)   ← 无锁、无 malloc、确定性时间      │
│    │                                                                     │
│    └─ [5] 调试/诊断输出（低频，非每次循环）                                 │
│                                                                          │
└──────────────────────────────────────────────────────────────────────────┘
```

### 1.2 与 PREEMPT_RT hrtimer 的绑定

`controller_manager` 使用 `rclcpp::WallTimer` 驱动控制循环。在 PREEMPT_RT 内核上，ROS 2 的 `WallTimer` 最终映射到 `hrtimer`（如果 `rclcpp` 使用 `CLOCK_MONOTONIC`）：

```cpp
// controller_manager_parameters.yaml
update_rate: 1000  # Hz

// 内部实现（controller_manager.cpp）：
// update_timer_ = create_wall_timer(
//     std::chrono::duration<double>(1.0 / update_rate),
//     std::bind(&ControllerManager::update, this));
// 在 PREEMPT_RT 下，此 timer 由 hrtimer 驱动，精度 ~5μs
```

**关键约束**：`ControllerManager::update()` 必须在 `1/update_rate` 时间内完成，否则下一个 timer 触发时会导致控制循环跳拍（missed cycle）。

### 1.3 read() → update() → write() 的时序约束

```
时间轴（1ms = 1000μs 控制周期）：

t=0       t=50μs    t=250μs   t=800μs  t=1000μs
│         │         │         │         │
▼         ▼         ▼         ▼         ▼
hrtimer   read()    update()  write()   hrtimer
触发      完成      完成      完成      下次触发
         
⬅50μs⬅  ⬅200μs⬅  ⬅550μs⬅  ⬅200μs⬅
hw读取   控制计算   控制计算   hw写入
预算     预算(低)  预算(高)  预算
```

---

## 二、hardware_interface RT 约束

### 2.1 on_configure() / on_activate() 中的允许操作

这两个生命周期回调在**非 RT 线程**中执行（controller_manager lifecycle 管理线程），因此可以：

```cpp
CallbackReturn ChassisHAL::on_configure(const rclcpp_lifecycle::State&) {
    // ✅ 允许：动态内存分配
    hw_positions_.resize(info_.joints.size(), 0.0);
    hw_velocities_.resize(info_.joints.size(), 0.0);
    hw_commands_.resize(info_.joints.size(), 0.0);

    // ✅ 允许：版本协商（移出 RT 线程）
    if (!transport_->connect()) return CallbackReturn::ERROR;
    if (!version_negotiator_->negotiate()) return CallbackReturn::ERROR;

    // ✅ 允许：互斥锁初始化
    config_mutex_ = std::make_unique<std::mutex>();

    // ✅ 允许：打印日志
    RCLCPP_INFO(get_logger(), "ChassisHAL configured, %zu joints",
                info_.joints.size());

    return CallbackReturn::SUCCESS;
}

CallbackReturn ChassisHAL::on_activate(const rclcpp_lifecycle::State&) {
    // ✅ 允许：mlockall（如果有权限）
    mlockall(MCL_CURRENT | MCL_FUTURE);

    // ✅ 允许：栈预热
    prefault_stack();

    // ✅ 允许：预先发送首帧数据（唤醒驱动器）
    transport_->send_enable_frame();

    return CallbackReturn::SUCCESS;
}
```

### 2.2 read() / write() 中的禁止操作

这两个函数在 **RT 线程**中以高优先级调用，严格禁止以下操作：

```cpp
// ❌ 严格禁止的操作（会导致不确定性延迟）

// ❌ 动态内存分配
hw_positions_.push_back(0.0);  // 可能触发 realloc
std::string debug_str = std::to_string(val);  // 临时字符串分配

// ❌ printf / cout（内部有 mutex 和系统调用）
printf("joint pos: %f\n", hw_positions_[0]);
std::cout << "debug" << std::endl;

// ❌ RCLCPP_INFO / WARN（即使是 throttled 版本也有 mutex）
RCLCPP_INFO(logger, "...");  // ❌
// ✅ 代替方案：
if (should_log_) {
    log_buffer_[log_idx_++] = hw_positions_[0];  // 非 RT 线程定期读取
}

// ❌ mutex / condition_variable（可能阻塞）
std::lock_guard<std::mutex> lock(mutex_);  // 可能阻塞！

// ❌ 可能失败的系统调用
int fd = open("/dev/ttyUSB0", O_RDWR);  // ❌ 文件操作
sleep(1);                               // ❌ 阻塞调用

// ❌ 抛出异常
throw std::runtime_error("...");        // ❌ 异常处理路径不确定
```

**正确的 read() 实现示例：**

```cpp
hardware_interface::return_type
ChassisHAL::read(const rclcpp::Time& /*time*/,
                 const rclcpp::Duration& /*period*/)
{
    // ✅ 只使用预分配的变量
    // ✅ 只访问栈变量或成员变量（已 reserve）
    // ✅ 使用原子操作读取来自驱动线程的数据
    
    // 从无锁环形缓冲区读取最新传感器数据
    SensorFrame frame;
    if (sensor_ring_buffer_.try_pop(frame)) {  // 无锁，O(1)
        hw_positions_[0] = frame.left_position;
        hw_positions_[1] = frame.right_position;
        hw_velocities_[0] = frame.left_velocity;
        hw_velocities_[1] = frame.right_velocity;
    }
    // 如果 try_pop 失败，保持上一次的值（比返回 ERROR 更安全）
    
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
ChassisHAL::write(const rclcpp::Time& /*time*/,
                  const rclcpp::Duration& /*period*/)
{
    // 将控制命令写入无锁发送队列（驱动线程异步发送）
    CommandFrame cmd;
    cmd.left_velocity = hw_commands_[0];
    cmd.right_velocity = hw_commands_[1];
    
    command_ring_buffer_.push(cmd);  // 无锁，O(1)
    
    return hardware_interface::return_type::OK;
}
```

---

## 三、ChainableController RT 路径

### 3.1 reference_interface 内存模型

`ChainableController` 通过 `reference_interface` 在控制器链中传递命令，无需通过 DDS topic：

```
┌─────────────────────────────────────────────────────────┐
│  ChainableController 链（级联控制）                        │
│                                                         │
│  Impedance Controller                                   │
│    output: reference_interface["joint0/position"] = x  │
│            ↓ (直接内存引用，无拷贝)                       │
│  JointTrajectoryController (as "chained" controller)    │
│    input:  reference_interface["joint0/position"] ← x  │
│    output: command_interface["joint0/velocity"] = v     │
│            ↓                                            │
│  hardware_interface::write()                            │
│    读取 command_interface 值并发送到执行器               │
└─────────────────────────────────────────────────────────┘
```

**reference_interface 是共享内存数组的引用**，不涉及 DDS 消息传输，延迟仅为内存访问时间（~1ns）。

### 3.2 链式更新的延迟叠加

```
t=0       t=50μs    t=100μs   t=150μs  t=250μs
│         │         │         │         │
▼         ▼         ▼         ▼         ▼
read()    Ctrl A    Ctrl B    Ctrl C    write()
全部HW   update()  update()  update()  全部HW

每个 Controller update() 的延迟叠加：
total_control_latency = Σ(ctrl_update_latency_i)

对于 4kHz 力控（250μs 周期）：
- read(): < 30μs
- Impedance controller: < 50μs  
- Force controller: < 50μs
- write(): < 30μs
- 总计: < 160μs（留 90μs 余量）
```

---

## 四、Force-Torque Controller 4kHz 配置

### 4.1 controller_manager 配置

```yaml
# controller_manager 参数（force_control_system.yaml）
controller_manager:
  ros__parameters:
    update_rate: 4000  # Hz（每 250μs 更新一次）
    
    # 控制器列表
    force_torque_controller:
      type: force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster
    joint_impedance_controller:
      type: impedance_controller/ImpedanceController

# 注意：4kHz 需要 PREEMPT_RT 内核 + 专用 CPU 核 + EtherCAT 硬件接口
```

### 4.2 RT 线程绑定（launch 文件）

```python
# force_control.launch.py
import launch
from launch_ros.actions import Node

def generate_launch_description():
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            '/path/to/force_control_system.yaml'
        ],
        # 在独立进程中运行，方便设置 RT 属性
        prefix=['sudo', 'chrt', '-f', '90',
                'taskset', '-c', '2,3'],  # SCHED_FIFO 90, Core 2-3
    )
    return launch.LaunchDescription([ros2_control_node])
```

### 4.3 力控 hardware_interface 实现要点

```cpp
class ForceTorqueHAL : public hardware_interface::SystemInterface {
public:
    hardware_interface::return_type read(
        const rclcpp::Time&, const rclcpp::Duration&)
    {
        // 4kHz 要求：read() 必须在 50μs 内完成
        
        // ✅ 使用内存映射 I/O（mmap EtherCAT PDO）
        // EtherCAT PDO 数据由 EtherCAT master 在 IRQ 线程中更新
        // read() 只需读取 mmap 内存（< 1μs）
        hw_force_[0] = *reinterpret_cast<float*>(ethercat_pdo_ + FX_OFFSET);
        hw_force_[1] = *reinterpret_cast<float*>(ethercat_pdo_ + FY_OFFSET);
        hw_force_[2] = *reinterpret_cast<float*>(ethercat_pdo_ + FZ_OFFSET);
        
        // 时间戳（使用 clock_gettime，唯一允许的系统调用）
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        last_read_time_ns_ = ts.tv_sec * 1000000000LL + ts.tv_nsec;
        
        return hardware_interface::return_type::OK;
    }

private:
    uint8_t* ethercat_pdo_;  // EtherCAT PDO 内存映射地址（on_configure 中 mmap）
    std::array<double, 6> hw_force_{};
    int64_t last_read_time_ns_{0};
    
    static constexpr int FX_OFFSET = 0;
    static constexpr int FY_OFFSET = 4;
    static constexpr int FZ_OFFSET = 8;
};
```

---

## 五、Broadcaster RT 影响与降频策略

### 5.1 JointStateBroadcaster 对控制环 CPU 的影响

`JointStateBroadcaster` 在每次 `controller_manager::update()` 中发布 `/joint_states`。对于 1kHz 控制环，这意味着每秒发布 1000 次 JointState 消息。

**影响分析：**

| 配置 | 每秒消息数 | DDS 序列化开销/周期 | 说明 |
|------|---------|--------------|------|
| 1kHz 控制 + 1kHz JointState | 1000/s | ~20μs | 占用 2% 控制周期 |
| 1kHz 控制 + 100Hz JointState | 100/s | ~2μs（平均） | **推荐配置** |
| 4kHz 控制 + 100Hz JointState | 100/s | ~0.5μs（平均） | 降频必须 |

### 5.2 降频配置（publish_rate 参数）

```yaml
# controller_manager 配置：降低 JointStateBroadcaster 发布频率
joint_state_broadcaster:
  ros__parameters:
    # 发布频率（独立于 controller_manager update_rate）
    publish_rate: 100.0  # Hz（从 1000Hz 降到 100Hz）
    # 这样 JointState 每 10ms 发布一次，控制环每 1ms 执行一次
    # Broadcaster 内部使用计数器跳过未达到发布间隔的更新周期
```

### 5.3 Broadcaster 异步发布模式

对于 RT 要求严格的场景，使用 `realtime_tools::RealtimePublisher` 将发布操作移出 RT 线程：

```cpp
// JointStateBroadcaster 的推荐实现方式
// 内部使用 RealtimePublisher，RT 线程只负责填充消息，
// 实际 DDS 发布在独立的非 RT 线程完成
class JointStateBroadcaster : public ControllerInterface {
    controller_interface::return_type update(
        const rclcpp::Time& time, const rclcpp::Duration&)
    {
        // 检查是否到达发布时间
        if (publish_period_ > 0 && 
            (time - last_publish_time_).seconds() < 1.0 / publish_rate_)
            return controller_interface::return_type::OK;
        
        if (rt_publisher_->trylock()) {
            auto& msg = rt_publisher_->msg_;
            // 填充消息（无 malloc，容量已预分配）
            for (size_t i = 0; i < joint_names_.size(); ++i) {
                msg.position[i] = state_interfaces_[i].get_value();
            }
            rt_publisher_->unlockAndPublish();  // 异步发布
        }
        last_publish_time_ = time;
        return controller_interface::return_type::OK;
    }
};
```

---

## 六、EtherCAT / RS485 硬件接口 RT 实现要点

### 6.1 EtherCAT — 最高确定性

EtherCAT 是工业机器人首选总线，通过 DMA 传输确保确定性：

```
EtherCAT 实时架构：

hrtimer 触发（1kHz / 4kHz）
  ↓
EtherCAT Master（SOEM / IGH EtherLab）
  ↓ DMA 传输（无 CPU 参与）
EtherCAT 从站（电机驱动器/力传感器）
  ↓ 物理以太网（100Mbps，确定性延迟 < 1μs/从站）
  
典型端到端延迟：< 100μs（10 个从站）
```

```cpp
// EtherCAT hardware_interface read() 示例（使用 SOEM）
hardware_interface::return_type
EtherCATHAL::read(const rclcpp::Time&, const rclcpp::Duration&)
{
    // EtherCAT PDO 数据已由 EtherCAT 中断线程在上一个周期写入 pdo_in_
    // read() 只需从 mmap 内存中复制数据（< 1μs）
    for (size_t i = 0; i < ec_slave_count_; ++i) {
        auto* pdo = reinterpret_cast<EcPdoIn*>(ec_slave[i+1].inputs);
        hw_positions_[i] = pdo->actual_position * pos_factor_;
        hw_velocities_[i] = pdo->actual_velocity * vel_factor_;
    }
    return hardware_interface::return_type::OK;
}
```

### 6.2 RS485 — 确定性读写设计

RS485 通常用于较低速率（< 500 Hz）的外设通信。关键是避免阻塞：

```cpp
// ❌ 错误做法：在 RT 线程中直接 select/read（可能阻塞）
hardware_interface::return_type
ChassisHAL::read(const rclcpp::Time&, const rclcpp::Duration&)
{
    char buf[64];
    // ❌ 这会在等待数据时阻塞！
    int n = ::read(serial_fd_, buf, sizeof(buf));
    ...
}

// ✅ 正确做法：独立驱动线程 + 无锁缓冲区
// 驱动线程（非 RT，SCHED_OTHER）负责 RS485 读写
// RT 的 read()/write() 只操作无锁缓冲区
class RS485Driver {
public:
    void driver_thread_func() {
        // 驱动线程：使用 epoll_wait 替代阻塞 read
        struct epoll_event ev, events[1];
        int epoll_fd = epoll_create1(0);
        ev.events = EPOLLIN;
        ev.data.fd = serial_fd_;
        epoll_ctl(epoll_fd, EPOLL_CTL_ADD, serial_fd_, &ev);
        
        while (running_) {
            // 等待 RS485 数据（最多 5ms 超时）
            int nfds = epoll_wait(epoll_fd, events, 1, 5);
            if (nfds > 0) {
                SensorFrame frame;
                if (read_frame(frame)) {
                    rx_buffer_.push(frame);  // 无锁推入
                }
            }
            
            // 发送待发数据
            CommandFrame cmd;
            while (tx_buffer_.try_pop(cmd)) {
                send_frame(cmd);
            }
        }
    }

    // RT 线程调用（无锁，确定性）
    bool try_get_latest(SensorFrame& frame) {
        return rx_buffer_.try_pop(frame);
    }
    
    void enqueue_command(const CommandFrame& cmd) {
        tx_buffer_.push(cmd);
    }

private:
    LockFreeRingBuffer<SensorFrame, 16> rx_buffer_;
    LockFreeRingBuffer<CommandFrame, 16> tx_buffer_;
    int serial_fd_;
    std::atomic<bool> running_{true};
};
```

---

## 七、与本仓库 chassis_protocol 的对接

### 7.1 当前问题分析

根据 `chassis_protocol/CMakeLists.txt` 分析，当前架构：

```
chassis_hal.cpp
  ├─ version_negotiator.cpp    ← 版本协商（含 mutex，不适合 RT）
  ├─ rs485_transport.cpp       ← 阻塞 select()，不适合 RT 线程直接调用
  └─ differential_controller.cpp / mecanum_controller.cpp
```

**主要 RT 不友好点**：
1. `ChassisHal::read()` 可能直接调用 `rs485_transport` 的阻塞读取
2. `version_negotiator` 在每次 `read()` 可能进行协议检查（含动态分配）
3. 没有无锁缓冲层隔离 RS485 I/O 和控制循环

### 7.2 改造设计建议（详见 07_integration.md）

```cpp
// 改造目标：ChassisHal::read() 和 write() 成为真正的 RT 安全函数

// 架构变化：
// 当前：RT 线程 → ChassisHal::read() → rs485_transport::read() → select()（阻塞）
// 改造后：RS485 驱动线程 → 无锁缓冲区 ← RT 线程 read()（O(1)，无阻塞）
```

完整改造路线图见 [`07_integration.md`](./07_integration.md)。
