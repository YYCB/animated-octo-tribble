# ROS 2 实时化全指南

> 本文档覆盖将 ROS 2 节点部署在 PREEMPT_RT 系统上所需的全套技术：内存管理、CPU 隔离、线程优先级、Executor 选择、realtime_tools 库的正确使用以及零拷贝 IPC。

---

## 一、内存管理：防止 Page Fault

Page fault（缺页中断）是 RT 系统中最常见的延迟尖峰来源之一。当进程访问尚未加载到物理内存的页面时，OS 需要从磁盘（或 swap）加载，可能产生毫秒级延迟。

### 1.1 mlockall — 锁定所有内存页

```cpp
#include <sys/mman.h>

// 在 main() 最开头，或 on_activate() 中调用
// MCL_CURRENT：锁定当前已分配的内存
// MCL_FUTURE： 锁定未来分配的内存（包括栈增长）
if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    RCLCPP_ERROR(logger, "mlockall 失败，errno=%d", errno);
    // 通常需要 root 权限或 CAP_IPC_LOCK capability
    return false;
}
```

**权限设置（非 root 运行）：**

```bash
# 方法 1：设置 capability
sudo setcap cap_ipc_lock+ep /path/to/ros2_node

# 方法 2：修改 ulimit（/etc/security/limits.conf）
# 添加：
# * soft memlock unlimited
# * hard memlock unlimited

# 方法 3：systemd unit 中设置
# LimitMEMLOCK=infinity
```

### 1.2 栈预分配

RT 线程在第一次访问深层栈帧时会触发 page fault。预先"触摸"栈：

```cpp
#include <string.h>

// 预分配 8MB 栈空间（在进入 RT 循环前调用）
void prefault_stack() {
    constexpr size_t STACK_SIZE = 8 * 1024 * 1024;  // 8MB
    volatile char stack_mem[STACK_SIZE];
    // 使用 memset 确保编译器不优化掉这个操作
    memset(const_cast<char*>(stack_mem), 0, STACK_SIZE);
}
```

### 1.3 堆预分配与自定义分配器

RT 线程中绝对禁止动态内存分配（`malloc` / `new` / `std::vector::push_back` 触发重分配）。

#### 使用 tlsf_cpp 自定义分配器

`tlsf_cpp` 提供 O(1) 的 Two-Level Segregated Fit 内存分配器，适合在有限动态分配场景中替代默认分配器：

```cpp
// 安装：ros-humble-tlsf-cpp
// 头文件：#include <tlsf_cpp/tlsf.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tlsf_cpp/tlsf.hpp>

// 创建使用 TLSF 分配器的 Publisher
rclcpp::PublisherOptionsWithAllocator<TLSFAllocator<void>> pub_options;
pub_options.allocator = std::make_shared<TLSFAllocator<void>>();

auto publisher = node->create_publisher<MsgType>(
    "topic", 10, pub_options);
```

#### RT 节点中的预分配模式

```cpp
class RTControlNode : public rclcpp::Node {
public:
    RTControlNode() : Node("rt_control_node") {
        // on_configure 中预分配所有需要的容量
        joint_positions_.reserve(MAX_JOINTS);   // 不触发运行时分配
        joint_velocities_.reserve(MAX_JOINTS);
        command_buffer_.reserve(MAX_JOINTS);

        // 预构建消息对象（复用而非每次 new）
        joint_state_msg_ = std::make_shared<sensor_msgs::msg::JointState>();
        joint_state_msg_->position.reserve(MAX_JOINTS);
        joint_state_msg_->velocity.reserve(MAX_JOINTS);
        joint_state_msg_->effort.reserve(MAX_JOINTS);
    }

private:
    static constexpr size_t MAX_JOINTS = 8;
    std::vector<double> joint_positions_;
    std::vector<double> joint_velocities_;
    std::vector<double> command_buffer_;
    sensor_msgs::msg::JointState::SharedPtr joint_state_msg_;
};
```

---

## 二、CPU 隔离与线程亲和性

### 2.1 isolcpus 内核参数

在 `/etc/default/grub` 中设置（参见 `01_preempt_rt_kernel.md §3.2`），将特定 CPU 核从 Linux 调度器隔离：

```bash
GRUB_CMDLINE_LINUX="... isolcpus=2,3 nohz_full=2,3 rcu_nocbs=2,3 ..."
```

验证隔离状态：

```bash
cat /sys/devices/system/cpu/isolated   # 应输出 "2-3"
cat /sys/devices/system/cpu/nohz_full  # 应输出 "2-3"
```

### 2.2 cpuset cgroup 隔离

比 `isolcpus` 更灵活，可运行时调整：

```bash
# 创建 RT 控制专用 cpuset
sudo mkdir -p /sys/fs/cgroup/cpuset/rt_control
echo "2-3" | sudo tee /sys/fs/cgroup/cpuset/rt_control/cpuset.cpus
echo "0"   | sudo tee /sys/fs/cgroup/cpuset/rt_control/cpuset.mems

# 将 controller_manager 进程加入 cpuset
RT_PID=$(pgrep -f controller_manager)
echo $RT_PID | sudo tee /sys/fs/cgroup/cpuset/rt_control/tasks
```

### 2.3 taskset — 命令行线程绑定

```bash
# 在 Core 2,3 上启动 ros2_control_node
taskset -c 2,3 ros2 run controller_manager ros2_control_node

# 对已运行进程设置亲和性
taskset -cp 2,3 <PID>
```

### 2.4 pthread_setaffinity_np — 代码中精确控制线程亲和性

```cpp
#include <pthread.h>
#include <sched.h>

// 将当前线程绑定到 Core 2
void pin_thread_to_core(int core_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);

    pthread_t current_thread = pthread_self();
    int rc = pthread_setaffinity_np(
        current_thread, sizeof(cpu_set_t), &cpuset);
    if (rc != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rt"), 
            "pthread_setaffinity_np 失败：%s", strerror(rc));
    }
}

// 在 executor spin 前调用
void run_rt_executor() {
    pin_thread_to_core(2);
    set_thread_priority(SCHED_FIFO, 80);
    executor.spin();
}
```

### 2.5 将 DDS 线程绑定到非 RT 核

DDS 内部有多个后台线程（Discovery、EventThread、RecvThread 等），它们不应占用 RT 核：

```bash
# 通过 DDS 配置（CycloneDDS 示例）或进程启动后手动迁移
# 启动 controller_manager 后，找到 DDS 线程并将其迁移
for tid in $(ls /proc/<CM_PID>/task/); do
    comm=$(cat /proc/<CM_PID>/task/$tid/comm 2>/dev/null)
    if [[ "$comm" == *"dds"* ]] || [[ "$comm" == *"recv"* ]]; then
        taskset -cp 0,1 $tid
    fi
done
```

---

## 三、线程优先级配置

### 3.1 SCHED_FIFO vs SCHED_RR

| 调度策略 | 行为 | RT 场景适用性 |
|---------|------|------------|
| `SCHED_FIFO` | 先进先出，直到主动让出或被更高优先级抢占 | **首选**：控制循环，一旦获得 CPU 时间运行到完成 |
| `SCHED_RR` | 轮转，同优先级任务按时间片轮转 | 同优先级多任务场景（少见） |
| `SCHED_DEADLINE` | 最早截止时间优先（EDF），需指定 runtime/deadline/period | 高级场景，配置复杂 |
| `SCHED_OTHER` | 默认 CFS 调度 | 非 RT 任务（DDS/录制/推理） |

### 3.2 pthread_setschedparam — 代码中设置优先级

```cpp
#include <pthread.h>
#include <sched.h>

// 设置线程调度策略和优先级
bool set_thread_priority(int policy, int priority) {
    sched_param param;
    param.sched_priority = priority;

    // policy: SCHED_FIFO (1) 或 SCHED_RR (2)
    if (pthread_setschedparam(pthread_self(), policy, &param) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rt"),
            "设置线程优先级失败（需要 root 或 CAP_SYS_NICE）：%s",
            strerror(errno));
        return false;
    }
    return true;
}

// ROS 2 Executor 线程中使用（在 spin() 调用前，在独立线程中设置）
auto rt_thread = std::thread([&executor]() {
    set_thread_priority(SCHED_FIFO, 80);
    pin_thread_to_core(2);
    executor.spin();
});
```

### 3.3 chrt 命令行工具

```bash
# 对已运行进程/线程设置优先级
sudo chrt -f -p 80 <PID>          # SCHED_FIFO prio 80
sudo chrt -r -p 80 <PID>          # SCHED_RR prio 80

# 以特定优先级启动进程
sudo chrt -f 80 ros2 run controller_manager ros2_control_node

# 查看当前线程调度策略
chrt -p <PID>
```

### 3.4 优先级分配建议

```
RT 优先级（1-99，高优先级先运行）：

prio 99  │  内核 RT 线程（不可用）
prio 90  │  硬件中断线程（EtherCAT ISR）
prio 85  │  controller_manager update() 线程
prio 80  │  hardware_interface（高优先级传感器）
prio 70  │  StaticSingleThreadedExecutor（ROS 2 RT 节点）
prio 60  │  realtime_tools::RealtimePublisher 发布线程
prio 50  │  内核 ksoftirqd（默认）
prio 40  │  DDS EventThread
─────────┤
prio 0   │  SCHED_OTHER（默认，所有非 RT 任务）
         │  rosbag2 录制、VLA 推理、Foxglove 监控
```

### 3.5 优先级继承（Priority Inheritance）

PREEMPT_RT 的 rtmutex 支持优先级继承，避免优先级反转：

```cpp
// 示例：高优先级 RT 线程等待被低优先级线程持有的锁时，
// 低优先级线程的优先级临时提升到高优先级（rtmutex 自动处理）
// 需要使用 pthread_mutex_t 配合 PTHREAD_PRIO_INHERIT 属性

pthread_mutexattr_t attr;
pthread_mutexattr_init(&attr);
pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
pthread_mutex_t pi_mutex;
pthread_mutex_init(&pi_mutex, &attr);
```

---

## 四、Executor 实时特性对比

ROS 2 的 Executor 负责调度 callback 的执行，选择错误的 Executor 是导致 RT 抖动的常见原因。

### 4.1 Executor 对比表

| Executor | 线程模型 | 实时适用性 | 延迟特性 | 适用场景 |
|---------|---------|----------|---------|--------|
| `SingleThreadedExecutor` | 单线程，轮询 wait_set | ⚠️ 中等 | wait_set 轮询开销，有抖动 | 简单节点 |
| `MultiThreadedExecutor` | 多线程，动态分配 | ❌ 不适合 | 线程创建/销毁，mutex 竞争 | 并发 I/O |
| **`StaticSingleThreadedExecutor`** | **单线程，静态 wait_set** | **✅ 推荐** | **最小开销，无动态分配** | **RT 控制节点** |
| `EventsExecutor` | 事件驱动，无轮询 | ✅ 推荐（新版） | 最低延迟，IRQ-driven | Jazzy+ 版本 |

### 4.2 StaticSingleThreadedExecutor 使用

```cpp
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/executors/static_single_threaded_executor.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // 使用 StaticSingleThreadedExecutor
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    auto node = std::make_shared<RTControlNode>();
    executor.add_node(node);

    // 在独立线程中以 RT 优先级运行
    auto rt_thread = std::thread([&]() {
        // 设置 RT 属性
        mlockall(MCL_CURRENT | MCL_FUTURE);
        set_thread_priority(SCHED_FIFO, 80);
        pin_thread_to_core(2);
        prefault_stack();

        // 进入 RT 循环
        executor.spin();
    });

    rt_thread.join();
    rclcpp::shutdown();
    return 0;
}
```

### 4.3 Intra-Process Communication（IPC）对 RT 的影响

同进程内的 Publisher → Subscriber 通信，启用 IPC 可绕过 DDS 序列化：

```cpp
// 创建节点时启用 IPC
rclcpp::NodeOptions options;
options.use_intra_process_comms(true);
auto node = std::make_shared<MyNode>(options);

// IPC 路径：
// Publisher::publish() → shared_ptr 传递 → Subscription callback
// 无序列化、无 DDS 开销，延迟 < 1μs（同线程）或 < 10μs（跨线程）
```

---

## 五、realtime_tools 库

`realtime_tools` 是 ros2_control 生态中专为 RT 场景设计的工具库，提供无锁数据交换原语。

### 5.1 RealtimeBuffer — RT 与非 RT 线程间数据交换

```cpp
#include <realtime_tools/realtime_buffer.h>

// 场景：非 RT 线程（DDS callback）写入，RT 控制线程读取
realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist> cmd_vel_buffer_;

// 非 RT 线程（DDS 接收 callback）中写入
void cmd_vel_callback(geometry_msgs::msg::Twist::SharedPtr msg) {
    cmd_vel_buffer_.writeFromNonRT(msg);  // 无锁写入（双缓冲）
}

// RT 控制线程（controller update() 中）读取
void update(const rclcpp::Time& time, const rclcpp::Duration& period) {
    auto cmd = cmd_vel_buffer_.readFromRT();  // 无锁读取
    if (cmd) {
        // 使用 cmd->linear.x, cmd->angular.z
    }
}
```

**RealtimeBuffer 原理**：
- 使用三缓冲（triple buffer）或 double buffer 实现
- 写入端：复制数据到写缓冲，原子交换指针
- 读取端：直接读取当前指针，无 mutex，无 blocking
- 保证：RT 线程读取不会阻塞

### 5.2 RealtimePublisher — RT 线程安全发布

直接在 RT 线程中调用 `publisher->publish()` 会触发 DDS 内部的动态分配和 mutex。`RealtimePublisher` 将发布操作卸载到非 RT 线程：

```cpp
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/msg/joint_state.hpp>

class RTController {
    // 构造函数中
    rt_pub_ = std::make_shared<realtime_tools::RealtimePublisher<
        sensor_msgs::msg::JointState>>(node, "joint_states", 10);

    // update() 中（RT 线程）
    void update(const rclcpp::Time& time, const rclcpp::Duration& period) {
        if (rt_pub_->trylock()) {  // 非阻塞 trylock
            auto& msg = rt_pub_->msg_;
            msg.header.stamp = time;
            msg.position = current_positions_;  // 假设 reserve 过
            rt_pub_->unlockAndPublish();  // 异步发布，RT 线程不等待
        }
        // trylock 失败时跳过本次发布（上一次消息还未发出）
    }

    std::shared_ptr<realtime_tools::RealtimePublisher<
        sensor_msgs::msg::JointState>> rt_pub_;
};
```

### 5.3 RealtimeBox — 可选值容器

```cpp
#include <realtime_tools/realtime_box.h>

// 类似 RealtimeBuffer，但支持 std::optional 语义
realtime_tools::RealtimeBox<ControlCommand> cmd_box_;

// 写入（非 RT）
cmd_box_.set(ControlCommand{1.0, 0.5, 0.0});

// 读取（RT）
ControlCommand cmd;
if (cmd_box_.get(cmd)) {
    // 使用 cmd
}
```

---

## 六、零拷贝 Intra-Process：Loaned Messages

### 6.1 Publisher::borrow_loaned_message()

Loaned Message 机制允许 Publisher 直接从 DDS 或 iceoryx 的共享内存中"借"一块内存，写入后无需拷贝即可发布：

```cpp
// 需要 RMW 支持（rmw_iceoryx / CycloneDDS with SHM）
auto loaned_msg = publisher->borrow_loaned_message();
loaned_msg.get().header.stamp = now();
loaned_msg.get().position[0] = joint_pos;
publisher->publish(std::move(loaned_msg));
// 发布后 loaned_msg 归还，零拷贝完成
```

### 6.2 TypeAdapter — 避免消息类型转换开销

```cpp
// TypeAdapter 允许用户定义类型直接映射到 ROS 消息，零拷贝转换
// REP-2007 实现

template<>
struct rclcpp::TypeAdapter<MyEigenMatrix, std_msgs::msg::Float64MultiArray> {
    using is_specialized = std::true_type;
    using custom_type = MyEigenMatrix;
    using ros_message_type = std_msgs::msg::Float64MultiArray;

    static void convert_to_ros_message(
        const custom_type& source,
        ros_message_type& destination)
    {
        // 仅在必要时（跨进程/跨 RMW）调用
        destination.data.assign(source.data(), source.data() + source.size());
    }
    // ...
};
```

---

## 七、完整 RT 节点模板

```cpp
// rt_control_node.cpp — 完整实时控制节点模板

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/static_single_threaded_executor.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <sys/mman.h>
#include <pthread.h>
#include <sched.h>

class RTControlNode : public rclcpp::Node {
public:
    explicit RTControlNode(const rclcpp::NodeOptions& options)
    : Node("rt_control_node", options)
    {
        // ── 预分配 ────────────────────────────────────────────────────
        constexpr size_t N_JOINTS = 6;
        state_msg_ = std::make_shared<sensor_msgs::msg::JointState>();
        state_msg_->name.reserve(N_JOINTS);
        state_msg_->position.reserve(N_JOINTS);
        state_msg_->velocity.reserve(N_JOINTS);
        state_msg_->effort.reserve(N_JOINTS);

        // ── Publisher（通过 RealtimePublisher 异步发布）────────────────
        rt_pub_ = std::make_shared<
            realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
            this->shared_from_this(), "joint_states", 10);

        // ── Subscription（非 RT，写入 RT Buffer）─────────────────────
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", rclcpp::SensorDataQoS(),
            [this](geometry_msgs::msg::Twist::SharedPtr msg) {
                cmd_buffer_.writeFromNonRT(msg);  // 非 RT 线程写入
            });

        // ── Timer（1kHz 控制循环）──────────────────────────────────────
        using namespace std::chrono_literals;
        timer_ = this->create_wall_timer(1ms, [this]() { this->control_loop(); });
    }

private:
    void control_loop() {
        // ── 读取最新指令（无锁）────────────────────────────────────────
        auto cmd = cmd_buffer_.readFromRT();

        // ── 执行控制算法（此处为 RT 核心）────────────────────────────
        // 禁止：malloc / new / printf / mutex blocking / 任何 syscall（除 clock_gettime）
        if (cmd) {
            // 简单示例：前向命令
            compute_joint_commands(cmd->linear.x, cmd->angular.z);
        }

        // ── 发布状态（非阻塞，RT 线程不等待 DDS）────────────────────
        if (rt_pub_->trylock()) {
            rt_pub_->msg_.header.stamp = this->now();
            // 拷贝状态数据（使用 assign，容量已预分配）
            rt_pub_->msg_.position.assign(
                current_positions_.begin(), current_positions_.end());
            rt_pub_->unlockAndPublish();
        }
    }

    void compute_joint_commands(double vx, double wz) {
        // RT 安全的控制算法（仅用栈变量和预分配的成员变量）
        double r = wheel_radius_;
        double d = wheel_base_;
        current_positions_[0] += (vx - wz * d / 2.0) / r * dt_;
        current_positions_[1] += (vx + wz * d / 2.0) / r * dt_;
    }

    // ── 数据成员 ──────────────────────────────────────────────────
    realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist::SharedPtr> cmd_buffer_;
    std::shared_ptr<realtime_tools::RealtimePublisher<
        sensor_msgs::msg::JointState>> rt_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::JointState::SharedPtr state_msg_;

    std::array<double, 2> current_positions_{0.0, 0.0};
    static constexpr double wheel_radius_ = 0.1;
    static constexpr double wheel_base_ = 0.5;
    static constexpr double dt_ = 0.001;  // 1ms
};

// ── 主函数 ─────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // 内存锁定（需要 root 或 CAP_IPC_LOCK）
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        RCLCPP_WARN(rclcpp::get_logger("main"),
            "mlockall 失败，RT 性能可能受影响");
    }

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);  // 启用 IPC（同进程内零拷贝）
    auto node = std::make_shared<RTControlNode>(options);

    // 使用 StaticSingleThreadedExecutor
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(node);

    // 在独立 RT 线程中运行 Executor
    auto rt_thread = std::thread([&executor]() {
        // 设置线程调度属性
        struct sched_param param;
        param.sched_priority = 80;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
            RCLCPP_WARN(rclcpp::get_logger("rt_thread"),
                "SCHED_FIFO 设置失败（需要 root）");
        }

        // 绑定到 RT 核心
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(2, &cpuset);  // Core 2
        pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

        // 预触摸栈（防止首次 page fault）
        volatile char stack_probe[8 * 1024 * 1024];
        memset(const_cast<char*>(stack_probe), 0, sizeof(stack_probe));

        // 进入实时控制循环
        executor.spin();
    });

    rt_thread.join();
    rclcpp::shutdown();
    return 0;
}
```
