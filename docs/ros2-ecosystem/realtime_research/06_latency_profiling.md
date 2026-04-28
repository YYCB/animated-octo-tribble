# 延迟分析工具链

> 本文档覆盖具身智能实时控制系统的完整延迟分析工具链，从内核级 trace 到 ROS 2 端到端延迟测量，并提供典型问题的诊断与解决方法。

---

## 一、ros2_tracing + LTTng

### 1.1 架构概述

`ros2_tracing` 是 ROS 2 官方的分布式追踪框架，基于 Linux Trace Toolkit Next Generation（LTTng）实现。它在 ROS 2 关键路径上插入低开销 tracepoint，记录时间戳和上下文：

```
┌──────────────────────────────────────────────────────────────────────┐
│  ros2_tracing 数据流                                                   │
│                                                                      │
│  ROS 2 代码中的 tracepoint（极低开销，~10ns）                          │
│    rclcpp_publish() / rcl_publish() / rmw_publish()                  │
│    executor_execute() / callback_start() / callback_end()            │
│                     ↓ LTTng 内核模块                                  │
│  环形缓冲区（per-CPU，无锁）                                            │
│                     ↓ lttng-sessiond 守护进程                         │
│  CTF（Common Trace Format）二进制文件                                  │
│                     ↓                                                 │
│  tracetools_analysis（Python 后处理）                                  │
│  Trace Compass / Babeltrace2（可视化）                                │
└──────────────────────────────────────────────────────────────────────┘
```

### 1.2 安装

```bash
# 安装 LTTng
sudo apt install lttng-tools lttng-modules-dkms liblttng-ust-dev

# 安装 ros2_tracing
sudo apt install ros-humble-tracetools \
                 ros-humble-tracetools-launch \
                 ros-humble-tracetools-trace \
                 ros-humble-tracetools-analysis \
                 ros-humble-ros2trace

# 验证 LTTng 模块
lttng-sessiond --version
```

### 1.3 ros2 trace 命令

```bash
# 基础追踪（追踪所有 ROS 2 关键路径）
ros2 trace \
    --session-name rt_analysis \
    --path /tmp/ros2_traces \
    --ros-enable \
    --kernel-enable \
    -- ros2 launch my_package rt_control.launch.py

# 只追踪特定 tracepoint（减少开销）
ros2 trace \
    --session-name latency_measure \
    --path ~/traces/latency \
    --events-ust \
        ros2:rclcpp_publish \
        ros2:rcl_publish \
        ros2:rmw_publish \
        ros2:executor_execute \
        ros2:callback_start \
        ros2:callback_end \
    -- ros2 run my_package rt_node

# 在 launch 文件中集成追踪
# tracetools_launch 提供 Trace action
```

### 1.4 关键 Tracepoint 列表

| Tracepoint | 触发时机 | 记录字段 |
|-----------|---------|---------|
| `ros2:rclcpp_publish` | rclcpp Publisher::publish() 调用时 | publisher_handle, message |
| `ros2:rcl_publish` | rcl_publish() 调用时（经过 rmw 前） | publisher_handle, rmw_publisher_handle |
| `ros2:rmw_publish` | rmw 将消息交给 DDS 前 | rmw_publisher_handle, message |
| `ros2:rclcpp_subscription_callback_start` | rclcpp subscription callback 开始 | subscription_handle, is_intra_process |
| `ros2:rclcpp_subscription_callback_end` | rclcpp subscription callback 结束 | subscription_handle |
| `ros2:executor_execute` | Executor 取到 callback 并执行 | handle |
| `ros2:rclcpp_timer_callback_start` | Timer callback 开始 | timer_handle |
| `ros2:rclcpp_timer_callback_end` | Timer callback 结束 | timer_handle |
| `lttng_ust_libc:malloc` | 动态内存分配（RT 检测用） | size, ptr |

### 1.5 端到端延迟计算方法

**定义**：从 Publisher 发出消息，到 Subscriber callback 开始执行的时间差。

```python
# 使用 babeltrace2 提取延迟
from tracetools_analysis.processor import Processor
from tracetools_analysis.utils.ros2 import Ros2DataModelUtil

# 加载追踪数据
processor = Processor.from_path("~/traces/latency")
data = processor.process()

ros2_util = Ros2DataModelUtil(data.data)

# 计算 /joint_states topic 的发布到订阅延迟
pub_sub_latencies = ros2_util.get_publish_to_subscribe_latencies("/joint_states")

for latency_ns in pub_sub_latencies:
    print(f"延迟: {latency_ns / 1000:.1f} μs")
```

---

## 二、tracetools_analysis — 分析脚本

### 2.1 ProcessMessageLatency

```python
# 完整延迟分析脚本
from tracetools_analysis.loading import load_file
from tracetools_analysis.processor import Processor
from tracetools_analysis.ros2 import Ros2Processor
import pandas as pd
import matplotlib.pyplot as plt

# 加载 CTF 追踪数据
events = load_file("~/traces/rt_analysis")

# 处理 ROS 2 事件
ros2_proc = Ros2Processor()
data = ros2_proc.process(events)

# 获取 callback 执行时间统计
callbacks = data.callback_durations
cb_stats = callbacks.groupby("callback_obj").agg({
    "duration": ["mean", "min", "max", "std"]
}).round(3)
print("=== Callback 执行时间统计（μs）===")
print(cb_stats * 1e-3)  # 纳秒转微秒

# 绘制延迟直方图
for topic in ["/joint_states", "/cmd_vel"]:
    latencies = data.get_subscription_latency(topic)
    if latencies is not None:
        plt.hist(latencies / 1000, bins=100, label=topic, alpha=0.7)

plt.xlabel("延迟 (μs)")
plt.ylabel("频次")
plt.title("ROS 2 消息延迟分布")
plt.legend()
plt.savefig("latency_histogram.png", dpi=150)
```

### 2.2 E2ELatency — 端到端延迟

```python
# 端到端延迟：从传感器读取到执行器输出
# 需要在代码中手动插入追踪标记

# 在 hardware_interface read() 中标记（使用 LTTng UST）
from lttng_ust import tracepoint

@tracepoint("my_robot", "sensor_read_start")
def mark_sensor_read_start(joint_idx: int):
    pass

# 在 hardware_interface write() 中标记
@tracepoint("my_robot", "actuator_write_end")  
def mark_actuator_write_end(joint_idx: int):
    pass
```

---

## 三、cyclictest — 完整命令参数

### 3.1 生产级测试配置

```bash
#!/bin/bash
# rt_validation.sh — 生产级 RT 验证脚本

# 参数说明：
#   -t1             : 1 个测试线程
#   -p95            : SCHED_FIFO 优先级 95（接近最高）
#   -n              : 使用 clock_nanosleep（更精确）
#   -i 250          : 唤醒间隔 250μs（4kHz 模拟）
#   -l 1000000      : 循环 100万次（250μs * 1M = 250秒）
#   --affinity=2    : 绑定到 Core 2（RT 核）
#   --mlockall      : 锁定内存
#   --histfile=...  : 保存直方图到文件

sudo cyclictest \
    -t1 \
    -p95 \
    -n \
    -i 250 \
    -l 1000000 \
    --affinity=2 \
    --mlockall \
    --histfile=/var/log/cyclictest_hist.txt \
    --quiet

echo "测试完成，查看直方图："
cat /var/log/cyclictest_hist.txt | head -20
```

### 3.2 直方图输出解读

```
# cyclictest --histogram 输出格式
# 每行：延迟(μs) 次数
000000 0
000001 1523
000002 4821
000003 12847
...
000047 3          ← 最大延迟附近
000048 1          ← 最大延迟 = 48μs（满足 < 50μs 目标）
000049 0

# 关键指标：
# Min = 最小延迟（通常 3-5μs，反映 hrtimer 基础精度）
# Avg = 平均延迟（通常 8-12μs）
# Max = 最大延迟（验收标准：< 50μs for x86 RT）
```

### 3.3 RT 内核验收 Checklist

```bash
# rt_acceptance_test.sh — 完整验收测试

echo "=== Step 1: 确认 PREEMPT_RT 内核 ==="
grep -q "PREEMPT_RT" /proc/version || { echo "❌ 非 RT 内核"; exit 1; }
echo "✅ PREEMPT_RT 内核"

echo "=== Step 2: 检测 SMI 硬件延迟 ==="
sudo hwlatdetect --duration=30 --threshold=20
# 若有 samples above threshold，需要在 BIOS 关闭 SMM

echo "=== Step 3: cyclictest 基线测试（5分钟）==="
MAX_LAT=$(sudo cyclictest -t1 -p90 -n -i1000 -D5m -q --mlockall 2>&1 \
    | grep "Max Latencies" | awk '{print $NF}')
echo "最大延迟: ${MAX_LAT} μs"
[ "$MAX_LAT" -lt 100 ] && echo "✅ 通过（< 100μs）" || echo "❌ 超标"

echo "=== Step 4: 施加压力测试 ==="
# 同时运行磁盘 I/O、网络负载，再次测试
stress-ng --cpu 2 --io 1 --hdd 1 &
STRESS_PID=$!
sudo cyclictest -t1 -p90 -n -i1000 -D2m -q --mlockall --affinity=2
kill $STRESS_PID
```

---

## 四、ros2_performance_test

### 4.1 延迟 / 吞吐基准测试

```bash
# 安装 ros2 performance_test
sudo apt install ros-humble-performance-test

# 发布者和订阅者延迟测试
ros2 run performance_test perf_test \
    --communication ROS2 \
    --topic Array1k \
    --rate 1000 \
    --max_runtime 30 \
    --logfile latency_results

# 参数说明
# --communication: ROS2 / ROS2_ZERO_COPY / ICEORYX / ...
# --topic: Array1k（1KB）/ Array4k / Array16k / PointCloud512k
# --rate: 发布频率 (Hz)
# --max_runtime: 运行时间 (秒)
```

### 4.2 YAML 配置文件测试

```yaml
# performance_test_config.yaml
experiments:
  - name: "joint_states_1kHz"
    communication: "ROS2"
    topic: "Array64"           # ~64B，模拟 JointState
    rate: 1000                 # 1kHz
    max_runtime: 60            # 60秒
    zero_copy: false
    
  - name: "joint_states_iceoryx"
    communication: "ICEORYX"
    topic: "Array64"
    rate: 1000
    max_runtime: 60
    
  - name: "image_30fps"
    communication: "ROS2"
    topic: "Array307200"       # ~300KB，模拟 640×480 RGB 图像
    rate: 30
    max_runtime: 60
```

```bash
# 运行配置文件
ros2 run performance_test perf_test --config performance_test_config.yaml

# 结果输出示例（CSV）：
# T_experiment, latency_min(ms), latency_mean(ms), latency_max(ms), ...
# 1.0, 0.005, 0.012, 0.048, ...
```

---

## 五、perf + ftrace — 内核级分析

### 5.1 function graph tracer — 分析 RT 调度路径

```bash
# 使用 ftrace function_graph 追踪 controller_manager 的调度路径
CM_PID=$(pgrep -f controller_manager)

# 设置 function_graph tracer
echo function_graph | sudo tee /sys/kernel/debug/tracing/current_tracer

# 只追踪 controller_manager 进程
echo $CM_PID | sudo tee /sys/kernel/debug/tracing/set_ftrace_pid

# 设置追踪深度（避免输出爆炸）
echo 3 | sudo tee /sys/kernel/debug/tracing/max_graph_depth

# 开始追踪
echo 1 | sudo tee /sys/kernel/debug/tracing/tracing_on
sleep 1
echo 0 | sudo tee /sys/kernel/debug/tracing/tracing_on

# 读取结果
sudo cat /sys/kernel/debug/tracing/trace | head -100
```

**ftrace 输出示例：**

```
 0)   3.142 us    |  timerfd_read() {
 0)   0.521 us    |    hrtimer_interrupt();
 0)   1.234 us    |    do_nanosleep();
 0)   =========== 
 0)   5.012 us    |  controller_manager_update_cb() {
 0)   0.912 us    |    hardware_interface_read();
 0)   2.345 us    |    controller_update();
 0)   0.785 us    |    hardware_interface_write();
```

### 5.2 perf sched latency — 调度延迟分析

```bash
# 记录调度事件（包含 wakeup-to-run 延迟）
sudo perf sched record -p $CM_PID -- sleep 5

# 分析调度延迟报告
sudo perf sched latency --sort max

# 输出示例：
# -----------------------------------------------
# Task                  |   Runtime ms  | Switches | Average delay ms | Maximum delay ms |
# -----------------------------------------------
# controller_manage:1234|      5001.234 |      5000|             0.005|             0.048|
#   ... (期望 maximum delay < 0.1ms)
```

### 5.3 perf stat — CPU 性能计数器

```bash
# 监控 RT 线程的 CPU 缓存性能（检测 cache miss）
sudo perf stat \
    -e cache-misses,cache-references,instructions,cycles \
    -p $CM_PID \
    -- sleep 5

# 示例输出：
# Performance counter stats for process id '1234':
#      23,456      cache-misses              #    0.12 % of all cache refs
#   19,234,567     cache-references
#  456,789,012     instructions              #    2.34  insn per cycle
#  195,123,456     cycles
```

---

## 六、Foxglove + ros2_tracing 可视化

### 6.1 将 LTTng 追踪数据导入 Foxglove

```python
# 将 CTF 追踪数据转换为 MCAP 格式（Foxglove 支持）
# 方法：使用 babeltrace2 + 自定义插件

# 或使用 ros2_tracing 的 Plotjuggler 集成
# ros2 run tracetools_analysis plot_publisher_latency \
#     --path ~/traces/rt_analysis \
#     --topic /joint_states
```

### 6.2 Foxglove 实时监控面板

在 Foxglove Studio 中配置实时延迟监控：

```json
{
  "layout": {
    "panels": [
      {
        "type": "Plot",
        "title": "控制循环延迟",
        "topics": [
          {
            "topic": "/diagnostics",
            "path": "status[?name=='controller_manager'].values[?key=='update_period_ms'].value",
            "label": "更新周期 (ms)"
          }
        ],
        "yAxisLabel": "延迟 (ms)",
        "maxYValue": 2.0,
        "showLegend": true
      },
      {
        "type": "StateTransitions",
        "title": "控制器状态",
        "topics": ["/controller_manager/lifecycle_state"]
      }
    ]
  }
}
```

---

## 七、具身智能控制环端到端延迟测量

### 7.1 完整测量链路

```
┌────────────────────────────────────────────────────────────────────┐
│  端到端延迟测量链路                                                   │
│                                                                    │
│  ① 传感器中断（外部事件）                                             │
│        ↓ IRQ 线程（SCHED_FIFO 90）                                  │
│  ② hardware_interface read()（最新传感器数据进入 state_interface）   │
│        ↓ 控制循环触发（hrtimer，每 1ms）                             │
│  ③ controller update()（计算控制命令）                               │
│        ↓ 命令写入 command_interface                                 │
│  ④ hardware_interface write()（命令发送到执行器）                    │
│        ↓ RS485/EtherCAT 传输                                       │
│  ⑤ 执行器响应（电机开始执行）                                         │
│                                                                    │
│  端到端延迟 = ⑤ - ① （理论最小值由总线时延决定）                      │
└────────────────────────────────────────────────────────────────────┘
```

### 7.2 硬件时间戳测量方法

```cpp
// 在 hardware_interface 中插入测量点
// 方法 1：LTTng UST tracepoint（推荐，低开销）
#include <lttng/tracepoint.h>

TRACEPOINT_EVENT(
    my_robot,
    hw_read_start,
    TP_ARGS(uint64_t, timestamp_ns),
    TP_FIELDS(ctf_integer(uint64_t, timestamp_ns, timestamp_ns))
)

hardware_interface::return_type ChassisHAL::read(...) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t t_ns = ts.tv_sec * 1e9 + ts.tv_nsec;
    
    tracepoint(my_robot, hw_read_start, t_ns);
    
    // ... 读取传感器数据 ...
    return hardware_interface::return_type::OK;
}
```

```python
# 方法 2：通过 /diagnostics 发布延迟统计（低频，不影响 RT）
# 在 JointStateBroadcaster 中统计并发布延迟信息

class LatencyMonitor:
    def __init__(self):
        self.latencies = collections.deque(maxlen=1000)
    
    def record(self, latency_ns):
        self.latencies.append(latency_ns)
    
    def get_stats(self):
        if not self.latencies:
            return {}
        arr = list(self.latencies)
        return {
            "min_us": min(arr) / 1000,
            "avg_us": sum(arr) / len(arr) / 1000,
            "max_us": max(arr) / 1000,
            "p99_us": sorted(arr)[int(0.99 * len(arr))] / 1000,
        }
```

---

## 八、典型问题诊断与解决

### 8.1 优先级反转（Priority Inversion）

**症状**：`cyclictest` 偶发大延迟（数百 μs），不规律，难以复现

**诊断**：

```bash
# 检查是否有 PI mutex 配置不当
# 查找 RT 线程等待的锁
sudo cat /proc/$CM_PID/wchan   # 若输出 "mutex_lock" 则发生阻塞

# 使用 ftrace 追踪锁等待
echo > /sys/kernel/debug/tracing/trace
echo "funcgraph" > /sys/kernel/debug/tracing/current_tracer
echo "rt_mutex_lock" > /sys/kernel/debug/tracing/set_graph_function
echo 1 > /sys/kernel/debug/tracing/tracing_on
```

**解决方案**：
- 使用带 `PTHREAD_PRIO_INHERIT` 的 pthread_mutex（见 `02_ros2_realtime.md §3.5`）
- 确保 RT 线程中不使用普通 mutex（改用 `realtime_tools::RealtimeBuffer`）
- 检查 DDS 内部锁（将 DDS 线程迁移到非 RT 核）

### 8.2 Cache Miss 导致延迟尖峰

**症状**：延迟周期性出现 50-200μs 的尖峰，与数据量相关

**诊断**：

```bash
# perf 监控 cache miss 率
sudo perf stat -e cache-misses -p $CM_PID sleep 5

# 若 cache miss 率 > 1%，需要优化内存布局
```

**解决方案**：
- 将热路径数据紧凑排列（`alignas(64)` 确保 cacheline 对齐）
- 预取数据（`__builtin_prefetch`）
- 避免在控制循环中分散访问大数据结构

```cpp
// ✅ cacheline 对齐的控制数据
struct alignas(64) ControlState {
    std::array<double, 8> positions;
    std::array<double, 8> velocities;
    std::array<double, 8> commands;
    uint64_t timestamp_ns;
};
```

### 8.3 TLB Shootdown

**症状**：多核系统上偶发的 20-50μs 尖峰

**诊断**：

```bash
# 查看 TLB 刷新频率
sudo perf stat -e dTLB-load-misses,iTLB-load-misses -p $CM_PID sleep 5

# 检查是否有大量内存映射操作（mmap/munmap）
sudo strace -p $CM_PID -e mmap,munmap 2>&1 | head -50
```

**解决方案**：
- 关闭 Transparent HugePages：`echo never > /sys/kernel/mm/transparent_hugepage/enabled`
- 避免 RT 线程触发内存映射操作
- 使用 `MAP_LOCKED` 和 `mlock()` 确保页面驻留

### 8.4 SMI（System Management Interrupt）

**症状**：完全不规律的 100-500μs 尖峰，通常每隔几秒出现一次

**诊断**：

```bash
# hwlatdetect 检测 SMI（见 01_preempt_rt_kernel.md §4.2）
sudo hwlatdetect --duration=60 --threshold=50

# MSR 读取（x86）：统计 SMI 次数
# 检查 MSR_SMI_COUNT (0x34)
sudo modprobe msr
sudo rdmsr 0x34  # 开始时记录
sleep 60
sudo rdmsr 0x34  # 结束时记录，差值 = 期间 SMI 次数
```

**解决方案**：
1. BIOS 中关闭 SMM（System Management Mode）——仅部分工业主板支持
2. 关闭 ACPI 电源管理功能（减少 SMI 来源）
3. 使用 `processor.max_cstate=0 idle=poll` 内核参数
4. 联系主板厂商获取固件更新（修复 SMI storm）

### 8.5 诊断决策树

```
cyclictest 最大延迟超标（> 100μs）？
         │
         ├─ hwlatdetect 检测到大延迟？
         │       ├─ 是：SMI 问题 → 关闭 ACPI/C-States/固件更新
         │       └─ 否：软件层面问题
         │
         ├─ 降低 OS 负载后改善？
         │       ├─ 是：CPU 资源不足 → 增加 isolcpus，检查进程亲和性
         │       └─ 否：内核配置问题
         │
         ├─ PREEMPT_RT 内核？
         │       ├─ 否：先安装 RT 内核
         │       └─ 是：继续排查
         │
         ├─ irqbalance 已禁用？
         │       ├─ 否：sudo systemctl disable irqbalance
         │       └─ 是：继续排查
         │
         └─ CPU 频率固定？（非动态调频）
                 ├─ 否：sudo cpupower frequency-set -g performance
                 └─ 是：深入内核 trace 分析（ftrace/perf）
```
