# 08 阶段 6：实时化 —— PREEMPT_RT、CPU 隔离、IRQ 亲和

> **目标**：把"平均快"变成"**最差情况可预测**"。控制环不仅需要 P50 低，更需要 P99.99 不超标。
>
> **重要**：本阶段可以**与阶段 1–4 任何时候并行**，并非最后才做。**一个好的实时内核 + CPU 隔离能把前面所有优化的效果放大 2–5 倍**。

---

## 8.1 实时性的层次

```
用户代码                    <-- 你的控制算法
rclcpp / rcl / rmw / DDS    <-- 中间件（阶段 1–5）
 │
 │  系统调用、内存分配、锁
 ▼
glibc / pthread            <-- C 库（malloc 非实时！）
 │
 ▼
Linux 内核                 <-- 本阶段的主战场
 ├─ 调度器（CFS / RT）
 ├─ 时钟（HRTIMER / HPET / TSC）
 ├─ 中断处理（硬中断 / 软中断）
 ├─ 内存（page fault / swap）
 └─ 同步原语（spinlock / mutex）
 │
 ▼
硬件                       <-- CPU 亲和、NUMA、缓存
```

**实时性差的征兆**：P99 延迟正常，但 P99.99 / 最大值偶尔超标 10 倍以上——说明存在**稀有事件**（page fault、CPU 迁移、IRQ 抢占）。

---

## 8.2 PREEMPT_RT 内核

### 8.2.1 原理

标准 Linux 内核的调度抖动来源：
- **不可抢占内核段**（spinlock、中断禁用）：可达毫秒级；
- **软中断**（ksoftirqd）：批量处理，延迟不可控；
- **锁的优先级反转**：低优先级持锁阻塞高优先级。

PREEMPT_RT 的改造：
- 大部分 spinlock → `rt_mutex`（带优先级继承）；
- 硬中断线程化（可被抢占）；
- Threaded IRQ handlers；
- 更细粒度的可抢占点。

### 8.2.2 安装

Ubuntu 22.04/24.04 已有官方 RT 变体：
```bash
# 通过 Ubuntu Pro（免费个人使用）
sudo pro attach <token>
sudo pro enable realtime-kernel
```

或自行编译（Debian 源码 + RT patch）。

### 8.2.3 验证

```bash
uname -v  # 应含 PREEMPT_RT
cyclictest -l 1000000 -m -Sp 90 -i 1000 -h 400 -q
# 运行 10–60 min，看 max latency 是否 < 100 μs
```

典型结果：
- 非 RT 内核：max ~ 500–2000 μs；
- PREEMPT_RT + isolcpus：max ~ 30–80 μs。

---

## 8.3 进程调度策略

### 8.3.1 SCHED_FIFO / SCHED_RR

- `SCHED_FIFO`：FIFO 运行直到阻塞或主动让出；
- `SCHED_RR`：同优先级内时间片轮转（通常不用）；
- 优先级 1–99（99 最高，保留给内核线程用 90 以下）；
- 需要 `CAP_SYS_NICE` 或 `rtprio` ulimit。

**建议优先级分配**：
```
99  （保留给内核关键线程）
90  控制环 timer 线程
85  DDS receive listener（若不可绑 SCHED_OTHER）
80  感知管线
--- SCHED_OTHER 之上分界 ---
 0  普通回调、ROS 工具
-10 诊断 / 日志 / rosbag
```

### 8.3.2 SCHED_DEADLINE

- 基于 runtime/deadline/period 的 EDF 调度；
- **资源预留**：保证在每个 period 内至少获得 runtime 的 CPU 时间；
- 更适合**周期性任务**（如 500 Hz 控制环）。

```cpp
struct sched_attr attr;
memset(&attr, 0, sizeof(attr));
attr.size = sizeof(attr);
attr.sched_policy = SCHED_DEADLINE;
attr.sched_runtime  =  500'000;  // 500 μs
attr.sched_deadline = 2'000'000; // 2 ms
attr.sched_period   = 2'000'000;
syscall(SYS_sched_setattr, 0, &attr, 0);
```

**代价**：内核必须 admission control 通过才允许设置——总 CPU 预留不能 > 95%。

### 8.3.3 优先级继承陷阱

pthread_mutex 默认不带优先级继承——SCHED_FIFO 90 持锁，SCHED_FIFO 70 等锁，期间 SCHED_OTHER 的进程抢占 90（因为内核认为"反正它被锁了"）——**优先级反转**。

**对策**：
```cpp
pthread_mutexattr_t attr;
pthread_mutexattr_init(&attr);
pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
pthread_mutex_init(&m, &attr);
```

---

## 8.4 CPU 隔离

### 8.4.1 isolcpus（传统方式）

内核启动参数（`/etc/default/grub` 的 `GRUB_CMDLINE_LINUX_DEFAULT`）：
```
isolcpus=4,5,6,7 nohz_full=4,5,6,7 rcu_nocbs=4,5,6,7
```
- `isolcpus`：这些核不参与 CFS 负载均衡，调度器不会往上放普通进程；
- `nohz_full`：tickless 模式，不产生调度时钟中断；
- `rcu_nocbs`：RCU callback 不在这些核上执行。

将 ROS 2 进程通过 `taskset -c 4-7` 启动。

### 8.4.2 cpuset cgroup（推荐）

比 isolcpus 更灵活：
```bash
sudo cgcreate -g cpuset:/rt
echo 4-7 | sudo tee /sys/fs/cgroup/rt/cpuset.cpus
echo 0 | sudo tee /sys/fs/cgroup/rt/cpuset.mems
# 移动关键进程
echo <pid> | sudo tee /sys/fs/cgroup/rt/cgroup.procs
```

### 8.4.3 NUMA

多路 CPU / 大型服务器要意识 NUMA：
- 通过 `numactl --cpubind=0 --membind=0` 绑定进程；
- 控制线程 + 其访问的 SHM pool 应在同一 NUMA 节点；
- 跨 NUMA 访问延迟 +100–200 ns / 次。

---

## 8.5 IRQ 亲和

把网卡 / 磁盘 / DMA 的中断**从控制核搬走**：

```bash
# 查看当前 IRQ 分布
cat /proc/interrupts
# 把 IRQ 42 绑定到核 0, 1
echo 3 | sudo tee /proc/irq/42/smp_affinity  # 3 = bitmask 0011
```

更完整的方案是 `irqbalance` 的排除列表：
```
# /etc/default/irqbalance
IRQBALANCE_BANNED_CPUS=f0   # bitmask: 核 4-7 不接受 IRQ
```

**验证**：`watch -n1 cat /proc/interrupts`，确认隔离核的 IRQ 计数稳定不增长。

---

## 8.6 内存与分页

### 8.6.1 mlockall

```cpp
if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
  // 需要 memlock ulimit unlimited
}
```
防止控制进程的任何页被换出。

### 8.6.2 预填充堆栈

```cpp
void prefault_stack(size_t size = 8 * 1024 * 1024) {
  unsigned char dummy[size];
  memset(dummy, 0, size);
}
```
避免第一次深调用触发 page fault。

### 8.6.3 Hugepages

减少 TLB miss：
```bash
echo 512 | sudo tee /proc/sys/vm/nr_hugepages
```
然后 mmap 时带 `MAP_HUGETLB`。对大消息缓冲 / SHM pool 效果显著（TLB miss 降 90%+）。

### 8.6.4 关闭 THP（Transparent Huge Pages）

THP 的合并操作是异步扫描进程——会产生不可预测的延迟。RT 系统**必须关**：
```bash
echo never | sudo tee /sys/kernel/mm/transparent_hugepage/enabled
echo never | sudo tee /sys/kernel/mm/transparent_hugepage/defrag
```

---

## 8.7 BIOS / 硬件层

- **关 HyperThreading**（SMT）：一对逻辑核共享执行单元，抖动大。关闭换确定性；
- **关 Turbo / P-states**：`cpupower frequency-set -g performance`，固定最高频；
- **关 C-states**：BIOS 关深度节能，或内核参数 `idle=poll intel_idle.max_cstate=0 processor.max_cstate=1`（代价：功耗↑）；
- **关 SpeedStep / Intel Enhanced SpeedStep**；
- **System Management Interrupts (SMI)**：某些主板的管理固件会偷走几百 μs——选 "RT-friendly" 主板（Supermicro / ADLINK MXE 系列）。

---

## 8.8 验证方法

### 8.8.1 cyclictest（裸机延迟）

```bash
cyclictest -l 100000000 -m -Sp 90 -i 200 -h 1000 -q > cyclic.log
```
跑 24 小时，看最差 latency。目标：**< 50 μs**。

### 8.8.2 应用层延迟

结合 ros2_tracing 与 LTTng：
```bash
ros2 trace -s my_session -e ros2:*
# 运行负载
ros2 trace -s my_session stop
babeltrace2 ~/.ros/tracing/my_session | analyze_latency.py
```

输出：每个 callback 的 P50/P99/P99.99 + 分布直方图。

### 8.8.3 负载干扰测试

在隔离核以外放 stress：
```bash
stress-ng --cpu 4 --cpu-method all --timeout 600
```
同时跑控制环，P99 应该**几乎不变**——如果变了，说明隔离未生效。

---

## 8.9 本阶段结束准则

| 指标 | 目标 |
|------|------|
| cyclictest max (24h) | < 50 μs |
| 500 Hz 控制环 P99 抖动 | < 50 μs |
| 500 Hz 控制环 P99.99 | < 100 μs |
| 干扰负载下 P99 变化 | < 10% |

达到以上指标 → **本阶段完成**。通常可以在此停步，与阶段 1–4 的优化结果汇合，上线。

---

## 8.10 维护成本

- 改动面：**内核配置 + 系统服务 + 启动脚本**；
- 与上游兼容性：**不影响 ROS 2 代码**，但依赖特定内核版本；
- 每次内核升级：**1–2 周**（验证 cyclictest + 应用层回归）；
- 典型团队规模：**0.5 人**（系统工程师兼任）；
- 硬件成本：可能需要更换 BIOS 友好的主板 + NIC；
- **这是 ROI 最高的阶段之一**——对延迟敏感业务必做。

---

## 总结：阶段 1–6 的叠加效果

| 指标 | 原始 | +P1 | +P2 | +P3 | +P4 | +P6 |
|------|------|-----|-----|-----|-----|-----|
| 1 KB 同机 P99 | 500 μs | 180 μs | 30 μs | 25 μs | — | **15 μs** |
| 1 MB 同机 P99 | 10 ms | 6 ms | 4 ms | 300 μs | 15 μs | **10 μs** |
| 500 Hz 控制抖动 σ | 300 μs | 120 μs | 50 μs | — | — | **20 μs** |
| 40 节点冷启动 | 45 s | 6 s | — | — | — | — |

**注意**：阶段 5（替换 DDS）不在表中——对同机场景收益边际很小。

下一章：[09 维护策略](./09_maintenance_strategy.md)。
