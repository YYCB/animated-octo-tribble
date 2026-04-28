# 03 阶段 1：配置级优化 —— 不动一行代码的 50% 收益

> **目标**：在**完全不 fork 任何仓库**、不改任何上游代码的前提下，通过环境变量、XML profile、QoS 覆盖、编译选项、内核参数，取得 30–60% 的延迟/抖动改进。
>
> **适用决策**：这一阶段的所有改动都应该先做、先测。如果在这一阶段之后性能仍不满足需求，再考虑 Fork。

---

## 本阶段的设计哲学

**"配置即代码"**：每一个可通过 XML / env / launch 参数配置的项，都应该先穷尽，再动代码。原因：
- 上游的 QoS / DDS profile / 编译选项已经覆盖了**绝大多数高性能场景**——工程师把能暴露的旋钮都暴露了，没暴露的才去改代码；
- 配置级改动**不污染上游追赶路径**，切换 ROS 2 发行版时只需重新验证配置；
- 配置级改动**可以做 A/B 测试**——快速回滚、精细观察。

---

## 3.1 rmw 实现选择

### 3.1.1 FastDDS vs CycloneDDS vs Connext

| 指标 | FastDDS | CycloneDDS | Connext |
|------|---------|-----------|---------|
| 默认发行版 | Humble 默认 | 某些发行默认（如 Galactic） | 商业 |
| 延迟 P99（1 KB 同机） | ~350 μs | ~200 μs | ~180 μs |
| 抖动 σ | 中 | **低**（单线程 reactor） | 低 |
| 吞吐（大消息） | 高（需调优） | 中 | **高** |
| SHM 传输 | ✅ 原生 + Iceoryx | ✅ 原生 | ✅ 原生 |
| 静态发现 | 有（Discovery Server） | 有（peers list） | 有 |
| 生态 & 调试工具 | 好 | 好 | **最好** |
| 成本 | 开源 | 开源 | 商业许可 |

**决策建议**：
- **控制环场景 / 抖动敏感** → CycloneDDS
- **大消息吞吐 / 多播密集** → FastDDS（调优后）
- **预算充足 / 需要最强支持** → Connext
- **想用 Zenoh** → 看阶段 5

**切换方式**（不 fork，一行环境变量）：
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# or
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

### 3.1.2 rmw 实现内部设置

不同 DDS 还有各自的 env / XML 调优。例如 FastDDS：
- `FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/profile.xml`（加载自定义 participant profile）
- `ROS_DISCOVERY_SERVER=<ip>:<port>`（启用 Discovery Server 模式，阶段 1 最大单项收益之一）

CycloneDDS：
- `CYCLONEDDS_URI=file:///path/to/cyclonedds.xml`
- 典型调优：`<SharedMemory enable="true"/>`、`<Peers>` 静态列表

---

## 3.2 QoS 调优矩阵

### 3.2.1 按流量角色分类

**控制流**（低延迟、高频、小消息）：
```yaml
# 适用：joint command, IMU, cmd_vel, 高频传感器
reliability: BEST_EFFORT
history: KEEP_LAST, depth=1
durability: VOLATILE
deadline: <控制周期的 1.5 倍>
```

**感知流**（大消息、中频、允许丢帧）：
```yaml
# 适用：camera, lidar, pointcloud
reliability: BEST_EFFORT
history: KEEP_LAST, depth=1   # 不要用默认 10，大消息会 10 MB/订阅
durability: VOLATILE
```

**命令流**（关键指令、必须送达）：
```yaml
# 适用：mission commands, emergency stop
reliability: RELIABLE
history: KEEP_LAST, depth=10
durability: TRANSIENT_LOCAL   # 新订阅也能收到最近一条
```

**监控流**（诊断、日志）：
```yaml
reliability: RELIABLE
history: KEEP_LAST, depth=100
durability: VOLATILE
```

### 3.2.2 QoS Override 机制（不改代码）

从 Iron 起，rclcpp 支持通过参数覆盖 QoS：
```yaml
/my_node:
  ros__parameters:
    qos_overrides:
      /scan:
        subscription:
          reliability: best_effort
          history: keep_last
          depth: 1
```
配合 `NodeOptions::allow_undeclared_parameters(false)` + `automatically_declare_parameters_from_overrides(true)` 启用。

节点代码中需要在 `create_subscription` 时传 `SubscriptionOptions` 并设置 `qos_overriding_options = QosOverridingOptions::with_default_policies()`。

### 3.2.3 深坑提醒

- **RELIABLE 发布 + BEST_EFFORT 订阅**：能建立连接（订阅端放宽）；
- **BEST_EFFORT 发布 + RELIABLE 订阅**：**无法建立连接**，且默认不会报错——只是收不到消息；
- **TRANSIENT_LOCAL 订阅端**：订阅后会立刻收到历史消息，可能导致控制环启动瞬间被灌入过期命令；
- **history.depth=0 不被允许**——最小 1；超过 depth 时 RELIABLE 的发布端会**阻塞在 publish()**，控制环死循环。

---

## 3.3 Executor 选型（不 fork）

官方 rclcpp 提供三种 Executor，通过 main 函数切换：

| Executor | 适用 | 延迟 | 并发度 | 注意 |
|---------|------|------|--------|------|
| `SingleThreadedExecutor` | 简单节点 | 高 | 串行 | 默认 |
| `MultiThreadedExecutor` | 多回调节点 | 中 | 并发（有 wait_mutex） | 注意死锁 |
| `StaticSingleThreadedExecutor` | 回调集合不变 | 低 | 串行 | 禁用动态增删 |
| `EventsExecutor`（experimental） | 延迟敏感 | **最低** | 事件驱动 | 回调必须线程安全 |

**一次切换即 30–50% 延迟收益**（对于 entity 数 > 20 的节点）：
```cpp
// 替换 rclcpp::spin(node);
rclcpp::experimental::executors::EventsExecutor exec;
exec.add_node(node);
exec.spin();
```

---

## 3.4 IntraProcess Communication（IPC）

**是否启用**：
```cpp
rclcpp::NodeOptions options;
options.use_intra_process_comms(true);
auto node = std::make_shared<MyNode>("my_node", options);
```

**效果**：同进程 Publisher → Subscription 走快速路径，`unique_ptr` 直接移交，**0 序列化 + 0 DDS 开销**。

**前置条件**（缺一不可）：
1. 发布端与订阅端在**同一进程**（用 composable node container）；
2. `publish(std::unique_ptr<MsgT>)` —— 必须是 unique_ptr（shared_ptr 降级为拷贝）；
3. QoS `history=KEEP_LAST`、`durability=VOLATILE`、reliability 一致；
4. Subscription 回调形参是 `std::unique_ptr<MsgT>` 或 `const MsgT&`（shared_ptr 会强制 DDS 路径）。

**一项典型应用**：把传感器 + 感知 + 规划 + 控制**全部放进一个 component container**：
```bash
ros2 run rclcpp_components component_container_mt &
ros2 component load /ComponentManager my_pkg MySensor
ros2 component load /ComponentManager my_pkg MyPerception
ros2 component load /ComponentManager my_pkg MyController
```
**收益**：整个管线 P99 端到端从 ~2 ms 降到 ~80 μs，**无代码改动**。

---

## 3.5 编译优化

### 3.5.1 编译选项

在 colcon workspace 中：
```bash
colcon build \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS_RELEASE="-O3 -march=native -DNDEBUG" \
    -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=ON
```

**LTO 收益**：rclcpp 大量模板 + 虚函数调用，LTO 后编译器能内联掉虚函数派发，对调度器热路径提升 10–15%。

**`-march=native` 代价**：二进制绑定到构建机器的 CPU。若要部署到多种 CPU，用 `-march=x86-64-v3`（AVX2 基线，2013 年后的 CPU 都支持）或 `-mcpu=neoverse-n1` 等指定。

### 3.5.2 链接器

装 mold：
```bash
sudo apt install mold
export LDFLAGS="-fuse-ld=mold"
```
**不影响运行时**，但 colcon build 时间降 30–60%——这是长期维护成本的一部分。

### 3.5.3 PGO（Profile-Guided Optimization）

两轮构建：
1. 第一轮 `-fprofile-generate=profdir` 构建 instrumented 版本；
2. 跑一遍**代表性工作负载**（真实机器人 30 分钟）；
3. 第二轮 `-fprofile-use=profdir` 用采集的数据重新构建。

**收益**：对 rclcpp Executor 循环、rmw dispatch、序列化热路径可再提 5–10%。
**代价**：需要 CI 跑代表性负载，基础设施投入大，适合稳定后再做。

---

## 3.6 进程与线程调优

### 3.6.1 进程优先级（不需要 root，但需要 `rtprio` ulimit）

```bash
# /etc/security/limits.d/99-realtime.conf
@realtime    -   rtprio     98
@realtime    -   memlock    unlimited
```

启动时：
```bash
chrt -f 90 ros2 run my_pkg my_node
# SCHED_FIFO priority 90
```

或在代码里：
```cpp
struct sched_param param{};
param.sched_priority = 90;
pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
```

### 3.6.2 内存锁定

防止页换出：
```cpp
#include <sys/mman.h>
mlockall(MCL_CURRENT | MCL_FUTURE);
```
**前置**：`memlock unlimited` ulimit + 预留足够物理内存。

### 3.6.3 CPU 亲和（阶段 1 可做的温和版本）

```cpp
cpu_set_t cpuset;
CPU_ZERO(&cpuset);
CPU_SET(4, &cpuset);   // 把控制线程钉到核 4
pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
```

**真正的 CPU 隔离（isolcpus / cgroup / nohz_full）** 是阶段 6 的内容，需要改内核启动参数。

---

## 3.7 内核调优（不换内核的温和版本）

`/etc/sysctl.d/99-ros2.conf`：
```ini
# 网络栈
net.core.rmem_max = 67108864
net.core.wmem_max = 67108864
net.core.rmem_default = 67108864
net.core.wmem_default = 67108864
net.core.netdev_max_backlog = 10000
net.ipv4.udp_mem = 65536 131072 262144

# 共享内存（给 SHM 传输）
kernel.shmmax = 17179869184
kernel.shmall = 4194304
```

**UDP 缓冲增大**是 DDS 配置下延迟抖动最常见的瓶颈——默认 `rmem_max=212 KB` 会导致突发消息丢失，触发 RELIABLE 重传，P99 暴涨。

---

## 3.8 Discovery Server 模式（FastDDS）

**一行配置解决 40 节点发现风暴**：

1. 启动 Discovery Server：
   ```bash
   fastdds discovery -i 0
   ```
2. 所有节点连到它：
   ```bash
   export ROS_DISCOVERY_SERVER=127.0.0.1:11811
   ros2 run ...
   ```

**收益**：40 节点冷启动 30 s → 5 s；稳态 meta-traffic 降 80%。
**代价**：Discovery Server 是单点，生产部署需做 HA（见 [09 维护策略](./09_maintenance_strategy.md)）。

---

## 3.9 本阶段结束准则

在做完上述所有配置后，**重新跑 [01 章 §3 基线](./01_motivation_and_baseline.md#3-基线测量方法)**。典型收益：

| 场景 | Before | After | 收益 |
|------|--------|-------|------|
| A-DDS（1 KB 同机） | 350 μs | 180 μs | **1.9×** |
| A-IPC（启用后） | — | 65 μs | **5.4×** |
| B（1 MB 大消息） | 10 ms | 6 ms | 1.7× |
| C（500 Hz 控制） | σ=200 μs | σ=120 μs | 1.7× |
| D（40 节点发现） | 45 s | 6 s | **7.5×** |

**如果你的业务指标在这一阶段已经达成 → 停止，写好配置文档，进入稳定运维期。**

**如果还有差距 → 进入阶段 2**（[04 阶段 2：rclcpp 改造](./04_phase2_rclcpp_executor.md)）。

---

## 3.10 本阶段的维护成本

- 改动面：**仅配置文件 + 启动脚本 + sysctl**；
- 与上游兼容性：**100%**，升级 ROS 2 版本只需验证配置；
- 需要新技能：DDS QoS / FastDDS XML / Linux 调度基础；
- 每次 ROS 2 版本升级的验证工作量：**1–2 天**（主要是 QoS 默认值变化）；
- 典型团队规模：**0.5 人**兼职维护即可。

这是**所有优化阶段中 ROI 最高的一阶段**。除非有极端需求，否则应该无条件做完。
