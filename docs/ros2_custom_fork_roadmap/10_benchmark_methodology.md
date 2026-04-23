# 10 基准方法论 —— 如何正确测量、避免自我欺骗

> **核心主张**：**测不准 = 没有优化**。没有可重复、可量化的基准，任何"我改了后感觉变快了"的陈述都不可采信。

---

## 10.1 为什么 ROS 2 性能测量特别难

1. **多层抽象**：rclcpp → rcl → rmw → DDS → kernel → hardware。任何一层有变动都影响结果；
2. **多进程/多线程**：时钟同步（`clock_gettime` 本身有 20–50 ns 开销）影响测量；
3. **预热效应**：JIT 缓存 / 分支预测 / L1-L3 填充，前几秒数据不可用；
4. **异常值分布**：延迟不是正态分布，是**重尾**分布——平均值无意义，必须看分位数；
5. **背景噪音**：cron job / 后台服务 / kswapd 偶发抢占；
6. **抖动源多**：Executor 抖动、DDS 抖动、内核抖动、硬件抖动（SMI）。

---

## 10.2 测量工具

### 10.2.1 performance_test（首选）

来源：Apex.AI 开源，ROS 官方收录。

**能力**：
- 配置发布/订阅对、QoS、消息大小、频率；
- 测量端到端延迟（收到消息时的 `now - msg.header.stamp`）；
- 输出 CSV：min / max / mean / 各分位数 / 吞吐；
- 支持多种 RMW 实现对比。

**用法**：
```bash
ros2 run performance_test perf_test \
    -c ROS2 \
    -t Array1k \
    --rate 100 \
    --num_pub_threads 1 \
    --num_sub_threads 1 \
    --max_runtime 60 \
    -l /tmp/perf.csv
```

**注意**：
- 发布端和订阅端时钟**必须同一进程**——否则跨进程 clock skew 被算进延迟；
- 若测跨进程，用 "inter-process" 模式 + 精确 wall clock 同步（CLOCK_REALTIME with PTP）。

### 10.2.2 iRobot Reference System

来源：https://github.com/ros-realtime/reference-system

**能力**：
- 27 个节点模拟真实 ADAS / 机器人管线；
- 包含传感器输入、融合、规划、控制；
- 输出：每个节点的调度延迟、端到端延迟、缺帧率；
- 跨实现对比（不同 RMW / 不同 Executor）。

**适用**：**系统级回归**、架构变更验证。

### 10.2.3 ros2_tracing + LTTng

**能力**：
- 在 rclcpp / rcl / rmw 的关键点插桩（不修改代码，通过 tracepoint）；
- 记录事件序列：subscription_receive → executor_ready → callback_start → callback_end → publish；
- 离线分析：每阶段耗时分解。

**用法**：
```bash
ros2 trace -s my_session -e 'ros2:*'
# 跑负载
ros2 trace -s my_session stop
# 分析
babeltrace2 ~/.ros/tracing/my_session > trace.txt
python3 tracetools_analysis/callback_duration.py ~/.ros/tracing/my_session
```

**价值**：能**定位瓶颈到具体调用**——不只是"慢"，而是"慢在哪一步"。

### 10.2.4 perf / eBPF

**能力**：
- CPU 采样：找热点函数；
- uprobe / kprobe：函数级时延直方图（不修改代码）；
- 内存分配 tracing（jemalloc --prof、heaptrack、bpftrace 的 `uprobe:*:malloc`）；
- 锁争用：`perf lock record`、`bpftrace` 的 `lockcontention`。

**典型命令**：
```bash
# CPU 火焰图
perf record -F 99 -p <pid> -g -- sleep 30
perf script | stackcollapse-perf.pl | flamegraph.pl > fg.svg

# 单函数延迟直方图
bpftrace -e 'uretprobe:/path/to/librclcpp.so:*execute_any_executable* {
  @ = lhist(nsecs - @start[tid], 0, 100000, 1000);
}'
```

### 10.2.5 cyclictest（内核延迟基线）

测**纯内核**的调度抖动，不涉及 ROS 2。**在做任何 ROS 2 延迟分析前先跑 cyclictest**——如果内核本身抖动 1 ms，再怎么优化 ROS 2 也无济于事。

```bash
sudo cyclictest -l 100000000 -m -Sp 90 -i 200 -h 1000 -q
```

---

## 10.3 实验设计原则

### 10.3.1 单变量原则

每次实验**只改一个变量**。别同时切 RMW + 改 QoS + 改 Executor——否则归因不可能。

### 10.3.2 预热 + 多次重复

- 预热：前 5–10 s 数据丢弃（JIT、缓存、DDS 建连）；
- 持续时间：至少 60 s 或 10⁶ 样本；
- 重复：至少 3 次实验，报告中位数或**对所有样本合并**后取分位数。

### 10.3.3 控制背景

```bash
# 关闭可能干扰的服务
sudo systemctl stop cron
sudo systemctl stop snapd
sudo systemctl stop packagekit

# 固定频率
sudo cpupower frequency-set -g performance

# 让机器"静"下来 30 秒再开始
sleep 30
```

### 10.3.4 专用 Bench 机器

- 硬件固定、BIOS 固定、内核固定；
- 不上网（或用单独 VLAN）；
- 每次实验前 reboot（消除长期运行状态）；
- **不要在开发机上跑生产基准**。

---

## 10.4 度量指标

### 10.4.1 延迟分位数（不用 mean）

| 指标 | 含义 | 用途 |
|------|------|------|
| P50（中位数） | 典型性能 | 日常感受 |
| P95 | 95% 场景 | 容量规划 |
| P99 | 1% 坏情况 | SLO 定义 |
| P99.9 / P99.99 | 极端尾部 | 安全关键系统 |
| Max | 最差 | 硬实时上界 |
| σ（标准差） | 抖动 | 稳定性 |

**具身智能控制环**必须看 P99.99 和 Max——一次超时就可能让机器人失稳。

### 10.4.2 吞吐指标

- 消息数/秒 @ 给定延迟 SLO；
- 字节数/秒；
- CPU 占用 per msg；
- 内存带宽占用（`perf stat -e cache-misses` 或 Intel VTune）。

### 10.4.3 资源指标

- CPU 使用率（per-core）；
- 内存（RSS / VSS / anonymous / file）；
- 上下文切换次数（`/proc/<pid>/status`）；
- Page fault 次数；
- 系统调用率（strace -c 或 perf trace）。

### 10.4.4 质量指标

- 丢帧率（BEST_EFFORT 下）；
- 乱序率；
- 重传率（RELIABLE 下）；
- 连接建立时间。

---

## 10.5 性能看板

### 10.5.1 组织方式

```
主看板
├── 延迟分布（按 topic / 按场景）
│    ├── P50 / P99 / P99.99 / Max
│    └── 时间序列 + 分布直方图
├── 资源消耗
│    ├── CPU per node
│    └── RSS over time
├── 吞吐
│    └── msg/s, MB/s
└── 质量
     ├── 丢包率
     └── 发现耗时
```

推荐实现：Prometheus + Grafana + ros2_tracing exporter。

### 10.5.2 趋势 vs 快照

- **快照**：当前 build 的一次测量；
- **趋势**：时间序列，每次 commit 都跑测；
- 只看快照会错过**缓慢退化**——一次退化 1%，半年后累计 30%。

### 10.5.3 告警规则

- P99 较 7 天均值 > 20%：yellow；
- P99.99 较 baseline > 50%：red；
- Max 突破 SLO：page on-call。

---

## 10.6 常见测量错误

1. **发布端和订阅端在不同机器，未做时钟同步** → 延迟包含 clock skew（可能为负）；
2. **`std::chrono::system_clock` 不单调** → 跨 NTP 校正时出现负延迟；正确做法 `steady_clock`；
3. **时间戳打在应用层，但中间件耗时被遗漏** —— 要区分 "app-to-app" 和 "rclcpp-publish-to-rclcpp-callback"；
4. **日志本身影响被测系统** → 关闭 rosout，或把日志写 tmpfs；
5. **预热不足** → 前 30 s 的 ramp-up 被算进结果；
6. **异常值被剔除** → 报告"去除 10% outlier 后的 P99"——**这不叫 P99**；
7. **报告了平均值** → 重尾分布下平均值可能比 P99 还小，误导性；
8. **没有对照组** → "我们的优化让延迟降了 20%"，但对比的是一个月前的旧基线，机器可能也换了；
9. **基准写得不真实** → 用一个空回调测延迟，真实负载下完全不同；
10. **没用多种 payload** → 只测 1 KB 的结果不代表 1 MB。

---

## 10.7 报告模板

一次优化报告应包含：

```markdown
## 优化：<标题>

### 目标
<要解决的问题，引用 [01 基线] 的某条指标>

### 实施
<patch 描述，diff 行数>

### 测量环境
- HW: Intel i7-12700K, 64 GB DDR5, Ubuntu 22.04
- Kernel: 6.5.0-28-generic PREEMPT_RT
- ROS: Humble Hawksbill
- RMW: rmw_cyclonedds_cpp 1.6.0
- Bench: performance_test, 10^6 samples, 3 runs

### 结果
| 指标 | Before | After | Δ |
|------|--------|-------|---|
| P50 | 25 μs | 18 μs | -28% |
| P99 | 120 μs | 45 μs | -62% |
| P99.99 | 800 μs | 200 μs | -75% |
| CPU | 15% | 12% | -20% |

### 副作用
<内存增加？吞吐下降？兼容性？>

### 回滚方案
<如何反转>
```

---

## 10.8 本章小结

- **性能测量是一门实验科学**：需要单变量、预热、多次重复、专用机器；
- 用 performance_test 做标准基准，ros2_tracing 做内部剖析；
- 报告分位数而非平均值；
- 建立 CI 性能看板，追踪长期趋势；
- **不测就不能说优化了**——这是从工程到科学的分界线。

下一章：[11 决策矩阵](./11_decision_matrix.md)。
