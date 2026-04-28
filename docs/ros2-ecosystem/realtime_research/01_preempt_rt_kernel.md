# PREEMPT_RT 内核深度指南

> 本文档覆盖 PREEMPT_RT 补丁的核心原理、在 Ubuntu 22.04/24.04 和 Jetson Orin 上的安装方法、内核配置参数、BIOS 调优以及延迟测试工具链。

---

## 一、PREEMPT_RT 补丁原理

### 1.1 普通 Linux 内核的实时性瓶颈

标准 Linux 内核并非为硬实时设计，存在以下不可预测延迟来源：

1. **Spinlock 临界区不可抢占**：内核代码在 spin_lock 保护下运行时不能被高优先级任务抢占
2. **Softirq / tasklet 高优先级执行**：网络、块设备等软中断在内核上下文中以最高优先级运行，无法被用户任务抢占
3. **中断处理不可抢占**：硬件中断处理程序（ISR）在内核上下文执行，持续时间无界
4. **低精度定时器（jiffies）**：默认 250 Hz（4ms 精度），无法满足 1 kHz 控制需求

### 1.2 PREEMPT_RT 的四大核心变换

#### 1.2.1 Spinlock → RT-Mutex（优先级继承互斥锁）

```
普通内核：
  spin_lock() → 关闭抢占 → 临界区 → spin_unlock() → 开启抢占
  （持有期间高优先级任务无法运行）

PREEMPT_RT：
  rt_mutex_lock() → 可睡眠的互斥量 → 临界区（可被抢占）→ rt_mutex_unlock()
  （优先级继承：低优先级持有者临时提升到等待者最高优先级）
```

大多数内核 spinlock 被替换为 `rtmutex`（`CONFIG_PREEMPT_RT` 自动完成），仅剩少量真正需要关闭抢占的路径保留原始 spinlock（如硬件寄存器访问）。

#### 1.2.2 Softirq 线程化

```
普通内核：
  硬中断 ISR → 触发 SOFTIRQ → 在 irq_exit() 中直接执行（不可抢占）

PREEMPT_RT：
  硬中断 ISR（最小化，只 wake up 线程） → ksoftirqd/N 内核线程执行
  ksoftirqd 线程可被 SCHED_FIFO 任务抢占
```

这意味着 `ksoftirqd` 可以被配置为低优先级，防止网络收包等 softirq 干扰 RT 控制线程。

#### 1.2.3 中断线程化（Threaded IRQ）

```
PREEMPT_RT：
  硬件中断 → 极短 ISR（只 ack 中断控制器）→ irqthread/irq_N 线程处理
  irqthread 线程默认优先级 SCHED_FIFO prio 50（可调整）
```

关键设备驱动（EtherCAT 网卡）的中断线程可以设置较高优先级以确保低延迟响应。

#### 1.2.4 高精度定时器（hrtimer）

PREEMPT_RT 依赖 `CONFIG_HIGH_RES_TIMERS`，通过硬件 one-shot 定时器（HPET/TSC/Local APIC Timer）提供纳秒级精度定时。

```c
// 内核中 hrtimer 触发路径（PREEMPT_RT 下）
hrtimer_interrupt()
  → __hrtimer_run_queues()
  → hrtimer_callback()  // 例如：controller_manager 的 periodic update
  → wake_up_process(rt_control_thread)
```

### 1.3 CONFIG_PREEMPT 选项对比

| 配置选项 | 抢占模式 | 典型最坏延迟 | 适用场景 |
|---------|---------|------------|--------|
| `CONFIG_PREEMPT_NONE` | 无抢占（服务器模式） | 10–100 ms | 高吞吐服务器 |
| `CONFIG_PREEMPT_VOLUNTARY` | 自愿抢占 | 1–10 ms | 桌面系统默认 |
| `CONFIG_PREEMPT` | 可抢占内核 | 100 μs – 1 ms | 一般嵌入式 |
| **`CONFIG_PREEMPT_RT`** | **完全可抢占（RT patch）** | **< 50 μs（x86工控机）** | **硬实时控制** |

---

## 二、Ubuntu 22.04 / 24.04 安装路径

### 2.1 方法一：Ubuntu Pro linux-realtime（推荐生产环境）

Ubuntu Pro 提供官方维护的 PREEMPT_RT 内核，无需手动编译：

```bash
# 1. 注册 Ubuntu Pro（免费个人/小团队账户）
sudo pro attach <token>

# 2. 启用 realtime-kernel addon
sudo pro enable realtime-kernel

# 3. 重启并验证
sudo reboot

# 验证内核版本和 RT 标识
uname -r
# 输出示例：5.15.0-1057-realtime

# 验证 PREEMPT_RT 编译选项
zcat /proc/config.gz | grep CONFIG_PREEMPT_RT
# CONFIG_PREEMPT_RT=y
```

> **注意**：Ubuntu Pro 的 `linux-realtime` 目前支持 x86_64（22.04/24.04），ARM64 支持有限。Jetson Orin 见 §2.3。

### 2.2 方法二：手动打补丁编译（适用于定制化需求）

```bash
# ── 步骤 1：安装编译依赖 ─────────────────────────────────────────────────
sudo apt install -y build-essential bc curl ca-certificates fakeroot \
    libncurses5-dev libssl-dev libelf-dev bison flex dwarves \
    debhelper rsync lz4 zstd

# ── 步骤 2：下载内核源码和 RT 补丁 ─────────────────────────────────────────
KERNEL_VER=6.6.30
RT_PATCH_VER=6.6.30-rt30

# 下载内核
wget https://cdn.kernel.org/pub/linux/kernel/v6.x/linux-${KERNEL_VER}.tar.xz
wget https://cdn.kernel.org/pub/linux/kernel/projects/rt/6.6/patch-${RT_PATCH_VER}.patch.xz

# 解压并打补丁
tar xf linux-${KERNEL_VER}.tar.xz
cd linux-${KERNEL_VER}
xzcat ../patch-${RT_PATCH_VER}.patch.xz | patch -p1

# ── 步骤 3：配置内核 ─────────────────────────────────────────────────────
# 从当前运行内核复制配置作为基础
cp /boot/config-$(uname -r) .config
make olddefconfig

# 通过菜单配置关键选项
make menuconfig
# 关键路径：
#   General Setup → Preemption Model → "Fully Preemptible Kernel (Real-Time)"
#   Processor type and features → Timer frequency → 1000 Hz
#   Processor type and features → No idle Hz (Full)
#   Processor type and features → CPU isolation

# ── 步骤 4：关键配置项（可用脚本直接设置）──────────────────────────────────
scripts/config --set-val CONFIG_PREEMPT_RT y
scripts/config --set-val CONFIG_HZ_1000 y
scripts/config --set-val CONFIG_HZ 1000
scripts/config --set-val CONFIG_NO_HZ_FULL y
scripts/config --set-val CONFIG_CPU_ISOLATION y
scripts/config --set-val CONFIG_HIGH_RES_TIMERS y
scripts/config --set-val CONFIG_RCU_NOCB_CPU y
scripts/config --set-val CONFIG_HZ_PERIODIC n
scripts/config --set-val CONFIG_NO_HZ_IDLE n
# 关闭 debug 选项（会显著增加延迟）
scripts/config --set-val CONFIG_DEBUG_PREEMPT n
scripts/config --set-val CONFIG_LOCKDEP n
scripts/config --set-val CONFIG_KASAN n

# ── 步骤 5：编译（多核加速）────────────────────────────────────────────────
make -j$(nproc) bindeb-pkg LOCALVERSION=-rt

# ── 步骤 6：安装 ────────────────────────────────────────────────────────
sudo dpkg -i ../linux-image-*.deb ../linux-headers-*.deb

# ── 步骤 7：配置 GRUB ───────────────────────────────────────────────────
# /etc/default/grub 中设置默认启动新内核
sudo update-grub
sudo reboot
```

### 2.3 关键内核配置项说明

| 配置项 | 作用 | 推荐值 |
|--------|------|--------|
| `CONFIG_PREEMPT_RT` | 启用完全可抢占实时内核 | `y` |
| `CONFIG_HZ_1000` | 定时器中断频率 1000 Hz（1ms 精度） | `y` |
| `CONFIG_NO_HZ_FULL` | 指定 CPU 在运行单任务时禁用定时器中断（tick isolation） | `y` |
| `CONFIG_CPU_ISOLATION` | 启用 `isolcpus` 内核参数支持 | `y` |
| `CONFIG_HIGH_RES_TIMERS` | 高精度 hrtimer 支持 | `y` |
| `CONFIG_RCU_NOCB_CPU` | 将 RCU 回调卸载到专用线程，减少 RT 核的 RCU 延迟 | `y` |
| `CONFIG_IRQ_FORCED_THREADING` | 强制所有中断线程化 | `y`（PREEMPT_RT 自动） |
| `CONFIG_HAVE_DYNAMIC_FTRACE` | 允许动态 ftrace 插桩（用于延迟分析） | `y` |

---

## 三、BIOS/UEFI 配置

**这是最常被忽视但影响最大的优化步骤。** 以下每一项都可能产生数十到数百微秒的不可预测延迟：

### 3.1 必须关闭的功能

| BIOS 选项 | 通常路径 | 关闭后效果 | 备注 |
|-----------|---------|----------|------|
| **C-States** | Power → CPU C State | 消除 CPU 唤醒延迟（最高 C6 唤醒可达 200μs） | 关闭后功耗增加 |
| **Intel Turbo Boost / AMD Precision Boost** | CPU → Turbo | 避免频率切换引起的抖动 | 固定频率更确定性 |
| **Hyper-Threading (HT)** | CPU → SMT/HT | 避免 SMT 线程间干扰 RT 核 | 减少一半逻辑核 |
| **SpeedStep / EIST** | Power → Intel SpeedStep | 固定 CPU 频率 | 与 Turbo 关闭配合 |
| **SMM（系统管理中断）** | 部分工控机支持关闭 | 消除 SMI 导致的 100μs+ 延迟 | 工业级主板功能 |
| **ACPI S1/S3 睡眠** | Power Management | 防止电源管理中断 | RT 系统通常禁用 |

### 3.2 内核命令行参数（/etc/default/grub）

```bash
# 完整 RT 优化的 GRUB_CMDLINE_LINUX 示例
GRUB_CMDLINE_LINUX="isolcpus=2,3 nohz_full=2,3 rcu_nocbs=2,3 \
    irqaffinity=0,1 \
    processor.max_cstate=0 idle=poll \
    intel_pstate=disable cpufreq.default_governor=performance \
    transparent_hugepage=never \
    skew_tick=1 \
    numa_balancing=disable \
    quiet splash"
```

| 参数 | 说明 |
|------|------|
| `isolcpus=2,3` | 将 Core 2,3 从 Linux 调度器隔离，只运行被明确 affinity 的任务 |
| `nohz_full=2,3` | 对 Core 2,3 启用 tick isolation（单任务运行时无 HZ 中断） |
| `rcu_nocbs=2,3` | 将 Core 2,3 的 RCU 回调迁移到其他核 |
| `irqaffinity=0,1` | 将所有 IRQ 默认绑定到 Core 0,1（RT 核不处理 IRQ） |
| `processor.max_cstate=0` | 禁止 CPU C-State 深度睡眠 |
| `idle=poll` | 空闲时 busy-wait 而非睡眠（极低延迟，高功耗） |
| `intel_pstate=disable` | 禁用 P-State 驱动，使用 cpufreq |
| `transparent_hugepage=never` | 关闭透明大页（可能触发 page fault 导致抖动） |
| `skew_tick=1` | 错开多核 tick 中断，减少 Core 间干扰 |

---

## 四、延迟测试工具

### 4.1 cyclictest — 调度延迟测试

`cyclictest` 是 PREEMPT_RT 开发者提供的标准延迟测试工具：

```bash
# 安装
sudo apt install rt-tests

# 基础测试：单线程，优先级 80，运行 60 秒，输出直方图
sudo cyclictest \
    --mlockall \
    --smp \
    --priority=80 \
    --interval=1000 \
    --distance=0 \
    --duration=60 \
    --histogram=100 \
    --histofall \
    --quiet \
    2>&1 | tee cyclictest_results.txt

# 生产级测试（建议运行 60 分钟以上，覆盖 SMI 等低概率事件）
sudo cyclictest \
    -t1 \
    -p95 \
    -n \
    -i 200 \
    -l 1000000 \
    --affinity=2 \
    -D 0 \
    -q \
    --histfile cyclictest_hist.txt
```

**关键参数说明：**

| 参数 | 说明 |
|------|------|
| `--mlockall` | 测试进程锁定内存，防止 page fault |
| `-p <prio>` | SCHED_FIFO 优先级（80–95 适合测试） |
| `-i <μs>` | 唤醒间隔（单位 μs，1000 = 1kHz） |
| `-n` | 使用 clock_nanosleep 而非 nanosleep |
| `--affinity=N` | 绑定到指定 CPU 核 |
| `-D <secs>` | 运行时长（0=无限） |
| `--histogram=N` | 输出延迟直方图（N=最大 bin 值，单位 μs） |

**典型输出解读：**

```
# /dev/cpu_dma_latency set to 0us
RTTSSRT Test 1.00
Thread 0 Interval: 1000 us

         Min Latencies: 00007
         Avg Latencies: 00010
         Max Latencies: 00047   ← 关键指标：最大延迟（μs）
```

**RT 系统验收标准：**

| 系统类型 | 最大延迟目标 | 说明 |
|---------|------------|------|
| x86 工控机（PREEMPT_RT + BIOS 优化） | < 50 μs | cyclictest 60 分钟测试 |
| x86 工控机（关闭 SMI 支持） | < 20 μs | 工业级工控机 |
| Jetson Orin（RT 内核） | < 100 μs | ARM SBC，受 GPU 时钟影响 |
| 普通 Linux 内核（无 RT 补丁） | 100–5000 μs | 不适合 1kHz 以上控制 |

### 4.2 hwlatdetect — 硬件延迟检测（SMI）

SMI（System Management Interrupt）是一种绕过 OS 的硬件中断，用于固件（BIOS/UEFI）执行电源管理等任务，可造成 100μs+ 的不可预测停顿：

```bash
# hwlatdetect 检测 SMI 风暴
sudo hwlatdetect --duration=30 --threshold=10

# 参数说明：
# --duration=30    运行 30 秒
# --threshold=10   报告超过 10μs 的硬件延迟

# 输出示例（存在 SMI 问题）：
# hwlatdetect:  21 samples below threshold
# hwlatdetect:  3 samples above threshold
# ...
# Maximum latency: 152us
# Samples recorded: 24
# Samples exceeding threshold: 3
```

如果 `hwlatdetect` 检测到频繁的大延迟，且 `cyclictest` 在 PREEMPT_RT 下仍有高尖峰，说明问题源于 SMI，需要在 BIOS 中关闭相关功能或联系主板厂商。

### 4.3 比较数据汇总

以下为在不同配置下运行 `cyclictest -t1 -p95 -i1000 -D 60` 的典型结果：

| 环境 | 最小延迟 | 平均延迟 | **最大延迟** |
|------|---------|---------|------------|
| 普通 Ubuntu 22.04（PREEMPT=VOLUNTARY） | 3 μs | 8 μs | **2,340 μs** |
| Ubuntu 22.04 + PREEMPT（CONFIG_PREEMPT） | 3 μs | 7 μs | **450 μs** |
| Ubuntu 22.04 + PREEMPT_RT（无 BIOS 优化） | 4 μs | 12 μs | **180 μs** |
| Ubuntu 22.04 + PREEMPT_RT + isolcpus + C-State 关闭 | 4 μs | 9 μs | **48 μs** |
| Ubuntu 22.04 + PREEMPT_RT + 完整 BIOS + SMI 关闭 | 3 μs | 8 μs | **15 μs** |
| **Jetson Orin（NVIDIA RT 内核 + L4T 36.x）** | 5 μs | 14 μs | **95 μs** |

---

## 五、Jetson Orin 实时内核

### 5.1 NVIDIA 提供的 RT 内核

NVIDIA 为 Jetson Orin 系列提供基于 PREEMPT_RT 的官方内核，通过 JetPack SDK 分发：

```bash
# 查看当前 L4T 版本
cat /etc/nv_tegra_release
# 示例：R36 REVISION: 3.0

# L4T 36.x（JetPack 6.x）对应内核 5.15 + PREEMPT_RT
# L4T 35.x（JetPack 5.x）对应内核 5.10 + PREEMPT_RT

# 检查当前内核是否为 RT
uname -r
# 示例：5.15.136-tegra（标准版）或 5.15.136-rt-tegra（RT 版）

# 安装 RT 内核（JetPack 5.x 时代方法）
sudo apt install nvidia-l4t-rt-kernel nvidia-l4t-rt-kernel-headers

# JetPack 6.x（L4T 36.x）：通过 SDK Manager 选择 RT 内核版本
# 或手动从 NVIDIA 开发者下载 BSP RT 包
```

### 5.2 Jetson Orin RT 配置特殊性

```bash
# Jetson Orin 特有：CUDA 和 RT 线程需要分离 CPU/GPU 亲和性
# GPU 工作负载（VLA 推理）使用 Core 4-7
# RT 控制线程使用 Core 0-3（Arm Cortex-A78AE 大核）

# 设置 nvpmodel 为最高性能模式（避免功耗管理干扰 RT）
sudo nvpmodel -m 0   # MODE_MAXN：所有核心满频

# 固定 CPU 频率（禁止动态调频）
sudo jetson_clocks

# 验证 tegrastats
tegrastats --interval 1000
# 观察 CPU 频率是否稳定在 2.2 GHz（Orin NX/AGX 大核）
```

### 5.3 L4T 版本与 RT 内核对应关系

| JetPack 版本 | L4T 版本 | 内核版本 | RT 补丁版本 | 备注 |
|------------|---------|---------|------------|------|
| JetPack 6.1 | L4T 36.4 | 5.15.x | PREEMPT_RT 5.15.x-rt | 官方支持 |
| JetPack 6.0 | L4T 36.3 | 5.15.x | PREEMPT_RT 5.15.x-rt | 官方支持 |
| JetPack 5.1.3 | L4T 35.5 | 5.10.x | PREEMPT_RT 5.10.x-rt | LTS 版本 |
| JetPack 5.0 | L4T 35.1 | 5.10.x | PREEMPT_RT 5.10.x-rt | — |

> **Jetson Orin 的 RT 限制**：Orin 的 GPU（Ampere 架构）驱动在 RT 内核下需要特别配置，CUDA 调用不应从 RT 线程中发起（可能造成不确定性延迟）。建议将 VLA GPU 推理放在独立非 RT 进程。

---

## 六、RT 内核验证检查清单

```bash
#!/bin/bash
# rt_kernel_verify.sh — RT 内核配置验证脚本

echo "=== PREEMPT_RT 内核验证 ==="

# 1. 内核版本
echo "[1] 内核版本：$(uname -r)"
echo "    RT 标识：$(uname -v | grep -oP 'PREEMPT_RT' || echo '未找到 PREEMPT_RT')"

# 2. 关键内核选项
echo "[2] 内核配置："
for opt in CONFIG_PREEMPT_RT CONFIG_HZ_1000 CONFIG_NO_HZ_FULL CONFIG_CPU_ISOLATION; do
    val=$(zcat /proc/config.gz 2>/dev/null | grep "^${opt}=" | cut -d= -f2)
    printf "    %-30s = %s\n" "$opt" "${val:-未找到}"
done

# 3. CPU 隔离
echo "[3] isolcpus：$(cat /sys/devices/system/cpu/isolated 2>/dev/null || echo '未配置')"
echo "    nohz_full：$(cat /sys/devices/system/cpu/nohz_full 2>/dev/null || echo '未配置')"

# 4. CPU 频率调节器
echo "[4] CPU 频率调节器："
for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    [ -f "$cpu" ] && echo "    $(dirname $cpu | xargs basename): $(cat $cpu)"
done | sort -u

# 5. irqbalance 状态（应禁用）
echo "[5] irqbalance：$(systemctl is-active irqbalance 2>/dev/null)"
echo "    建议：sudo systemctl disable --now irqbalance"

# 6. 内存大页
echo "[6] 透明大页：$(cat /sys/kernel/mm/transparent_hugepage/enabled)"
echo "    建议：echo never > /sys/kernel/mm/transparent_hugepage/enabled"

echo "=== 验证完成 ==="
```
