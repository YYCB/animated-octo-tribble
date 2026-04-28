# 大规模采集性能优化

> **参考来源**：`ros2/rosbag2@humble`：`rosbag2_transport/src/rosbag2_transport/recorder.cpp`；  
> `ros2/rclcpp@humble`：`rclcpp/loaned_message.hpp`；  
> ROS 2 设计文档：Zero-copy communication via IPC

---

## 一、具身智能采集带宽需求基线

### 1.1 典型场景带宽分析

**场景：6-DoF 机械臂 + 双 RGB-D + IMU（具身智能标准配置）**

| 数据源 | Topic | 频率 | 单帧字节数 | 带宽（MB/s）|
|--------|-------|------|-----------|------------|
| 关节状态（6 DoF）| `/joint_states` | 1000 Hz | 220 B | 0.21 |
| 关节命令（6 DoF）| `/joint_commands` | 1000 Hz | 220 B | 0.21 |
| RGB 图像（720p）| `/camera/color/image_raw` | 30 fps | 1,382,400 B（1.32 MB）| 39.6 |
| 深度图像（720p，uint16）| `/camera/depth/image_raw` | 30 fps | 921,600 B（0.88 MB）| 26.4 |
| 第二路 RGB（720p）| `/camera2/color/image_raw` | 30 fps | 1,382,400 B | 39.6 |
| 第二路深度（720p）| `/camera2/depth/image_raw` | 30 fps | 921,600 B | 26.4 |
| IMU | `/imu/data` | 200 Hz | 100 B | 0.02 |
| 力矩传感器 | `/wrench` | 500 Hz | 100 B | 0.05 |
| TF | `/tf` | 50 Hz | 500 B | 0.025 |
| **原始合计** | | | | **~132.5 MB/s** |
| **lz4 压缩后估算** | | | | **~80-100 MB/s** |
| **使用 compressed image 后**| | | | **~15-25 MB/s** |

### 1.2 磁盘选型建议

| 磁盘类型 | 顺序写入 | 4K 随机写入 | 是否满足需求 |
|---------|---------|-----------|------------|
| HDD（7200rpm）| 100-150 MB/s | 0.5-1 MB/s | ❌ 无法满足双相机 |
| SATA SSD | 300-550 MB/s | 20-50 MB/s | ⚠️ 双相机原始勉强 |
| NVMe SSD（PCIe 3.0）| 1500-3500 MB/s | 200-500 MB/s | ✅ 所有场景 |
| NVMe SSD（PCIe 4.0）| 4000-7000 MB/s | 500-1000 MB/s | ✅ 甚至有余量 |
| RAM 缓冲 + NVMe | 全内存写入速度 | - | ✅ 极高吞吐 |

**Jetson AGX Orin** 的 NVMe 接口支持 PCIe 4.0 × 4，写入速度可达 5+ GB/s，是具身智能嵌入式平台的首选。

---

## 二、零拷贝路径

### 2.1 rosbag2 中的零拷贝原理

零拷贝（Zero-copy）在 rosbag2 语境中有两层含义：

**Layer 1：IPC 进程内零拷贝（同节点）**
- 录制节点与发布节点在**同一进程**内（Component 模式）
- 通过 IntraProcessManager 传递**原始指针**，无 CDR 序列化
- 但 rosbag2 最终仍需 CDR 序列化写入文件

**Layer 2：Loaned Message（共享内存）**
- `rclcpp::LoanedMessage<T>` 在 RMW 层分配消息内存
- 写操作完成后，消息内存可被 DDS 直接读取（无额外拷贝）
- 回放时：`GenericPublisher::publish_as_loaned_msg()` 尝试零拷贝发布

```
录制的"零拷贝"实际路径：

有效路径（IPC + loaned message）：
发布者节点 → [共享内存/同进程 IntraProcess]
           → GenericSubscription raw callback
           → rcutils_uint8_array_t（CDR bytes）
           → writer_thread_ 异步队列
           → storage_->write(serialized_bytes)
           → 磁盘

注意：写磁盘前始终需要 CDR bytes（rosbag2 的存储格式）
     → 真正的"零拷贝"仅节省了从 DDS 到应用层的那次拷贝
     → 不影响最终的 CDR 序列化和磁盘写入

回放的零拷贝路径（Humble 支持）：
storage_->read_next() → SerializedBagMessage（CDR bytes）
                      → GenericPublisher（loaned message 模式）
                      → RMW 直接引用该内存发布
                      → 订阅者接收时无额外拷贝
```

### 2.2 何时有效，何时失效

| 场景 | 零拷贝有效性 | 原因 |
|------|-----------|------|
| 录制节点作为 Component 同进程 | 部分有效 | IPC 避免网络序列化，但写磁盘仍需 CDR |
| 跨进程录制（独立进程）| 无效 | 经过 DDS 序列化/反序列化 |
| 回放（--disable-loan-message=false）| 有效（RMW 支持时）| LoanedMessage 回放，订阅者零拷贝接收 |
| 回放（--disable-loan-message=true）| 无效 | 强制拷贝（兼容性） |
| 压缩模式（per-message）| 无效 | 压缩引入额外 CPU 拷贝 |

### 2.3 Loaned Message 配置

```bash
# 回放时启用 loaned message（需要 RMW 支持，如 rmw_cyclonedds）
ros2 bag play my_bag
# 默认已启用，除非显式禁用：
ros2 bag play my_bag --disable-loan-message  # 禁用（调试用）
```

---

## 三、内存映射写入

### 3.1 mmap vs 标准 write()

MCAP 存储插件的写入方式：

```
标准 write() 路径：
  application buffer → kernel page cache → 磁盘
  每次 write() syscall 有用户态/内核态切换开销

mmap 路径：
  mmap(file, MAP_SHARED, PROT_WRITE)
  → 直接写内存映射区域（无 syscall）
  → OS 异步将脏页刷入磁盘
  → msync() 显式同步

rosbag2_storage_mcap 的实际做法：
  使用标准 write()（通过 mcap C++ library）
  配合 O_DIRECT 和 block-aligned 写入（可选）
  → 绕过 page cache，直接写入磁盘（减少内存压力）

高吞吐场景推荐：
  ulimit -n 65536           # 提高文件描述符限制
  echo 500000 > /proc/sys/vm/dirty_expire_centisecs  # 增加脏页保留时间
  echo 80 > /proc/sys/vm/dirty_ratio  # 允许更多脏页（内存富裕时）
```

---

## 四、多线程录制架构

### 4.1 RecorderImpl 的线程模型

```
rosbag2_transport::RecorderImpl 的线程模型：

主线程（ROS 2 Executor 线程）：
  └─ GenericSubscription 回调
      └─ receive_raw_message(SerializedBagMessage)
         └─ push to write_cache_（线程安全循环队列）

writer_thread_（独立后台线程）：
  └─ while(running_):
       cache_chunk = write_cache_.pop_batch()  ← 批量取出
       for msg in cache_chunk:
         writer_->write(msg)
       ← 批量写入减少锁争用

write_cache_ 设计：
  类型：std::deque<SerializedBagMessage> + std::mutex
  默认容量：max_cache_size = 100MB（字节限制）
  满时行为：阻塞主线程（back pressure）or 丢弃（配置）
```

### 4.2 异步队列深度配置

```yaml
# 通过 StorageOptions 配置写缓存
storage:
  max_cache_size: 524288000   # 500MB（具身智能高带宽场景推荐）

# 通过环境变量（隐藏参数）：
export ROS_BAG_CACHE_SIZE=524288000
```

**队列深度选取原则**：

```
min_cache_size = max_burst_bandwidth × max_expected_io_stall_time

示例：
  max_burst_bandwidth = 200 MB/s（双相机原始 + 关节）
  max_expected_io_stall_time = 1s（NVMe 偶发慢速）
  → min_cache_size = 200 MB

  NVMe SSD 持续写入速度 = 2000 MB/s >> 200 MB/s
  → 200MB 缓存足够吸收 1s 抖动
  → 推荐设置 500MB-1GB 以防万一
```

### 4.3 compression_threads 配置

```yaml
storage:
  compression_format: "zstd"
  compression_mode: "message"
  compression_threads: 4   # 使用 4 个线程并发压缩

# 注意：compression_threads 越多，压缩越快，但 CPU 争用也越大
# 具身智能场景建议：
#   Jetson AGX Orin（12核 CPU）：compression_threads = 4
#   工控机（8核 CPU）：compression_threads = 2
#   实时控制环节同机器上：compression_threads = 1（不影响控制频率）
```

---

## 五、丢帧检测与报警

### 5.1 丢帧的根本原因

rosbag2 录制可能丢帧的原因：

```
原因链：
  1. 发布者发布速率 > 录制节点处理速率
     → subscription queue 溢出（QoS depth 不足）
     → OS 丢弃消息（BEST_EFFORT）

  2. writer_thread_ 写速度 < 消息到达速率
     → write_cache_ 积压 → 达到 max_cache_size
     → 后续消息被丢弃（或阻塞发布者）

  3. 磁盘 I/O 瓶颈
     → write() 调用阻塞 → writer_thread_ 停滞
     → 连锁造成 write_cache_ 堆积
```

### 5.2 丢帧检测方案

**方案1：统计 expected vs actual 消息数**

```python
import rosbag2_py
import math

def check_frame_loss(bag_path: str):
    """检查 bag 文件的丢帧情况"""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr')
    )

    metadata = reader.get_metadata()
    duration_s = metadata.duration.nanoseconds / 1e9

    # 期望消息数（基于已知发布频率）
    EXPECTED_RATES = {
        '/joint_states': 1000,     # Hz
        '/imu/data': 200,
        '/camera/color/image_raw': 30,
        '/camera/depth/image_raw': 30,
    }

    print(f"Bag duration: {duration_s:.2f}s")
    print(f"\nFrame loss report:")
    print(f"{'Topic':<40} {'Expected':>10} {'Actual':>10} {'Loss%':>8}")
    print("-" * 72)

    for topic_info in metadata.topics_with_message_count:
        topic = topic_info.topic_metadata.name
        actual = topic_info.message_count

        if topic in EXPECTED_RATES:
            expected = int(EXPECTED_RATES[topic] * duration_s)
            loss_pct = max(0, (expected - actual) / expected * 100)
            status = "⚠️" if loss_pct > 1.0 else "✅"
            print(f"{status} {topic:<38} {expected:>10} {actual:>10} {loss_pct:>7.2f}%")
```

**方案2：时间戳连续性检查**

```python
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import numpy as np

def check_timestamp_continuity(bag_path: str, topic: str,
                                expected_hz: float, tolerance: float = 0.1):
    """检查时间戳间隔的连续性"""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr')
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    expected_interval_ns = int(1e9 / expected_hz)
    tolerance_ns = expected_interval_ns * tolerance

    timestamps = []
    while reader.has_next():
        (_, _, ts) = reader.read_next()
        timestamps.append(ts)

    if len(timestamps) < 2:
        return

    intervals = np.diff(timestamps)
    mean_interval = np.mean(intervals)
    std_interval = np.std(intervals)

    # 检测跳变（间隔 > 期望间隔 × 2）
    jumps = np.where(intervals > expected_interval_ns * 2)[0]

    print(f"\nTimestamp continuity: {topic}")
    print(f"  Expected interval: {expected_interval_ns/1e6:.2f}ms")
    print(f"  Mean interval:     {mean_interval/1e6:.2f}ms")
    print(f"  Std deviation:     {std_interval/1e6:.2f}ms")
    print(f"  Timestamp jumps:   {len(jumps)} (> 2× expected interval)")
    if len(jumps) > 0:
        for idx in jumps[:5]:  # 显示前5个
            print(f"    Jump at t={timestamps[idx]/1e9:.3f}s: "
                  f"gap={intervals[idx]/1e6:.1f}ms")
```

### 5.3 关节限位越界检测

```python
def check_joint_limits(bag_path: str, joint_limits: dict):
    """
    检查关节状态是否超出限位
    joint_limits = {
        'joint1': (-1.57, 1.57),  # (min_rad, max_rad)
        'joint2': (-0.5, 3.14),
        ...
    }
    """
    from sensor_msgs.msg import JointState

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr')
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/joint_states']))

    violations = []
    while reader.has_next():
        (_, data, ts) = reader.read_next()
        msg = deserialize_message(data, JointState)

        for i, name in enumerate(msg.name):
            if name in joint_limits and i < len(msg.position):
                lo, hi = joint_limits[name]
                pos = msg.position[i]
                if pos < lo or pos > hi:
                    violations.append({
                        'timestamp': ts,
                        'joint': name,
                        'position': pos,
                        'limit': (lo, hi)
                    })

    print(f"Joint limit violations: {len(violations)}")
    return violations
```

---

## 六、调优建议清单

### 6.1 操作系统层调优

```bash
# 1. 调整 I/O 调度器（NVMe 推荐 none 或 mq-deadline）
echo "none" > /sys/block/nvme0n1/queue/scheduler

# 2. 增加内核脏页写回阈值（允许更大 write buffer）
echo 80 > /proc/sys/vm/dirty_ratio
echo 40 > /proc/sys/vm/dirty_background_ratio
echo 500000 > /proc/sys/vm/dirty_expire_centisecs

# 3. 调整文件系统挂载选项（ext4/xfs）
# /etc/fstab:
# /dev/nvme0n1p1 /data ext4 noatime,nodiratime,discard,data=writeback 0 0

# 4. CPU 性能模式（避免动态降频导致写入抖动）
echo performance | tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# 5. 提高文件描述符限制
ulimit -n 65536
```

### 6.2 rosbag2 参数调优

```yaml
# 高吞吐具身智能录制配置
storage:
  storage_id: mcap
  max_cache_size: 1073741824     # 1GB 写缓存
  max_bagfile_size: 4294967296   # 4GB 分片（减少分片切换频率）
  compression_format: ""          # 写入时不压缩（避免 CPU 竞争控制环）

# QoS 配置（相机 topic 使用 BEST_EFFORT）
qos_overrides:
  /camera/color/image_raw:
    reliability: best_effort
    depth: 1                       # 只保留最新帧（防止队列堆积）
  /joint_states:
    reliability: reliable
    depth: 10
```

### 6.3 进程隔离

```bash
# 将录制节点绑定到非实时 CPU 核（0-3），控制环使用核 4-7
taskset -c 0-3 ros2 bag record -a -o /data/episode_001

# 或在 launch 文件中设置（需要 ros2_tracing 支持或手动 taskset）
# 具体见 Phase 7 实时性文档
```

### 6.4 网络传输建议

若录制节点与发布节点在**不同机器**：

```bash
# 增大 DDS 接收缓冲区（UDP）
echo 'net.core.rmem_max = 67108864' >> /etc/sysctl.conf
echo 'net.core.rmem_default = 67108864' >> /etc/sysctl.conf
echo 'net.ipv4.udp_mem = 65536 131072 262144' >> /etc/sysctl.conf
sysctl -p

# 使用 CycloneDDS 并配置大接收缓冲区
export CYCLONEDDS_URI='<CycloneDDS><Domain><General>
  <NetworkInterfaceAddress>eth0</NetworkInterfaceAddress>
</General><Compatibility>
  <ManySocketsMode>true</ManySocketsMode>
</Compatibility></Domain></CycloneDDS>'
```

### 6.5 调优效果汇总

| 优化措施 | 预期提升 | 适用场景 |
|---------|---------|---------|
| NVMe SSD（vs SATA）| 5-10× 写入速度 | 所有高带宽场景 |
| max_cache_size 1GB | 减少 I/O 抖动 | 图像高频录制 |
| 压缩图像 topic | 减少 80% 磁盘占用 | 存储受限场景 |
| lz4 per-chunk（mcap）| 减少 30-50% 磁盘 | 纯传感器/关节数据 |
| CPU 绑定隔离 | 控制环不受影响 | 与 ros2_control 同机 |
| I/O 调度器 = none | 减少 write 延迟 | NVMe 场景 |
| dirty_ratio 增大 | 减少 fsync 阻塞 | 内存充足场景 |
