# 与本仓库已有组件的对接

> **关联文档**：
> - `docs/ros2-ecosystem/ros2_control_research/`（Phase 1）
> - `docs/embodied-ai/llm_vla_ros2_integration/07_data_flywheel.md`（Phase 3）
> - `docs/ros2-ecosystem/nav2_research/`（Phase 5）
> - `chassis_protocol/`（本仓库）

---

## 一、chassis_protocol 集成

### 1.1 chassis_node 发布的录制清单

`chassis_protocol` 实现了移动底盘的 HAL 层，其标准 topic 发布如下：

```yaml
# chassis_protocol 标准 topic 录制清单
chassis_recording:
  topics:
    # 里程计（nav_msgs/Odometry）
    - name: /odom
      type: nav_msgs/msg/Odometry
      hz: 50-100
      qos:
        reliability: reliable
        durability: volatile
        depth: 10

    # 速度命令（接收的指令，非发布）
    # 录制 cmd_vel 可追溯控制意图
    - name: /cmd_vel
      type: geometry_msgs/msg/Twist
      hz: 10-50
      qos:
        reliability: reliable
        durability: volatile
        depth: 10

    # 电机关节状态（如底盘驱动轮）
    # chassis_protocol 通过 ros2_control SystemInterface 提供
    - name: /joint_states
      type: sensor_msgs/msg/JointState
      hz: 100-1000     # 取决于 controller_manager update_rate
      qos:
        reliability: reliable
        durability: volatile
        depth: 10

    # IMU 数据（底盘 IMU）
    - name: /imu/data
      type: sensor_msgs/msg/Imu
      hz: 100-200
      qos:
        reliability: best_effort  # IMU 高频，通常 BEST_EFFORT
        durability: volatile
        depth: 1

    # 底盘诊断信息
    - name: /diagnostics
      type: diagnostic_msgs/msg/DiagnosticArray
      hz: 1-10
      qos:
        reliability: reliable
        durability: volatile
        depth: 10

    # TF（odom → base_link 变换）
    - name: /tf
      type: tf2_msgs/msg/TFMessage
      hz: 50-100
    - name: /tf_static
      type: tf2_msgs/msg/TFMessage

    # 电池状态（可选）
    - name: /battery_state
      type: sensor_msgs/msg/BatteryState
      hz: 1
```

### 1.2 QoS 配置（录制节点与 chassis_protocol 匹配）

```yaml
# /config/chassis_recording_qos.yaml
# 针对 chassis_protocol 的 QoS 覆盖配置

/odom:
  subscription:
    reliability: reliable      # chassis_protocol 发布 RELIABLE
    durability: volatile
    history: keep_last
    depth: 10

/joint_states:
  subscription:
    reliability: reliable
    durability: volatile
    history: keep_last
    depth: 10

/imu/data:
  subscription:
    reliability: best_effort   # IMU 发布 BEST_EFFORT
    durability: volatile
    history: keep_last
    depth: 1

/cmd_vel:
  subscription:
    reliability: reliable
    durability: volatile
    history: keep_last
    depth: 10
```

```bash
# 录制 chassis_protocol 数据
ros2 bag record \
  /odom /cmd_vel /joint_states /imu/data /tf /tf_static /diagnostics \
  -o /data/chassis_episode_001 \
  --storage mcap \
  --qos-profile-overrides-path /config/chassis_recording_qos.yaml
```

### 1.3 离线回放驱动 chassis_hal mock 方案

回放 bag 文件来测试上层算法（Nav2 / 感知模块）时，需要 chassis_hal mock：

```
离线回放架构：

bag 文件
  └─ /odom → 直接回放（驱动 Nav2 的定位）
  └─ /joint_states → 直接回放（驱动 ros2_control mock）
  └─ /cmd_vel → 不回放（由被测算法生成）

等效于：用 bag 数据替代真实机器人传感器

实现方案（launch 文件）：
```

```python
# chassis_mock_replay.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # 回放传感器 topic（odom、joint_states、imu）
        # 不回放 /cmd_vel（由被测控制算法生成）
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'play',
                '/data/chassis_episode_001',
                '--clock',
                '--topics', '/odom', '/joint_states', '/imu/data', '/tf', '/tf_static',
                # 不包含 /cmd_vel！让 Nav2 生成新的控制指令
            ],
            output='screen'
        ),

        # 启动 Nav2（use_sim_time=True，跟随 /clock）
        # 此时 Nav2 接收回放的 /odom 和 /tf，输出新的 /cmd_vel
        # 可以对比 bag 中记录的原始 /cmd_vel 与重放的差异
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            parameters=[{'use_sim_time': True}]
        ),
    ])
```

---

## 二、Phase 3 数据飞轮闭环

### 2.1 与 Phase 3 `07_data_flywheel.md` 的对应关系

Phase 3 文档描述了数据飞轮的**整体架构**，本 Phase 补充了底层 **rosbag2_cpp API 实现细节**：

| Phase 3 设计目标 | Phase 6 实现细节 |
|----------------|----------------|
| rosbag2 录制配置 | `RecordOptions` 参数全览（`03_transport_recording.md §2`）|
| 清洗流水线 | `DataQualityChecker` 实现（`05_imitation_learning_schema.md §8`）|
| LeRobot 格式转换 | `convert_bag_to_lerobot()` 完整实现（`05_imitation_learning_schema.md §6`）|
| `align_observations` | `AlignObservations` 工具实现思路（`05_imitation_learning_schema.md §7`）|
| sim2real bag format | Isaac Sim → rosbag2 配置（本文档 §4）|
| 存储格式选型 | mcap vs sqlite3 深度对比（`02_storage_plugins.md`）|

### 2.2 完整数据飞轮脚本（补充 rosbag2_cpp 细节）

```python
#!/usr/bin/env python3
"""
数据飞轮完整流水线
Phase 3 设计 + Phase 6 实现细节
"""

import os
from pathlib import Path
from dataclasses import dataclass, field
from typing import List, Optional

@dataclass
class FlyWheelConfig:
    # 输入 bags 目录
    bags_dir: str = '/data/raw_bags'
    # 临时处理目录
    processed_dir: str = '/data/processed'
    # 最终 LeRobot 数据集目录
    lerobot_dir: str = '/data/lerobot_dataset'

    # 录制配置
    storage_id: str = 'mcap'
    max_bag_size_gb: float = 4.0
    max_cache_size_mb: float = 500.0

    # 质量过滤阈值
    max_missing_frame_rate: float = 0.05  # 5%
    max_timestamp_jumps: int = 0
    allow_joint_violations: bool = False

    # LeRobot 格式配置
    image_height: int = 480
    image_width: int = 640
    reference_hz: int = 30
    action_chunk_size: int = 1


class DataFlywheel:
    """具身智能数据飞轮完整实现"""

    def __init__(self, config: FlyWheelConfig):
        self.config = config

    def step1_record(self, episode_name: str) -> str:
        """步骤1：录制（返回 bag 路径）"""
        bag_path = os.path.join(self.config.bags_dir, episode_name)
        record_cmd = [
            'ros2', 'bag', 'record',
            '-o', bag_path,
            '--storage', self.config.storage_id,
            f'--max-bag-size={int(self.config.max_bag_size_gb * 1024**3)}',
            '/joint_states', '/joint_commands',
            '/camera/color/image_raw', '/camera/depth/image_raw',
            '/imu/data', '/wrench', '/tf', '/tf_static', '/episode_marker',
            '--qos-profile-overrides-path', '/config/qos_overrides.yaml',
        ]
        print(f"Recording: {' '.join(record_cmd)}")
        return bag_path

    def step2_quality_check(self, bag_path: str) -> bool:
        """步骤2：质量检查"""
        # 使用 05_imitation_learning_schema.md 中的 DataQualityChecker
        from check_quality import check_bag_quality  # 伪代码
        result = check_bag_quality(bag_path)

        if result['missing_frame_rate'] > self.config.max_missing_frame_rate:
            print(f"REJECT: missing_frame_rate={result['missing_frame_rate']:.1%}")
            return False

        if result['timestamp_jumps'] > self.config.max_timestamp_jumps:
            print(f"REJECT: timestamp_jumps={result['timestamp_jumps']}")
            return False

        return True

    def step3_convert(self, bag_path: str, episode_id: int) -> str:
        """步骤3：转换为 LeRobot HDF5"""
        from convert import convert_bag_to_lerobot  # 伪代码
        output_path = os.path.join(
            self.config.lerobot_dir, f'episode_{episode_id:06d}.hdf5')

        result = convert_bag_to_lerobot(bag_path, output_path, episode_id)
        print(f"Converted: {result['n_frames']} frames → {output_path}")
        return output_path

    def step4_update_index(self, dataset_dir: str):
        """步骤4：更新数据集索引（LeRobot dataset_meta.json）"""
        import json, h5py, glob

        episodes = []
        for hdf5_path in sorted(glob.glob(f'{dataset_dir}/episode_*.hdf5')):
            with h5py.File(hdf5_path, 'r') as f:
                n_frames = f.attrs.get('n_frames', 0)
                episodes.append({
                    'id': f.attrs.get('episode_id', -1),
                    'n_frames': int(n_frames),
                    'path': os.path.basename(hdf5_path),
                })

        meta = {
            'total_episodes': len(episodes),
            'total_frames': sum(e['n_frames'] for e in episodes),
            'episodes': episodes,
        }
        with open(f'{dataset_dir}/dataset_meta.json', 'w') as f:
            json.dump(meta, f, indent=2)
        print(f"Dataset index updated: {meta['total_episodes']} episodes, "
              f"{meta['total_frames']} frames")
```

---

## 三、Phase 7 实时性对接

### 3.1 录制节点的资源占用分析

rosbag2 录制节点对实时控制环的潜在影响：

```
资源占用分析：

CPU：
  - writer_thread_：1 个后台线程，持续进行磁盘写入
  - 压缩线程（如配置）：N 个线程（N = compression_threads）
  - GenericSubscription 回调：在 Executor 线程中执行（反序列化 CDR）
  典型 CPU 占用（无压缩，双相机+关节）：15-25%

内存：
  - write_cache_：默认 100MB，建议 500MB
  - 操作系统 page cache（脏页）：取决于磁盘写入速度
  典型内存占用：500MB-1GB

磁盘 I/O：
  - 顺序写入 50-100 MB/s（具身智能标准配置）
  - 可能触发 I/O 调度延迟，影响同机器上的实时进程

对 ros2_control 1kHz 控制环的影响：
  - CPU 抢占：如果录制节点占用 CPU 核心 > 50%，可能导致控制环 jitter
  - 建议：将录制节点绑定到非实时 CPU 核（见下方隔离方案）
```

### 3.2 进程隔离方案

```bash
# 方案1：CPU 亲和性绑定（最简单）
# 假设系统有 8 核（0-7）
# 控制环使用核 6-7（配合 PREEMPT_RT 和 SCHED_FIFO）
# 录制节点使用核 0-5

taskset -c 0-5 ros2 bag record -a -o /data/episode

# 方案2：cgroup v2 资源限制
# 创建录制专用 cgroup
mkdir /sys/fs/cgroup/rosbag2_record
echo "0-5" > /sys/fs/cgroup/rosbag2_record/cpuset.cpus
echo "1" > /sys/fs/cgroup/rosbag2_record/cpuset.mems
# 将录制进程加入 cgroup
echo <PID> > /sys/fs/cgroup/rosbag2_record/cgroup.procs

# 方案3：独立容器（最强隔离）
# 录制节点在独立 Docker 容器中运行
# 通过 /dev/shm 或网络（DDS）接收来自控制容器的 topic
docker run --cpuset-cpus="0-5" --memory="2g" \
  --network host \
  ros:humble-ros-base \
  ros2 bag record -a -o /data/episode
```

### 3.3 与 Phase 7 实时性调研的接口

```
Phase 6 录制 ↔ Phase 7 实时性的关键交叉点：

1. iceoryx 共享内存与 rosbag2
   - iceoryx/rmw_iceoryx 支持 Loaned Message
   - 录制节点订阅 iceoryx topic 时，CDR 序列化成为瓶颈
   - 解决方案：使用 Typed Subscriber（rosbag2 1.x 计划中）

2. PREEMPT_RT 内核下的录制
   - writer_thread_ 不应设置 SCHED_FIFO（非实时任务）
   - GenericSubscription 回调在 Executor 线程中：
     → 如果 Executor 是实时优先级，录制回调会拖慢实时线程
     → 建议：单独的 rclcpp::executors::SingleThreadedExecutor
       给录制节点（非实时优先级）

3. 录制节点优先级设置（Phase 7 建议方案）
   # 正常优先级（SCHED_OTHER）
   chrt -o 0 $(pgrep -f 'ros2 bag record')
   # 或在启动脚本中
   nice -n 10 ros2 bag record ...
```

---

## 四、Phase 9 仿真数据生成

### 4.1 Isaac Sim → rosbag2 录制配置

Isaac Sim 通过 `omni.isaac.ros2_bridge`（或新版 `omni.isaac.ros2`）发布 ROS 2 topic：

```python
# Isaac Sim OmniGraph 配置（伪代码，Action Graph 节点）
# 发布关节状态
ArticulationReader → JointStatePublisher → /joint_states

# 发布 RGB 图像
RenderProduct → IsaacSimulationGate → ROS2CameraHelper → /camera/color/image_raw

# 发布 TF
IsaacReadSimulationTime → ROS2Context → ROS2PublishTransformTree → /tf

# 仿真时钟
SimulationContext → ROS2PublishClock → /clock
```

```bash
# 在 Isaac Sim 运行时录制 bag
ros2 bag record -a \
  -o /data/sim_episode_001 \
  --storage mcap \
  --use-sim-time \              # 使用 Isaac 的 /clock 打时间戳
  --qos-profile-overrides-path /config/sim_qos.yaml
```

```yaml
# /config/sim_qos.yaml — Isaac Sim topic 的 QoS 适配
/joint_states:
  subscription:
    reliability: reliable
    depth: 10

/camera/color/image_raw:
  subscription:
    reliability: best_effort    # Isaac Sim 图像通常 BEST_EFFORT
    depth: 1
```

### 4.2 Sim-to-Real Bag Format 统一

为保证仿真数据与真机数据格式一致（便于混合训练）：

```yaml
# 统一 topic 命名规范（sim 和 real 一致）
unified_topics:
  joint_states: /joint_states    # sensor_msgs/JointState
  joint_commands: /joint_commands
  rgb_image: /camera/color/image_raw
  depth_image: /camera/depth/image_raw
  camera_info: /camera/color/camera_info
  imu: /imu/data
  wrench: /wrench
  tf: /tf
  tf_static: /tf_static
  episode_marker: /episode_marker

# Isaac Sim 中需要将默认 topic 名称重映射到统一名称
# 例：/rgb → /camera/color/image_raw
ros2 run isaac_ros_bridge rgb_republisher \
  --ros-args --remap /rgb:=/camera/color/image_raw
```

### 4.3 仿真数据质量校验

```python
def validate_sim_bag(bag_path: str):
    """验证 Isaac Sim 录制的 bag 数据质量"""

    # 特别关注仿真特有问题：
    # 1. 时间戳单调性（仿真时钟重置时会跳变）
    # 2. 图像与关节同步（仿真步长一致性）
    # 3. TF 完整性（所有 frame 都有 static 定义）

    issues = []

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))

    prev_ts = 0
    ts_resets = 0
    while reader.has_next():
        (_, _, ts) = reader.read_next()
        if ts < prev_ts:
            ts_resets += 1
        prev_ts = ts

    if ts_resets > 0:
        issues.append(f"时间戳重置 {ts_resets} 次（仿真 reset 导致）")

    return issues
```

---

## 五、整体数据管线架构图

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                     具身智能完整数据管线（Phase 6 视角）                       │
│                                                                              │
│  ┌──────────────────────────┐    ┌──────────────────────────────────────┐   │
│  │  真机采集                │    │  仿真采集（Phase 9 + Phase 6）        │   │
│  │  chassis_protocol +      │    │  Isaac Sim omni.isaac.ros2_bridge    │   │
│  │  ros2_control（Phase 1） │    │  Gazebo Harmonic ros_gz_bridge       │   │
│  │  机械臂 + 夹爪 + 底盘     │    │  → domain randomization             │   │
│  └───────────┬──────────────┘    └──────────────┬───────────────────────┘   │
│              │                                   │                          │
│              └──────────────────┬────────────────┘                          │
│                                 │                                            │
│                  ┌──────────────▼──────────────┐                            │
│                  │  rosbag2 录制                │                            │
│                  │  storage: mcap               │                            │
│                  │  压缩: lz4（可选）            │                            │
│                  │  分片: 4GB/episode           │                            │
│                  └──────────────┬──────────────┘                            │
│                                 │ raw bag files                             │
│                  ┌──────────────▼──────────────┐                            │
│                  │  数据质量检查                 │                            │
│                  │  DataQualityChecker          │                            │
│                  │  - 缺帧率 < 5%               │──── REJECT ──→ 丢弃         │
│                  │  - 时间戳连续性              │                            │
│                  │  - 关节限位检查              │                            │
│                  └──────────────┬──────────────┘                            │
│                                 │ qualified bags                            │
│                  ┌──────────────▼──────────────┐                            │
│                  │  观测同步 + 动作对齐          │                            │
│                  │  OfflineSynchronizer          │                            │
│                  │  align_actions_to_observations│                            │
│                  └──────────────┬──────────────┘                            │
│                                 │ synced episodes                           │
│                  ┌──────────────▼──────────────┐                            │
│                  │  LeRobot HDF5 转换           │                            │
│                  │  convert_bag_to_lerobot()    │                            │
│                  │  episode_XXXXXX.hdf5         │                            │
│                  └──────────────┬──────────────┘                            │
│                                 │ HDF5 dataset                              │
│                  ┌──────────────▼──────────────┐                            │
│                  │  VLA / BC 训练               │                            │
│                  │  LeRobot ACT / Diffusion     │                            │
│                  │  Policy / π0 / RDT           │                            │
│                  └──────────────┬──────────────┘                            │
│                                 │ trained policy                            │
│                  ┌──────────────▼──────────────┐                            │
│                  │  部署 & 验证                  │                            │
│                  │  VlaActionBridgeNode（Phase 3）│                           │
│                  │  MoveIt Servo / JTC（Phase 4）│                           │
│                  │  回放验证（Foxglove）          │                           │
│                  └─────────────────────────────┘                            │
│                                 │                                            │
│                    ←──── 新数据采集，循环迭代 ────                             │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## 六、选型决策矩阵

### 6.1 存储格式选型

| 场景 | MCAP | SQLite3 | 建议 |
|------|------|---------|------|
| 真机高频录制（> 50MB/s）| ✅ 优秀 | ⚠️ 性能上限 | **MCAP** |
| 低频传感器录制（< 5MB/s）| ✅ | ✅ | MCAP（统一格式）|
| 需要 SQL 离线分析 | ❌ 需转换 | ✅ 原生 | SQLite3 |
| Foxglove 可视化 | ✅ 原生 | ⚠️ 需 MCAP 转换 | **MCAP** |
| 随机 seek 性能 | ✅ O(1) | ⚠️ O(logN) | **MCAP** |
| ROS 1 bag 迁移 | ⚠️ 需转换 | ✅ 部分兼容 | 取决于工具链 |
| 嵌入式存储受限 | ✅ 更小 | ✅ | MCAP |

**结论：具身智能场景统一使用 MCAP**

### 6.2 压缩开关决策

| 场景 | 压缩格式 | 压缩模式 | 理由 |
|------|---------|---------|------|
| 实时采集（真机）| 无压缩 | - | 优先保证写入吞吐，避免 CPU 竞争控制环 |
| 仿真采集 | lz4 | message（MCAP chunk）| 仿真 CPU 资源充足 |
| 长期归档 | zstd level 3 | file | 最高压缩比，离线操作 |
| 图像 topic | 使用 compressed 图像 | - | image_transport 压缩比 ZSTD 高 10× |
| 纯传感器（无图像）| zstd level 1 | message | 关节/IMU 数据压缩效果好 |

### 6.3 零拷贝开关决策

| 场景 | 启用零拷贝 | 禁用零拷贝 | 建议 |
|------|-----------|-----------|------|
| 录制节点作为 Component | 有效（IPC）| - | **启用**（默认）|
| 跨进程录制 | 无效（DDS 仍序列化）| - | 无关 |
| 回放（发布端）| 节省订阅者拷贝 | - | **启用**（默认）|
| 调试/问题排查 | - | 使用 `--disable-loan-message` | 禁用以简化问题定位 |
| rmw_fastrtps（不支持 loan）| 无效 | - | 无关 |
| rmw_cyclonedds / rmw_zenoh | 有效 | - | **启用** |

### 6.4 录制节点部署拓扑决策

```
部署拓扑选择树：

Q1: 录制节点与控制节点是否在同一机器？
  → 是：
    Q2: 是否有实时控制环（ros2_control 1kHz）？
      → 是：taskset 隔离 CPU 核心；禁用压缩；max_cache_size 增大
      → 否：标准配置
  → 否（录制节点在独立 NAS/工作站）：
    Q3: 网络带宽是否足够？（≥ 1Gbps）
      → 是：通过 DDS 网络录制（注意 UDP buffer 设置）
      → 否：使用 mcap remote relay（中继 + 本地写入）
```

### 6.5 综合推荐配置

```yaml
# 具身智能真机采集 — 综合最优配置

# 录制命令
ros2 bag record \
  /joint_states /joint_commands \
  /camera/color/image_raw/compressed \    # 使用 compressed 节省带宽
  /camera/depth/image_raw/compressedDepth \
  /imu/data /wrench /tf /tf_static /episode_marker \
  -o /data/bags/episode_${DATE} \
  --storage mcap \
  --max-bag-size 4294967296 \             # 4GB 分片
  --qos-profile-overrides-path /config/qos_overrides.yaml

# 存储选项
storage_options:
  storage_id: mcap
  max_cache_size: 524288000              # 500MB 写缓存
  compression_format: ""                 # 不压缩（图像已在 topic 层压缩）
  storage_preset_profile: "resilient"   # 开启 CRC 校验

# 离线归档（录制后执行）
ros2 bag convert \
  -i /data/bags/episode_${DATE} \
  -o /data/archive/episode_${DATE}_compressed \
  --output-options archive_config.yaml
```
