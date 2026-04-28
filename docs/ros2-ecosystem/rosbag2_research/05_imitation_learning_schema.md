# 面向模仿学习的数据 Schema 设计

> **参考来源**：`ros2/rosbag2@humble`；`ros-perception/message_filters@humble`；  
> LeRobot：`huggingface/lerobot@main`；Phase 3：`docs/embodied-ai/llm_vla_ros2_integration/07_data_flywheel.md`

---

## 一、模仿学习数据需求分析

模仿学习（Imitation Learning / Behavior Cloning）的训练数据需要满足：

1. **观测完整性**：每个训练样本包含同一时刻的所有传感器观测
2. **动作对齐**：控制指令与对应时刻的观测精确对齐（时间戳匹配）
3. **Episode 独立性**：每个 episode 是一个独立的任务执行序列
4. **低噪声**：时间戳抖动、缺帧、越界数据需过滤

```
模仿学习数据流水线：

rosbag2 录制
    │
    ▼
离线数据处理（Python）
    ├─ 观测同步（ApproximateTimeSynchronizer 离线重现）
    ├─ 动作对齐（高频控制指令 → 低频视觉帧）
    ├─ Episode 切分
    └─ 质量过滤
    │
    ▼
LeRobot HDF5 格式
    ├─ observations/  (N × obs_dim)
    ├─ actions/       (N × act_dim)
    ├─ episode_index  (N,)
    └─ timestamps     (N,)
    │
    ▼
VLA / BC 训练（LeRobot / ACT / Diffusion Policy）
```

---

## 二、标准观测 Topic 清单

### 2.1 具身智能标准观测集

```yaml
# 观测类型分类

# 本体感知（Proprioception）
/joint_states:
  type: sensor_msgs/msg/JointState
  fields:
    - name: ['joint1', 'joint2', ..., 'joint6']
    - position: [rad] × N_joints      ← 主要观测
    - velocity: [rad/s] × N_joints    ← 可选
    - effort: [Nm] × N_joints         ← 可选（需要力矩传感器）
  hz: 1000

/wrench:
  type: geometry_msgs/msg/WrenchStamped
  fields:
    - force:  {x, y, z} [N]
    - torque: {x, y, z} [Nm]
  hz: 500

# 末端位姿（来自 TF）
/tf + /tf_static:
  type: tf2_msgs/msg/TFMessage
  frames:
    - world → base_link → ... → ee_link  ← 末端执行器位姿
  hz: 50-100 (publication rate)

# 视觉（Vision）
/camera/color/image_raw:
  type: sensor_msgs/msg/Image
  encoding: rgb8 | bgr8
  resolution: 640×480 | 1280×720
  hz: 30

/camera/depth/image_raw:
  type: sensor_msgs/msg/Image
  encoding: 16UC1 (mm) | 32FC1 (m)
  resolution: 640×480 | 1280×720
  hz: 30

/camera/color/camera_info:
  type: sensor_msgs/msg/CameraInfo
  note: 包含内参矩阵 K（标定参数），每帧必须配对存储

# 惯性（Inertial）
/imu/data:
  type: sensor_msgs/msg/Imu
  fields:
    - angular_velocity: {x, y, z} [rad/s]
    - linear_acceleration: {x, y, z} [m/s²]
    - orientation: quaternion（如有 AHRS）
  hz: 200

# 动作标注（Action）
/joint_commands:
  type: std_msgs/msg/Float64MultiArray | trajectory_msgs/msg/JointTrajectory
  note: 控制器接收到的关节目标位置/速度
  hz: 1000 (同 joint_states)

# Episode 控制
/episode_marker:
  type: std_msgs/msg/String
  values: ["START", "END", "RESET", "GOOD", "BAD"]
  note: 人工标注 episode 边界和质量
```

---

## 三、观测同步策略

### 3.1 在线同步（录制时）

对于实时推理，使用 `message_filters::ApproximateTimeSynchronizer`：

```cpp
// 在线同步（仅用于推理，不用于录制）
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"

using namespace message_filters;
using ApproxSync = sync_policies::ApproximateTime<
    sensor_msgs::msg::JointState,
    sensor_msgs::msg::Image,
    sensor_msgs::msg::Image>;

// 缓冲 10 条消息，时间容忍 50ms
Synchronizer<ApproxSync> sync(ApproxSync(10), sub_joint, sub_rgb, sub_depth);
sync.setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.05));
sync.registerCallback(&my_callback);
```

**录制时不做同步**：直接录制各 topic 的原始时间序列，同步在离线处理时进行。原因：
- 避免录制节点因同步等待而丢帧
- 保留原始时间戳，支持事后灵活配置同步策略

### 3.2 离线同步（处理时）

```python
import rosbag2_py
import numpy as np
from collections import defaultdict
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

class OfflineSynchronizer:
    """
    离线实现 ApproximateTimeSynchronizer 逻辑
    输入：多个 topic 的 (timestamp, message) 序列
    输出：时间对齐后的 (t, joint_state, rgb_image, depth_image) 元组
    """

    def __init__(self, max_time_diff_ns: int = 50_000_000):  # 50ms
        self.max_time_diff_ns = max_time_diff_ns

    def sync(self, topic_buffers: dict) -> list:
        """
        topic_buffers: {
            '/joint_states': [(ts_ns, msg), ...],
            '/camera/color/image_raw': [(ts_ns, msg), ...],
            '/camera/depth/image_raw': [(ts_ns, msg), ...],
        }
        """
        # 以最低频率 topic（RGB）为基准
        reference_topic = '/camera/color/image_raw'
        reference_msgs = topic_buffers[reference_topic]

        synced_samples = []

        for ref_ts, ref_msg in reference_msgs:
            sample = {'timestamp': ref_ts, reference_topic: ref_msg}
            valid = True

            for topic, msgs in topic_buffers.items():
                if topic == reference_topic:
                    continue

                # 找到最近的消息
                closest = self._find_closest(msgs, ref_ts)
                if closest is None:
                    valid = False
                    break

                closest_ts, closest_msg = closest
                if abs(closest_ts - ref_ts) > self.max_time_diff_ns:
                    valid = False
                    break

                sample[topic] = closest_msg
                sample[f'{topic}_ts'] = closest_ts

            if valid:
                synced_samples.append(sample)

        return synced_samples

    def _find_closest(self, msgs: list, target_ts: int):
        """二分查找最近时间戳的消息"""
        if not msgs:
            return None

        lo, hi = 0, len(msgs) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if msgs[mid][0] < target_ts:
                lo = mid + 1
            else:
                hi = mid

        # 检查 lo-1 和 lo 哪个更近
        candidates = [(abs(msgs[i][0] - target_ts), i)
                      for i in [max(0, lo-1), min(lo, len(msgs)-1)]]
        candidates.sort()
        best_i = candidates[0][1]
        return msgs[best_i]


def load_all_topics(bag_path: str, topics: list) -> dict:
    """从 bag 文件加载多个 topic 的全部消息"""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr')
    )

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))

    buffers = defaultdict(list)
    while reader.has_next():
        (topic, data, ts) = reader.read_next()
        msg_type = get_message(topic_types[topic])
        msg = deserialize_message(data, msg_type)
        buffers[topic].append((ts, msg))

    return dict(buffers)
```

---

## 四、动作对齐

### 4.1 高频控制指令与低频视觉帧的对齐

```
对齐问题：
  关节状态：1000 Hz → 每 1ms 一条
  RGB 图像：30 Hz  → 每 33.3ms 一条

对齐策略（以视觉帧为基准）：

策略1：最近邻（Nearest Neighbor）
  对每个 RGB 帧时间戳 t_i，找最近的关节状态
  时间差 <= 10ms 认为有效
  优点：简单；缺点：可能引入最多 0.5ms × action_delay 误差

策略2：线性插值（Linear Interpolation）
  对每个 RGB 帧时间戳 t_i，取前后两条关节状态
  position_interp = lerp(pos_before, pos_after, alpha)
  优点：精确；适合关节位置这样的连续量
  注意：速度、力矩不应直接插值（应用差分）

策略3：动作窗口（Action Chunking）
  与 ACT (Action Chunking with Transformers) 一致：
  对每个 RGB 帧，提取未来 T 步的关节指令序列
  actions_chunk[i] = joint_commands[t_i : t_i + T]  # T=20, dt=50ms
  → 网络输出 action chunk，减少历史依赖
```

```python
def align_actions_to_observations(
    joint_commands: list,   # [(ts_ns, cmd_array), ...]
    obs_timestamps: list,   # [ts_ns, ...]，来自 RGB 帧
    interp_mode: str = 'linear',  # 'nearest' | 'linear'
    action_chunk_size: int = 1,   # 1 = 单步；>1 = action chunk
) -> np.ndarray:
    """
    将高频关节命令对齐到低频视觉帧时间戳
    返回：aligned_actions，shape = (N_obs, action_chunk_size, action_dim)
    """
    cmd_times = np.array([t for t, _ in joint_commands])
    cmd_values = np.array([v for _, v in joint_commands])
    action_dim = cmd_values.shape[1]

    aligned = np.zeros((len(obs_timestamps), action_chunk_size, action_dim))

    for i, obs_ts in enumerate(obs_timestamps):
        for j in range(action_chunk_size):
            # 目标时间戳（action chunk 第 j 步）
            target_ts = obs_ts + j * int(1e9 / 20)  # 假设 20Hz action

            if interp_mode == 'nearest':
                idx = np.argmin(np.abs(cmd_times - target_ts))
                aligned[i, j] = cmd_values[idx]

            elif interp_mode == 'linear':
                # 找前后两个点做线性插值
                idx_after = np.searchsorted(cmd_times, target_ts)
                idx_before = max(0, idx_after - 1)
                idx_after = min(idx_after, len(cmd_times) - 1)

                if idx_before == idx_after:
                    aligned[i, j] = cmd_values[idx_before]
                else:
                    t0, t1 = cmd_times[idx_before], cmd_times[idx_after]
                    alpha = (target_ts - t0) / max(t1 - t0, 1)
                    alpha = np.clip(alpha, 0, 1)
                    aligned[i, j] = (1 - alpha) * cmd_values[idx_before] + \
                                    alpha * cmd_values[idx_after]

    return aligned
```

---

## 五、Episode 切分

### 5.1 基于 /episode_marker topic 的切分

```
Episode 切分方案：

方案A：/episode_marker topic 标注（推荐）
  人工或脚本在每个 episode 开始/结束时发布：
  ros2 topic pub /episode_marker std_msgs/msg/String "data: 'START'" --once
  ros2 topic pub /episode_marker std_msgs/msg/String "data: 'END'" --once

  切分逻辑：
  episodes = []
  current_start = None
  for ts, msg in episode_markers:
    if msg.data == "START":
      current_start = ts
    elif msg.data == "END" and current_start is not None:
      episodes.append((current_start, ts))
      current_start = None

方案B：bag 文件分片（物理分片 = episode 分片）
  录制开始 = 新 bag 文件（ros2 service call .../split_bagfile）
  一个 bag 文件 = 一个 episode
  优点：无需额外标注；缺点：需要精确控制录制启停

方案C：静止检测自动切分
  检测关节速度均接近 0 的时间段 → 认为是 episode 间隙
  threshold = 0.05 rad/s（所有关节速度绝对值均低于此值）
  gap_duration > 2s → episode 边界
```

### 5.2 Episode 切分粒度建议

| 任务类型 | Episode 时长建议 | 切分粒度 |
|---------|---------------|---------|
| 抓取-放置（pick and place）| 3-10s | 每次抓取为一个 episode |
| 装配任务（assembly）| 30-120s | 每个子步骤为一个 episode |
| 擦桌子/清扫 | 10-60s | 每次来回为一个 episode |
| 手术缝合 | 60-300s | 每针为一个 episode |

**推荐做法**：每个 episode 对应一个 bag 分片，文件名编码 episode 信息：
```
/data/episodes/
├── task_pick_cup_001.mcap
├── task_pick_cup_002.mcap
├── task_pick_cup_003.mcap
└── metadata.json  ← episode 级元数据（操作者、环境、结果、质量评分）
```

---

## 六、LeRobot HDF5 格式转换

### 6.1 LeRobot 数据格式规范

LeRobot（Hugging Face）使用 HDF5 格式存储模仿学习数据集：

```
episode_XXXXXX.hdf5 结构（LeRobot v1.6 格式）：
├── /data/
│   ├── observation.state        (N, state_dim)  ← 关节位置
│   ├── observation.images.top   (N, H, W, 3)    ← RGB 图像（uint8）
│   ├── observation.images.wrist (N, H, W, 3)    ← 手腕摄像头（可选）
│   ├── action                   (N, act_dim)    ← 关节目标位置
│   ├── timestamp                (N,)            ← 相对时间（s）
│   ├── frame_index              (N,)            ← 帧序号
│   ├── episode_index            (N,)            ← episode ID（全局唯一）
│   ├── index                    (N,)            ← 全局行索引
│   ├── next.done                (N,)            ← episode 结束标志
│   └── next.reward              (N,)            ← 奖励（可选）
```

### 6.2 rosbag2 → LeRobot 转换脚本（完整设计）

```python
#!/usr/bin/env python3
"""
rosbag2 → LeRobot HDF5 转换脚本
支持：关节状态、RGB 图像、深度图像、力矩传感器
"""

import os
import json
import numpy as np
import h5py
import cv2
import rosbag2_py
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import JointState, Image, Imu
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

# ============================================================
# 配置
# ============================================================
class ConversionConfig:
    # 关节名称（与 URDF 一致）
    JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

    # Topic 映射
    JOINT_STATES_TOPIC = '/joint_states'
    JOINT_COMMANDS_TOPIC = '/joint_commands'
    RGB_TOPIC = '/camera/color/image_raw'
    DEPTH_TOPIC = '/camera/depth/image_raw'
    IMU_TOPIC = '/imu/data'
    WRENCH_TOPIC = '/wrench'
    EPISODE_MARKER_TOPIC = '/episode_marker'

    # 同步参数
    MAX_SYNC_DIFF_NS = 50_000_000   # 50ms
    REFERENCE_HZ = 30               # 以 RGB 帧率为基准

    # 输出图像尺寸
    IMAGE_HEIGHT = 480
    IMAGE_WIDTH = 640

    # Action chunk 大小（1 = 单步 BC）
    ACTION_CHUNK_SIZE = 1


# ============================================================
# 数据加载器
# ============================================================
class BagDataLoader:
    def __init__(self, bag_path: str, config: ConversionConfig):
        self.bag_path = bag_path
        self.config = config
        self.reader = rosbag2_py.SequentialReader()
        self.reader.open(
            rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap'),
            rosbag2_py.ConverterOptions('cdr', 'cdr')
        )
        self.topic_types = {t.name: t.type
                            for t in self.reader.get_all_topics_and_types()}

    def load_topic(self, topic: str) -> list:
        """加载单个 topic 的所有消息"""
        if topic not in self.topic_types:
            return []

        self.reader.seek(0)  # 重置到开头
        self.reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

        msg_type = get_message(self.topic_types[topic])
        messages = []
        while self.reader.has_next():
            (_, data, ts) = self.reader.read_next()
            msg = deserialize_message(data, msg_type)
            messages.append((ts, msg))

        return messages

    def load_all(self) -> dict:
        """加载所有相关 topic"""
        topics = [
            self.config.JOINT_STATES_TOPIC,
            self.config.JOINT_COMMANDS_TOPIC,
            self.config.RGB_TOPIC,
            self.config.DEPTH_TOPIC,
            self.config.WRENCH_TOPIC,
            self.config.EPISODE_MARKER_TOPIC,
        ]
        return {t: self.load_topic(t) for t in topics if t in self.topic_types}


# ============================================================
# 图像处理
# ============================================================
def decode_image_msg(msg: Image, target_h: int, target_w: int) -> np.ndarray:
    """将 sensor_msgs/Image 转为 HWC uint8 numpy array"""
    # 构建 OpenCV Mat
    if msg.encoding in ('rgb8', 'bgr8', 'rgba8', 'bgra8'):
        channels = 3 if 'rgb8' in msg.encoding or 'bgr8' in msg.encoding else 4
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, channels)
        if msg.encoding in ('rgb8', 'rgba8'):
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)  # → BGR for cv2
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # → back to RGB for LeRobot
    elif msg.encoding == '16UC1':
        img = np.frombuffer(msg.data, dtype=np.uint16).reshape(
            msg.height, msg.width)
        img = (img / 1000.0).astype(np.float32)  # mm → m
    else:
        raise ValueError(f"Unsupported encoding: {msg.encoding}")

    # Resize
    if img.shape[:2] != (target_h, target_w):
        img = cv2.resize(img, (target_w, target_h),
                         interpolation=cv2.INTER_LINEAR)

    return img


# ============================================================
# 主转换函数
# ============================================================
def convert_bag_to_lerobot(
    bag_path: str,
    output_path: str,
    episode_id: int,
    config: ConversionConfig = None
) -> dict:
    """
    将单个 rosbag2 文件转换为 LeRobot HDF5 格式

    返回：转换统计信息
    """
    if config is None:
        config = ConversionConfig()

    loader = BagDataLoader(bag_path, config)
    data = loader.load_all()

    # 1. 加载 RGB 帧（作为时间基准）
    rgb_msgs = data.get(config.RGB_TOPIC, [])
    if not rgb_msgs:
        raise ValueError(f"No RGB images found in {bag_path}")

    N = len(rgb_msgs)
    obs_timestamps = [ts for ts, _ in rgb_msgs]
    t0 = obs_timestamps[0]  # episode 开始时间

    # 2. 同步其他 topic 到 RGB 时间戳
    synchronizer = OfflineSynchronizer(config.MAX_SYNC_DIFF_NS)

    joint_msgs = data.get(config.JOINT_STATES_TOPIC, [])
    cmd_msgs = data.get(config.JOINT_COMMANDS_TOPIC, [])
    wrench_msgs = data.get(config.WRENCH_TOPIC, [])

    # 3. 构建输出数组
    n_joints = len(config.JOINT_NAMES)
    obs_state = np.zeros((N, n_joints), dtype=np.float32)
    obs_images = np.zeros((N, config.IMAGE_HEIGHT, config.IMAGE_WIDTH, 3),
                          dtype=np.uint8)
    actions = np.zeros((N, n_joints), dtype=np.float32)
    timestamps = np.zeros(N, dtype=np.float64)
    next_done = np.zeros(N, dtype=bool)

    for i, (obs_ts, rgb_msg) in enumerate(rgb_msgs):
        # 时间戳（相对秒）
        timestamps[i] = (obs_ts - t0) / 1e9

        # RGB 图像
        obs_images[i] = decode_image_msg(rgb_msg, config.IMAGE_HEIGHT,
                                         config.IMAGE_WIDTH)

        # 关节状态（最近邻匹配）
        if joint_msgs:
            closest = synchronizer._find_closest(joint_msgs, obs_ts)
            if closest:
                _, js_msg = closest
                for j, name in enumerate(config.JOINT_NAMES):
                    if name in js_msg.name:
                        idx = js_msg.name.index(name)
                        obs_state[i, j] = js_msg.position[idx]

        # 动作（关节命令，最近邻匹配）
        if cmd_msgs:
            closest = synchronizer._find_closest(cmd_msgs, obs_ts)
            if closest:
                _, cmd_ts_msg = closest
                # 假设 cmd 是 Float64MultiArray
                actions[i] = cmd_ts_msg.data[:n_joints]
        else:
            # 无显式命令时，用关节位置作为动作（行为克隆标准做法）
            actions[i] = obs_state[i]

    # 最后一帧标记 done
    next_done[-1] = True

    # 4. 写入 HDF5
    os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)
    with h5py.File(output_path, 'w') as f:
        grp = f.create_group('data')
        grp.create_dataset('observation.state', data=obs_state,
                           compression='gzip', compression_opts=4)
        grp.create_dataset('observation.images.top', data=obs_images,
                           compression='gzip', compression_opts=4,
                           chunks=(1, *obs_images.shape[1:]))
        grp.create_dataset('action', data=actions,
                           compression='gzip', compression_opts=4)
        grp.create_dataset('timestamp', data=timestamps)
        grp.create_dataset('frame_index', data=np.arange(N))
        grp.create_dataset('episode_index',
                           data=np.full(N, episode_id, dtype=np.int64))
        grp.create_dataset('index', data=np.arange(N))
        grp.create_dataset('next.done', data=next_done)

        # 元数据
        f.attrs['episode_id'] = episode_id
        f.attrs['source_bag'] = bag_path
        f.attrs['n_frames'] = N
        f.attrs['fps'] = config.REFERENCE_HZ
        f.attrs['joint_names'] = json.dumps(config.JOINT_NAMES)

    return {'episode_id': episode_id, 'n_frames': N, 'output': output_path}


# ============================================================
# 批量转换（目录级）
# ============================================================
def convert_bag_directory(
    bags_dir: str,
    output_dir: str,
    config: ConversionConfig = None
):
    """将目录下所有 .mcap bag 文件批量转换"""
    bags_dir = Path(bags_dir)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    bag_files = sorted(bags_dir.glob('*.mcap'))
    # 或支持目录格式的 bag
    bag_dirs = [d for d in bags_dir.iterdir()
                if d.is_dir() and (d / 'metadata.yaml').exists()]

    all_items = list(bag_files) + bag_dirs
    results = []

    for ep_id, bag_path in enumerate(all_items):
        output_path = output_dir / f"episode_{ep_id:06d}.hdf5"
        print(f"Converting episode {ep_id}: {bag_path}")

        try:
            result = convert_bag_to_lerobot(
                str(bag_path), str(output_path), ep_id, config)
            results.append(result)
        except Exception as e:
            print(f"  ERROR: {e}")
            results.append({'episode_id': ep_id, 'error': str(e)})

    # 写入数据集元数据
    dataset_meta = {
        'total_episodes': len(results),
        'total_frames': sum(r.get('n_frames', 0) for r in results),
        'joint_names': config.JOINT_NAMES if config else ConversionConfig.JOINT_NAMES,
        'episodes': results
    }
    with open(output_dir / 'dataset_meta.json', 'w') as f:
        json.dump(dataset_meta, f, indent=2)

    print(f"\nConversion complete: {len(results)} episodes → {output_dir}")
    return dataset_meta
```

---

## 七、align_observations 工具实现思路

Phase 3 `07_data_flywheel.md` 提到的 `align_observations` 工具，核心实现思路：

```python
class AlignObservations:
    """
    align_observations 工具 — 将多模态观测对齐到统一时间基准

    功能：
    1. 从 bag 文件读取多个 topic
    2. 以指定 reference_topic 的时间戳为基准
    3. 将其他 topic 通过插值/最近邻对齐
    4. 输出时间对齐的 numpy 数组字典

    设计原则：
    - 懒加载（按需读取，不预加载全部到内存）
    - 支持自定义插值函数（per-topic）
    - 支持时间窗口过滤（仅处理 episode 内的数据）
    """

    def __init__(self,
                 bag_path: str,
                 reference_topic: str = '/camera/color/image_raw',
                 max_sync_diff_ms: float = 50.0):
        self.bag_path = bag_path
        self.reference_topic = reference_topic
        self.max_sync_diff_ns = int(max_sync_diff_ms * 1e6)
        self._loaders = {}  # topic → callable(ts) → value
        self._registered_topics = {}

    def register(self, topic: str, extractor_fn, interp_mode: str = 'nearest'):
        """
        注册 topic 提取函数
        extractor_fn: (msg) → np.ndarray  将 ROS 消息转为 numpy 数组
        """
        self._registered_topics[topic] = (extractor_fn, interp_mode)

    def run(self, t_start_ns: int = None, t_end_ns: int = None) -> dict:
        """
        执行对齐，返回：
        {
            'timestamps': np.ndarray (N,),
            '/joint_states': np.ndarray (N, n_joints),
            '/camera/color/image_raw': np.ndarray (N, H, W, 3),
            ...
        }
        """
        # 加载 reference topic
        ref_msgs = self._load_topic(self.reference_topic, t_start_ns, t_end_ns)
        obs_timestamps = np.array([ts for ts, _ in ref_msgs])

        result = {'timestamps': (obs_timestamps - obs_timestamps[0]) / 1e9}

        # 加载并对齐其他 topic
        for topic, (extractor_fn, interp_mode) in self._registered_topics.items():
            topic_msgs = self._load_topic(topic, t_start_ns, t_end_ns)
            if not topic_msgs:
                continue

            topic_times = np.array([ts for ts, _ in topic_msgs])
            topic_values = np.array([extractor_fn(msg) for _, msg in topic_msgs])

            if interp_mode == 'nearest':
                indices = self._nearest_indices(topic_times, obs_timestamps)
                aligned = topic_values[indices]
            elif interp_mode == 'linear':
                aligned = self._linear_interp(
                    topic_times, topic_values, obs_timestamps)
            else:
                raise ValueError(f"Unknown interp_mode: {interp_mode}")

            result[topic] = aligned

        return result

    def _nearest_indices(self, source_times, target_times):
        indices = np.searchsorted(source_times, target_times, side='left')
        indices = np.clip(indices, 0, len(source_times) - 1)
        return indices

    def _linear_interp(self, source_times, source_values, target_times):
        # 使用 numpy 插值（per feature 维度）
        n_features = source_values.shape[1] if source_values.ndim > 1 else 1
        result = np.zeros((len(target_times), n_features))
        for j in range(n_features):
            result[:, j] = np.interp(target_times, source_times, source_values[:, j])
        return result
```

---

## 八、数据质量检查

### 8.1 完整质量检查管道

```python
class DataQualityChecker:
    """episode 级数据质量检查"""

    def check(self, hdf5_path: str, config: ConversionConfig = None) -> dict:
        if config is None:
            config = ConversionConfig()

        with h5py.File(hdf5_path, 'r') as f:
            data = f['data']
            obs_state = data['observation.state'][:]     # (N, n_joints)
            actions = data['action'][:]                   # (N, n_joints)
            timestamps = data['timestamp'][:]             # (N,)

        results = {}

        # 1. 缺帧率检查
        n_frames = len(timestamps)
        expected_frames = int(timestamps[-1] * config.REFERENCE_HZ) + 1
        missing_rate = 1.0 - n_frames / max(expected_frames, 1)
        results['missing_frame_rate'] = missing_rate
        results['missing_frame_ok'] = missing_rate < 0.05  # 允许 5% 丢帧

        # 2. 时间戳连续性检查
        dt = np.diff(timestamps)
        expected_dt = 1.0 / config.REFERENCE_HZ
        timestamp_jumps = np.sum(dt > expected_dt * 2)
        results['timestamp_jumps'] = int(timestamp_jumps)
        results['timestamp_ok'] = timestamp_jumps == 0

        # 3. 关节限位越界检查
        JOINT_LIMITS = {  # rad（示例，需根据实际机器人配置）
            0: (-2.09, 2.09), 1: (-2.09, 2.09), 2: (-2.09, 2.09),
            3: (-3.14, 3.14), 4: (-2.09, 2.09), 5: (-3.14, 3.14),
        }
        violations = 0
        for j, (lo, hi) in JOINT_LIMITS.items():
            if j < obs_state.shape[1]:
                violations += np.sum(
                    (obs_state[:, j] < lo) | (obs_state[:, j] > hi))
        results['joint_limit_violations'] = int(violations)
        results['joint_limits_ok'] = violations == 0

        # 4. 动作平滑性检查（高频抖动）
        action_diff = np.diff(actions, axis=0)
        max_action_step = np.max(np.abs(action_diff))
        results['max_action_step_rad'] = float(max_action_step)
        results['action_smooth_ok'] = max_action_step < 0.1  # 阈值 0.1 rad/frame

        # 5. 总体质量评分
        checks = ['missing_frame_ok', 'timestamp_ok',
                  'joint_limits_ok', 'action_smooth_ok']
        results['quality_score'] = sum(1 for c in checks if results[c]) / len(checks)
        results['quality_grade'] = 'A' if results['quality_score'] == 1.0 else \
                                   'B' if results['quality_score'] >= 0.75 else \
                                   'C' if results['quality_score'] >= 0.5 else 'F'

        return results
```

### 8.2 质量检查结果示例

```json
{
  "missing_frame_rate": 0.002,
  "missing_frame_ok": true,
  "timestamp_jumps": 0,
  "timestamp_ok": true,
  "joint_limit_violations": 0,
  "joint_limits_ok": true,
  "max_action_step_rad": 0.023,
  "action_smooth_ok": true,
  "quality_score": 1.0,
  "quality_grade": "A"
}
```
