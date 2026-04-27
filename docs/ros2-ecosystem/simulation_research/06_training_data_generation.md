# 仿真训练数据生成

> 本文档覆盖从仿真环境到 VLA 训练数据集的完整生成流水线：`omni.replicator` 合成数据生成、`rosbag2` 仿真录制、并行演示数据采集、数据质量评估以及混合数据集管理。

---

## 一、合成数据生成流水线（omni.replicator）

### 1.1 整体架构

```
omni.replicator 流水线

┌─────────────────────────────────────────────────────────────────┐
│  场景层                                                           │
│  USD Stage → 机器人 + 物体 + 光照 + 背景                          │
│  omni.replicator.core.randomizer → Domain Randomization         │
└──────────────────────────────────┬──────────────────────────────┘
                                   │ 每帧触发
┌──────────────────────────────────▼──────────────────────────────┐
│  Annotator 层                                                     │
│  ┌─────────────┐  ┌──────────────┐  ┌────────────────────────┐  │
│  │ LdrColorSD  │  │DepthLinearSD │  │ BoundingBox2DTightSD   │  │
│  │  (RGB)      │  │  (Depth)     │  │  (2D BBox)             │  │
│  ├─────────────┤  ├──────────────┤  ├────────────────────────┤  │
│  │InstanceSeg  │  │ SemanticSeg  │  │ ArticulationAction      │  │
│  │  (Mask)     │  │  (Seg)       │  │  (关节角 + 末端姿态)    │  │
│  └─────────────┘  └──────────────┘  └────────────────────────┘  │
└──────────────────────────────────┬──────────────────────────────┘
                                   │
┌──────────────────────────────────▼──────────────────────────────┐
│  Writer 层                                                        │
│  BasicWriter → COCO JSON / HDF5 / NPZ                            │
│  CustomWriter → LeRobot HDF5 格式                                │
└─────────────────────────────────────────────────────────────────┘
```

### 1.2 Python 脚本结构（伪代码）

```python
import omni.replicator.core as rep
import omni.isaac.core.utils.nucleus as nucleus_utils
import numpy as np
import h5py
import json

# ── 1. 初始化场景 ─────────────────────────────────────────────────
with rep.new_layer():
    # 加载机器人 USD
    robot = rep.create.from_usd(
        nucleus_utils.get_assets_root_path() + "/MyRobot/robot.usd",
        semantics=[('class', 'robot')]
    )
    # 加载待操作物体
    objects = rep.create.from_usd(
        nucleus_utils.get_assets_root_path() + "/Objects/", count=5,
        semantics=[('class', 'target_object')]
    )
    # 背景场景
    table = rep.create.plane(scale=(2, 2, 1), semantics=[('class', 'table')])

# ── 2. Domain Randomization 配置 ─────────────────────────────────
with rep.trigger.on_frame(num_frames=10000):
    # 光照随机化
    with rep.create.light(light_type='sphere', count=3):
        rep.modify.pose(
            position=rep.distribution.uniform((-3, -3, 2), (3, 3, 4))
        )
        rep.modify.attribute(
            "intensity", rep.distribution.uniform(500, 2000)
        )
        rep.modify.attribute(
            "color", rep.distribution.uniform((0.8, 0.8, 0.8), (1.0, 1.0, 1.0))
        )
    # 物体位姿随机化
    with objects:
        rep.modify.pose(
            position=rep.distribution.uniform((-0.3, -0.3, 0.05), (0.3, 0.3, 0.05)),
            rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 360))
        )
    # 材质纹理随机化
    with objects:
        rep.randomizer.texture(
            textures=rep.utils.get_usd_files(
                nucleus_utils.get_assets_root_path() + "/Materials/Textures/"
            )
        )

# ── 3. 注册 Annotator ─────────────────────────────────────────────
render_product = rep.create.render_product(
    "/World/Camera", resolution=(640, 480)
)
rgb_annotator = rep.AnnotatorRegistry.get_annotator("LdrColorSD")
depth_annotator = rep.AnnotatorRegistry.get_annotator("DistanceToImagePlaneSD")
seg_annotator = rep.AnnotatorRegistry.get_annotator("SemanticSegmentationSD")
bbox_annotator = rep.AnnotatorRegistry.get_annotator("BoundingBox2DTightSD")
joint_annotator = rep.AnnotatorRegistry.get_annotator("ArticulationAction")

rgb_annotator.attach(render_product)
depth_annotator.attach(render_product)
seg_annotator.attach(render_product)
bbox_annotator.attach(render_product)

# ── 4. 自定义 Writer（LeRobot HDF5 格式） ─────────────────────────
class LeRobotWriter(rep.WriterRegistry.get_writer("BasicWriter").__class__):
    def __init__(self, output_dir: str, episode_id: int = 0):
        self._hdf5_path = f"{output_dir}/episode_{episode_id:06d}.hdf5"
        self._buffer = []

    def write(self, data: dict):
        frame = {
            'observation.images.rgb': data['LdrColorSD'],           # (H, W, 3) uint8
            'observation.images.depth': data['DistanceToImagePlane'], # (H, W) float32
            'observation.state': data['ArticulationAction']['joint_positions'],  # (N_joints,)
            'action': data['ArticulationAction']['joint_velocities'],            # (N_joints,)
        }
        self._buffer.append(frame)

    def flush_episode(self, episode_id: int, task_name: str = "pick_place"):
        with h5py.File(self._hdf5_path, 'w') as f:
            f.attrs['task'] = task_name
            f.attrs['episode_id'] = episode_id
            for key in self._buffer[0].keys():
                data_arr = np.stack([step[key] for step in self._buffer], axis=0)
                f.create_dataset(key, data=data_arr, compression='gzip', chunks=True)
        self._buffer.clear()

# ── 5. 运行仿真 ───────────────────────────────────────────────────
rep.orchestrator.run()
```

### 1.3 输出格式对比

| 格式 | 适用场景 | 优势 | 劣势 |
|------|---------|------|------|
| **COCO JSON** | 目标检测 / 分割模型训练 | 生态兼容（Detectron2/MMDet） | 不含时序信息 |
| **HDF5（LeRobot）** | 模仿学习 / VLA 训练 | 高效压缩、顺序访问快 | 随机访问较慢 |
| **NPZ（NumPy）** | 快速原型 | 简单 | 无元数据、版本管理困难 |
| **WebDataset（TAR）** | 大规模流式训练 | 支持分布式 DataLoader | 写入复杂 |

---

## 二、rosbag2 仿真录制

### 2.1 完整 Topic 清单

```bash
# 仿真演示数据录制（含全量传感器）
ros2 bag record \
  --use-sim-time \
  --storage mcap \
  --compression-mode file \
  --compression-format zstd \
  -o sim_episode_001 \
  /joint_states \
  /joint_commands \
  /tf \
  /tf_static \
  /camera/color/image_raw \
  /camera/color/camera_info \
  /camera/depth/image_raw \
  /camera/depth/camera_info \
  /camera/depth/points \
  /imu/data \
  /wrench \
  /odom \
  /cmd_vel \
  /move_group/display_planned_path \
  /clock
```

| Topic | 消息类型 | 典型频率 | 占带宽 |
|-------|---------|---------|--------|
| `/joint_states` | `sensor_msgs/JointState` | 1000 Hz | ~2 MB/s |
| `/joint_commands` | `trajectory_msgs/JointTrajectory` | 50 Hz | 小 |
| `/tf` | `tf2_msgs/TFMessage` | 100 Hz | ~1 MB/s |
| `/camera/color/image_raw` | `sensor_msgs/Image` (1920×1080 RGB) | 30 fps | ~180 MB/s |
| `/camera/depth/image_raw` | `sensor_msgs/Image` (848×480 F32) | 30 fps | ~50 MB/s |
| `/imu/data` | `sensor_msgs/Imu` | 400 Hz | 小 |
| `/wrench` | `geometry_msgs/WrenchStamped` | 1000 Hz | ~2 MB/s |

> **存储建议**：图像使用 `--compression-mode message`（zstd 压缩单帧），可节省 60–70% 空间；`/joint_states` 等小消息使用 `file` 级压缩。

### 2.2 MCAP 分片配置

```yaml
# mcap_storage_config.yaml
storage:
  uri: sim_episode_001
  storage_id: mcap
  storage_config_uri: mcap_config.yaml

# mcap_config.yaml 内容：
chunk_size: 4194304    # 4MB chunks，兼顾压缩率和随机访问
use_chunking: true
compression: zstd
compression_level: 1   # 快速压缩（仿真录制时 CPU 压力）
```

---

## 三、具身智能演示数据生成

### 3.1 VR Controller / SpaceMouse 遥操作

```
遥操作演示数据生成流程：

┌──────────────────────────────────────────────────────────────────┐
│  输入设备层                                                        │
│  ├── SpaceMouse → /spacemouse/twist (TwistStamped)               │
│  ├── VR Controller (Meta Quest) → /vr/hand_pose (PoseStamped)    │
│  └── Keyboard/Joystick → /teleop/cmd (JointJog)                  │
└──────────────────────────────┬───────────────────────────────────┘
                               │
┌──────────────────────────────▼───────────────────────────────────┐
│  遥操作节点                                                        │
│  isaac_ros_teleop / teleop_twist_keyboard → MoveIt Servo         │
│  → /servo_node/delta_twist_cmds → ros2_control → 仿真关节         │
└──────────────────────────────┬───────────────────────────────────┘
                               │ 同时录制
┌──────────────────────────────▼───────────────────────────────────┐
│  rosbag2 录制（episode 级别）                                      │
│  按任务完成信号切分：/task_success (Bool) → 触发 bag split         │
└──────────────────────────────┬───────────────────────────────────┘
                               │
┌──────────────────────────────▼───────────────────────────────────┐
│  LeRobot HDF5 转换                                                 │
│  rosbag2_to_lerobot.py → episode_{:06d}.hdf5                     │
└──────────────────────────────────────────────────────────────────┘
```

### 3.2 Policy Rollout 录制（VLA 推理验证）

```python
# policy_rollout_recorder.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import WrenchStamped
import rosbag2_py
from rclpy.serialization import serialize_message

class PolicyRolloutRecorder(Node):
    """记录 VLA policy rollout，用于行为克隆质量验证"""

    def __init__(self):
        super().__init__('policy_rollout_recorder')
        self.writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(
            uri='policy_rollout_001', storage_id='mcap'
        )
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        # 注册 topic schema
        for topic, msg_type in [
            ('/camera/color/image_raw', 'sensor_msgs/msg/Image'),
            ('/joint_states', 'sensor_msgs/msg/JointState'),
            ('/vla/predicted_action', 'trajectory_msgs/msg/JointTrajectory'),
        ]:
            topic_info = rosbag2_py.TopicMetadata(
                name=topic, type=msg_type, serialization_format='cdr'
            )
            self.writer.create_topic(topic_info)

        # 订阅话题
        self.create_subscription(Image, '/camera/color/image_raw',
            lambda msg: self._record(msg, '/camera/color/image_raw'), 10)
        self.create_subscription(JointState, '/joint_states',
            lambda msg: self._record(msg, '/joint_states'), 10)

    def _record(self, msg, topic: str):
        self.writer.write(topic, serialize_message(msg),
                          self.get_clock().now().nanoseconds)
```

---

## 四、并行数据生成（Isaac Lab）

### 4.1 1024 环境并行架构

```python
# Isaac Lab 并行演示数据生成
from omni.isaac.lab.envs import ManagerBasedEnv
from omni.isaac.lab_tasks.utils.data_collector import RobotLearningDataCollector

# 创建 1024 个并行仿真环境
env_cfg = MyPickPlaceEnvCfg()
env_cfg.scene.num_envs = 1024
env_cfg.sim.device = 'cuda:0'
env_cfg.sim.use_fabric = True  # 启用 Isaac Lab USD Fabric 加速

env = ManagerBasedEnv(cfg=env_cfg)

# 数据收集器
collector = RobotLearningDataCollector(
    env=env,
    dataset_path="./datasets/sim_pick_place/",
    max_episodes_per_env=100,   # 每个并行环境收集 100 条 episode
    episode_len=200,            # 每条 episode 200 步
    export_format='hdf5',       # LeRobot 兼容格式
)

# 运行策略（oracle / scripted / teleop 回放）
oracle_policy = OraclePickPlacePolicy(env)
collector.collect(policy=oracle_policy, num_episodes=102400)  # 总计 10万条
```

### 4.2 HDF5 并行写入

```python
import h5py
import numpy as np
from concurrent.futures import ThreadPoolExecutor

class ParallelEpisodeWriter:
    """多线程 HDF5 写入，避免 GPU 数据传输阻塞仿真"""

    def __init__(self, output_dir: str, num_workers: int = 8):
        self._output_dir = output_dir
        self._executor = ThreadPoolExecutor(max_workers=num_workers)
        self._futures = []

    def write_episode_async(self, episode_id: int, data: dict):
        """异步写入，不阻塞仿真主循环"""
        future = self._executor.submit(self._write_episode, episode_id, data)
        self._futures.append(future)

    def _write_episode(self, episode_id: int, data: dict):
        path = f"{self._output_dir}/episode_{episode_id:06d}.hdf5"
        with h5py.File(path, 'w', libver='latest') as f:
            f.swmr_mode = True  # 允许并发读取（训练时）
            for key, arr in data.items():
                f.create_dataset(
                    key, data=arr,
                    compression='gzip', compression_opts=1,
                    chunks=(min(64, arr.shape[0]),) + arr.shape[1:],
                )

    def wait_all(self):
        for f in self._futures:
            f.result()  # 等待写入完成（epoch 结束时调用）
```

---

## 五、数据质量评估

### 5.1 多维度质量检查

```python
import numpy as np
from scipy import stats

class EpisodeQualityChecker:
    """自动过滤低质量 episode"""

    def __init__(self, joint_limits: dict, min_episode_len: int = 50):
        self.joint_limits = joint_limits  # {joint_name: (min, max)}
        self.min_episode_len = min_episode_len

    def check(self, episode: dict) -> tuple[bool, list[str]]:
        """返回 (is_valid, rejection_reasons)"""
        reasons = []

        # 1. Episode 长度检查
        T = episode['action'].shape[0]
        if T < self.min_episode_len:
            reasons.append(f"too_short: {T} < {self.min_episode_len}")

        # 2. 关节角越界检查
        joint_pos = episode['observation.state']  # (T, N_joints)
        for i, (jname, (jmin, jmax)) in enumerate(self.joint_limits.items()):
            violations = np.sum((joint_pos[:, i] < jmin) | (joint_pos[:, i] > jmax))
            if violations > 0:
                reasons.append(f"joint_limit_violation: {jname} ({violations} frames)")

        # 3. 动作抖动检查（急动度过大）
        actions = episode['action']
        jerk = np.diff(actions, n=2, axis=0)
        max_jerk = np.max(np.abs(jerk))
        if max_jerk > 50.0:  # rad/s³ 阈值
            reasons.append(f"excessive_jerk: {max_jerk:.1f} rad/s^3")

        # 4. 任务完成检查（末端到目标距离）
        if 'task_success' in episode and not episode['task_success']:
            reasons.append("task_failed")

        # 5. 图像质量检查（FID 检测近似：方差过低表示图像无信息量）
        if 'observation.images.rgb' in episode:
            img_var = np.var(episode['observation.images.rgb'].astype(float))
            if img_var < 100.0:
                reasons.append(f"low_image_variance: {img_var:.1f}")

        return len(reasons) == 0, reasons

    def filter_dataset(self, episodes: list[dict]) -> list[dict]:
        valid = []
        for ep in episodes:
            ok, reasons = self.check(ep)
            if ok:
                valid.append(ep)
            else:
                print(f"Rejected episode: {reasons}")
        print(f"Quality filter: {len(valid)}/{len(episodes)} episodes retained "
              f"({100*len(valid)/len(episodes):.1f}%)")
        return valid
```

### 5.2 关节角分布 / 力矩分布可视化

```python
import matplotlib.pyplot as plt

def plot_dataset_statistics(episodes: list[dict], save_path: str):
    joint_positions = np.concatenate([ep['observation.state'] for ep in episodes], axis=0)
    actions = np.concatenate([ep['action'] for ep in episodes], axis=0)

    fig, axes = plt.subplots(2, joint_positions.shape[1], figsize=(20, 8))
    for j in range(joint_positions.shape[1]):
        axes[0, j].hist(joint_positions[:, j], bins=100, density=True)
        axes[0, j].set_title(f'Joint {j} Position')
        axes[1, j].hist(actions[:, j], bins=100, density=True, color='orange')
        axes[1, j].set_title(f'Joint {j} Action')
    plt.tight_layout()
    plt.savefig(save_path)
```

---

## 六、混合数据集（Sim+Real）

### 6.1 Sim-to-Real 数据比例研究

基于 Droid / RT-2 / π0 等策略的实验结论：

| 数据集构成 | 任务成功率（零样本） | 注意事项 |
|----------|------------------|---------|
| 100% 真实数据 | 基准（~~55%~~ 依任务） | 数据采集成本最高 |
| 100% 仿真数据 | ~30%（Sim-to-Real gap） | 需充分 DR |
| **80% 仿真 + 20% 真实** | ~62%（略优于纯真实） | 仿真需高质量 RTX 渲染 |
| **50% 仿真 + 50% 真实** | ~68%（最优点，多项研究一致） | 平衡多样性与真实感 |
| **π0 混合策略**：大规模仿真预训练 → 少量真实微调 | ~75%+ | 预训练用低成本仿真，微调用真实 |

> **RT-2 结论**（Brohan et al., 2023）：仿真数据在提升 VLA 泛化性（new objects）方面作用显著，但质量比数量更重要——高质量 RTX 渲染仿真 1 万条 ≈ 低质量仿真 10 万条。

### 6.2 数据集混合配置

```python
# 混合真实 + 仿真数据集的 PyTorch Dataset
from torch.utils.data import ConcatDataset, WeightedRandomSampler
import lerobot

class MixedDataset:
    def __init__(self, real_path: str, sim_path: str,
                 real_weight: float = 0.5):
        self.real_ds = lerobot.LeRobotDataset(real_path)
        self.sim_ds = lerobot.LeRobotDataset(sim_path)
        self.combined = ConcatDataset([self.real_ds, self.sim_ds])

        # 加权采样：确保真实数据按 real_weight 比例采样
        n_real = len(self.real_ds)
        n_sim = len(self.sim_ds)
        weights_real = [real_weight / n_real] * n_real
        weights_sim = [(1 - real_weight) / n_sim] * n_sim
        self.sampler = WeightedRandomSampler(
            weights=weights_real + weights_sim,
            num_samples=len(self.combined),
            replacement=True,
        )
```

---

## 七、数据增强

### 7.1 图像级增强

```python
import torchvision.transforms as T

# 颜色抖动（应对 Sim-to-Real 颜色偏差）
color_jitter = T.ColorJitter(
    brightness=0.4,
    contrast=0.4,
    saturation=0.4,
    hue=0.1,
)

# 随机裁剪（增强位置鲁棒性）
random_crop = T.RandomResizedCrop(
    size=(480, 640),
    scale=(0.8, 1.0),
    ratio=(0.75, 1.33),
)

# 高斯模糊（模拟散焦/运动模糊）
gaussian_blur = T.GaussianBlur(kernel_size=5, sigma=(0.1, 2.0))
```

### 7.2 关节噪声注入

```python
def augment_joint_trajectory(
    joint_positions: np.ndarray,       # (T, N)
    noise_std: float = 0.01,           # rad，典型编码器分辨率
    dropout_prob: float = 0.02,        # 模拟传感器丢帧
) -> np.ndarray:
    """在关节角轨迹上注入高斯噪声和随机 dropout"""
    noisy = joint_positions + np.random.normal(0, noise_std, joint_positions.shape)

    # 随机 dropout（用上一帧值填充）
    dropout_mask = np.random.random(joint_positions.shape[0]) < dropout_prob
    for t in np.where(dropout_mask)[0]:
        if t > 0:
            noisy[t] = noisy[t - 1]

    return noisy
```

---

## 八、存储管理与版本控制

### 8.1 DVC 数据版本管理

```bash
# 初始化 DVC
dvc init
git add .dvc .dvcignore && git commit -m "Initialize DVC"

# 添加仿真数据集（存储到 S3/GCS 等远端）
dvc add datasets/sim_pick_place/
dvc push -r myremote

# 打标签对应模型版本
git tag -a "dataset-v1.2-sim50k" -m "50k sim episodes, DR v2.1"
dvc push --run-cache

# 查看历史
dvc dag
dvc metrics show
```

### 8.2 大规模数据集管理建议

| 规模 | 推荐方案 | 存储后端 | 访问模式 |
|------|---------|---------|---------|
| < 1TB | DVC + 本地 NAS | NFS / SATA SSD | 直接读 |
| 1–10TB | DVC + 对象存储 | S3 / GCS | 流式（WebDataset） |
| > 10TB | DVC + 分布式存储 | Ceph / HDFS | 预分片 + 缓存 |

```bash
# 估算仿真数据集大小
# 单条 episode（200步 × 30fps 相当于 ~7s）：
# RGB: 200 × 640×480×3 × (1/10 压缩比) ≈ 18MB
# Depth: 200 × 640×480×2 × (1/5) ≈ 24MB
# Joints: 200 × 7 × 4bytes ≈ 5.6KB
# 合计：约 42MB/episode

echo "100k episodes × 42MB = ~4.2TB（使用 zstd 压缩后约 1.5TB）"
```
