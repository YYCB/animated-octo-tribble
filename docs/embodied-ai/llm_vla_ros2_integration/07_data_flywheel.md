# 数据飞轮 — rosbag2 采集 → 清洗 → VLA 训练 → 回放验证

---

## 一、数据飞轮概述

具身智能系统的能力提升依赖**数据飞轮**：机器人执行任务 → 收集成功/失败数据 → 训练更好的模型 → 机器人执行更好 → 循环。

```
┌──────────────────────────────────────────────────────────────────┐
│  数据飞轮全流程                                                    │
│                                                                  │
│  ① 人工遥操作 / 任务执行（机器人运行）                             │
│       ↓ rosbag2 录制                                             │
│  ② 原始 episode 数据（图像 + 动作 + 状态 + LLM 工具调用序列）       │
│       ↓ 数据清洗（过滤失败 episode，时间对齐）                     │
│  ③ 训练数据集（LeRobot 格式 / RLDS / HDF5）                       │
│       ↓ 模型训练（OpenVLA / π0 fine-tuning / RDT）               │
│  ④ 新模型（TensorRT engine / GGUF）                              │
│       ↓ 部署到 Triton / 本机 TRT                                 │
│  ⑤ 模型回放验证（rosbag2 回放 + 评估指标）                        │
│       ↓ 指标合格 → 上线；指标不合格 → 采集更多数据 → 回到 ①        │
└──────────────────────────────────────────────────────────────────┘
```

---

## 二、Episode 录制设计

### 2.1 rosbag2 录制配置

```yaml
# record_config.yaml
output_bags:
  - uri: /data/episodes/episode_{timestamp}
    storage_id: mcap           # 推荐：mcap 格式（比 sqlite3 快，支持索引）
    compression_mode: message
    compression_format: zstd   # zstd 压缩（速度快，压缩比好）

topics:
  # 感知（关键：多视角图像）
  - name: /camera_wrist/image_rect_color
    storage_options:
      topic_qos_profile_overrides:
        reliability: best_effort
  - name: /camera_head/image_rect_color
  - name: /depth/image_rect_raw

  # 机器人状态
  - name: /joint_states                   # 关节位置/速度/力矩
  - name: /joint_trajectory_controller/state  # 控制器跟踪误差
  - name: /tf                             # 坐标系变换
  - name: /tf_static

  # LLM / VLA 元数据（用于训练和调试）
  - name: /rai/tool_call_log              # LLM 工具调用序列
  - name: /rai/task_status               # 任务状态（成功/失败）
  - name: /vla/language_instruction      # 语言指令

  # 底盘（若有）
  - name: /odom
  - name: /cmd_vel
```

### 2.2 Episode 元数据

每次任务开始时，发布一条 episode 元数据消息：

```python
# 发布 episode 开始标记（带语言标注）
episode_meta = EpisodeMeta()
episode_meta.header.stamp = get_clock().now().to_msg()
episode_meta.task_description = "pick up the red cup and place it on shelf A"
episode_meta.operator = "human_teleop"  # 或 "vla_policy" / "llm_agent"
episode_meta.robot_id = "robot_001"
episode_meta.episode_id = str(uuid.uuid4())
episode_meta_pub.publish(episode_meta)
```

---

## 三、数据清洗流水线

### 3.1 清洗步骤

```python
# 数据清洗脚本（离线处理）
import rosbag2_py
import numpy as np

class EpisodeFilter:
    def process_bag(self, bag_path: str) -> dict:
        """处理单个 episode bag，返回是否保留及原因"""
        reader = rosbag2_py.SequentialReader()
        reader.open(bag_path)
        
        # Step 1: 提取 episode 结果（成功/失败）
        task_success = self._extract_task_status(reader)
        if not task_success:
            return {"keep": False, "reason": "task_failed"}
        
        # Step 2: 检查数据完整性（帧率、时间戳连续性）
        fps = self._check_image_fps(reader, '/camera_wrist/image_rect_color')
        if fps < 20.0:  # 要求至少 20 Hz
            return {"keep": False, "reason": f"low_fps: {fps:.1f}Hz"}
        
        # Step 3: 检查关节数据完整性
        joint_gaps = self._find_joint_state_gaps(reader)
        if joint_gaps > 0:
            return {"keep": False, "reason": f"joint_gaps: {joint_gaps}"}
        
        # Step 4: 检查动作质量（过于抖动的动作排除）
        action_smoothness = self._compute_action_smoothness(reader)
        if action_smoothness < 0.5:  # 自定义平滑度指标
            return {"keep": False, "reason": "jerky_motion"}
        
        return {"keep": True, "duration": self._get_duration(reader)}
```

### 3.2 时间对齐（图像 + 动作同步）

VLA 训练数据要求图像与动作的时间戳严格对齐：

```python
def align_observations_and_actions(bag_path: str, dt: float = 0.05) -> list:
    """
    将图像（可能 30Hz）和关节状态（可能 1000Hz）对齐到统一时间步。
    dt: 目标时间步（s），如 0.05s = 20Hz
    """
    images = load_topic(bag_path, '/camera_wrist/image_rect_color')  # [(stamp, img)]
    joints = load_topic(bag_path, '/joint_states')                    # [(stamp, JointState)]
    actions = load_topic(bag_path, '/joint_trajectory_controller/command')

    aligned_steps = []
    for t in np.arange(start_time, end_time, dt):
        step = {
            # 观测：最近的图像（不超过 dt/2 的时间差）
            "observation": {
                "image": find_nearest(images, t, max_dt=dt/2),
                "joint_pos": find_nearest(joints, t, max_dt=dt/2).position,
                "joint_vel": find_nearest(joints, t, max_dt=dt/2).velocity,
            },
            # 动作：t+dt 时刻的关节命令（预测下一步动作）
            "action": {
                "joint_pos": find_nearest(actions, t + dt, max_dt=dt/2),
            },
            "timestamp": t,
        }
        if all(v is not None for v in step["observation"].values()):
            aligned_steps.append(step)
    
    return aligned_steps
```

---

## 四、训练数据格式

### 4.1 LeRobot 格式（推荐）

[LeRobot](https://github.com/huggingface/lerobot) 是 Hugging Face 的机器人学习框架，定义了标准的数据集格式：

```
dataset/
├── meta/
│   ├── info.json              ← 数据集元信息（robot_type, fps, features）
│   ├── stats.json             ← 归一化统计（各特征的 mean/std/min/max）
│   └── episodes.jsonl         ← episode 列表（长度、任务描述）
└── data/
    ├── chunk-000/
    │   ├── episode_000000.parquet   ← 每个 episode 一个 Parquet 文件
    │   └── ...
    └── videos/
        ├── observation.images.wrist_camera/
        │   ├── episode_000000.mp4  ← 压缩视频（节省存储）
        │   └── ...
        └── observation.images.head_camera/
```

```json
// meta/info.json 示例
{
  "robot_type": "my_robot_7dof",
  "fps": 20,
  "features": {
    "observation.images.wrist_camera": {"dtype": "video", "shape": [3, 224, 224]},
    "observation.state": {"dtype": "float32", "shape": [14], "names": ["q1", "q2", ..., "v1", ...]},
    "action": {"dtype": "float32", "shape": [7], "names": ["q1_target", ..., "gripper"]},
    "task_index": {"dtype": "int64"},
    "timestamp": {"dtype": "float32"}
  }
}
```

### 4.2 rosbag2 → LeRobot 转换

```python
# convert_bag_to_lerobot.py
from lerobot.common.datasets.push_dataset_to_hub import push_dataset_to_hub

episodes = []
for bag_path in glob.glob('/data/episodes/*.mcap'):
    filter_result = EpisodeFilter().process_bag(bag_path)
    if not filter_result["keep"]:
        continue
    steps = align_observations_and_actions(bag_path)
    episodes.append({
        "steps": steps,
        "task": extract_task_description(bag_path),
        "success": True,
    })

# 上传到 Hugging Face Hub（或保存本地）
push_dataset_to_hub(
    episodes,
    repo_id="my_org/my_robot_pick_place_v1",
    fps=20,
    video_backend="ffmpeg",
    private=True
)
```

---

## 五、模型回放验证

### 5.1 验证流程

```
① 录制的 "ground truth" episode（人工遥操）
       ↓ 只回放观测（图像 + 初始状态）
② VLA 策略输出动作（推理模式）
       ↓ 与 ground truth 动作对比
③ 评估指标：
   - 动作 MSE（均方误差）
   - 轨迹 DTW（动态时间规整距离）
   - 任务成功率（在真实机器人或仿真器上验证）
```

### 5.2 sim2real 验证（Isaac Sim + ROS 2）

```python
# 在 Isaac Sim 中回放并评估 VLA 策略
class SimEvaluator:
    def evaluate_policy(self, episode_bag: str, policy: VlaPolicy) -> dict:
        # 在 Isaac Sim 中重置场景（从 bag 中的初始状态）
        self.sim.reset_scene(load_state_from_bag(episode_bag, t=0))
        
        success_count = 0
        for trial in range(10):  # 10 次重复评估（随机初始化扰动）
            self.sim.reset_with_noise(noise_std=0.01)
            
            for step in range(max_steps):
                # 获取当前图像（从 Isaac Sim）
                obs_image = self.sim.get_camera_image("wrist_camera")
                language_cond = self.language_instruction
                
                # VLA 推理
                action = policy.infer(obs_image, language_cond)
                
                # 执行动作（Isaac Sim）
                self.sim.step(action)
                
                # 检查成功条件
                if self.sim.check_success():
                    success_count += 1
                    break
        
        return {
            "success_rate": success_count / 10,
            "mean_steps_to_success": ...,
        }
```

---

## 六、数据飞轮的规模化建议

| 阶段 | 数据量 | 主要工具 | 关键指标 |
|------|--------|---------|---------|
| MVP（概念验证）| 50-200 episodes | rosbag2 + 人工标注 | 成功率 > 50% |
| 早期产品 | 1k-5k episodes | LeRobot + 自动过滤 | 成功率 > 80% |
| 大规模 | 10k+ episodes | 分布式采集 + RLDS | 泛化到新场景 |
| SOTA | 100k+ episodes | 多机器人 + 数据增强 | zero-shot 泛化 |

**数据增强建议**（降低数据采集量）：
- 图像增强（颜色抖动、随机裁剪、随机光照）
- 关节状态噪声注入（提升 sim2real 鲁棒性）
- 时间翻转（可逆动作，如放置任务可以逆向生成"拿起"数据）
- 跨具身（cross-embodiment）迁移：使用其他机器人的大规模数据预训练
