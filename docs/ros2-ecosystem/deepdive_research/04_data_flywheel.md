# 完整数据飞轮架构深化

> 综合 Phase 3（lerobot/VLA）、Phase 6（Isaac Sim 数据采集）、Phase 9（omni.replicator）的研究成果，构建完整的具身智能数据飞轮架构，覆盖在线数据筛选、持续学习防遗忘和 DVC+MLflow 版本管理。

---

## 一、数据飞轮整体架构

```
完整数据飞轮（具身智能 Data Flywheel）：

                    ┌──────────────────────────┐
                    │     真实世界部署（Robot）  │
                    │   执行策略 → 收集 rosbag2 │
                    └────────────┬─────────────┘
                                 │ 原始数据
                                 ▼
                    ┌──────────────────────────┐
                    │     在线数据筛选器         │
                    │  DataQualityFilter Node  │
                    │  ├── 失败回合自动标记       │
                    │  ├── 多样性去重（SimHash）  │
                    │  └── 人工标注接口（可选）   │
                    └────────────┬─────────────┘
                                 │ 高质量样本
                      ┌──────────┴──────────┐
                      ▼                     ▼
         ┌────────────────────┐  ┌──────────────────────┐
         │  真实数据 Dataset   │  │  仿真补充数据          │
         │  (LeRobot format)  │  │  Isaac Lab / Gazebo  │
         │  DVC 版本管理       │  │  Domain Randomization │
         └─────────┬──────────┘  └──────────┬───────────┘
                   │                         │
                   └────────────┬────────────┘
                                │ Sim+Real Mixed Dataset
                                ▼
                   ┌────────────────────────────┐
                   │     Continual Learning      │
                   │  EWC / PackNet / Replay     │
                   │  防灾难性遗忘机制            │
                   └────────────┬───────────────┘
                                │ 更新后模型权重
                                ▼
                   ┌────────────────────────────┐
                   │   MLflow Experiment Track  │
                   │  版本对比 / A-B 测试         │
                   └────────────┬───────────────┘
                                │ 通过验证的新版本
                                ▼
                   ┌────────────────────────────┐
                   │      OTA 部署（Phase 11）   │
                   │  cosign 签名 + 蓝绿发布      │
                   └────────────┬───────────────┘
                                │ 新策略上线
                                ▼
                    ┌──────────────────────────┐
                    │   (回到真实世界部署)       │
                    └──────────────────────────┘
```

---

## 二、在线数据筛选器

### 2.1 回合质量评估器

```python
# data_flywheel/episode_quality_filter.py
import numpy as np
from dataclasses import dataclass
from typing import Optional
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped

@dataclass
class EpisodeMetrics:
    success: bool
    completion_time: float    # 秒
    mean_force: float         # 平均力矩（N）
    path_smoothness: float    # 轨迹平滑度（越小越好）
    novel_score: float        # 多样性分数（相对已有数据集）

class EpisodeQualityFilter(Node):
    """
    在线评估每个 episode 质量，决定是否加入训练集。
    """
    def __init__(self):
        super().__init__('episode_quality_filter')
        
        # 可配置阈值
        self.declare_parameter('min_success_rate', 0.0)
        self.declare_parameter('max_completion_time', 30.0)   # 秒
        self.declare_parameter('novelty_threshold', 0.15)     # SimHash 最小距离
        
        self._joint_buffer = []
        self._force_buffer = []
        
        self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 10)
        self.create_subscription(
            WrenchStamped, '/ft_sensor', self._force_cb, 10)
    
    def evaluate_episode(
        self, episode_id: str, success: bool
    ) -> tuple[bool, EpisodeMetrics]:
        metrics = EpisodeMetrics(
            success=success,
            completion_time=self._compute_time(),
            mean_force=np.mean([f['norm'] for f in self._force_buffer]),
            path_smoothness=self._compute_smoothness(),
            novel_score=self._compute_novelty(episode_id),
        )
        
        # 筛选规则
        keep = (
            metrics.success and
            metrics.completion_time <= self.get_parameter(
                'max_completion_time').value and
            metrics.novel_score >= self.get_parameter(
                'novelty_threshold').value
        )
        
        if keep:
            self.get_logger().info(
                f'Episode {episode_id}: KEEP '
                f'(t={metrics.completion_time:.1f}s, '
                f'novel={metrics.novel_score:.2f})')
        else:
            self.get_logger().info(
                f'Episode {episode_id}: DISCARD '
                f'(success={success}, '
                f't={metrics.completion_time:.1f}s)')
        
        return keep, metrics
    
    def _compute_smoothness(self) -> float:
        """轨迹平滑度：关节加速度的 L2 范数"""
        if len(self._joint_buffer) < 3:
            return 0.0
        positions = np.array([j['pos'] for j in self._joint_buffer])
        # 二阶差分近似加速度
        accel = np.diff(positions, n=2, axis=0)
        return float(np.mean(np.linalg.norm(accel, axis=1)))
    
    def _compute_novelty(self, episode_id: str) -> float:
        """使用 SimHash 计算与已有数据集的最小 Hamming 距离"""
        # 实际实现需对接 lerobot dataset index
        # 此处返回模拟值
        return np.random.uniform(0.1, 0.9)
    
    def _compute_time(self) -> float:
        if len(self._joint_buffer) < 2:
            return 0.0
        return self._joint_buffer[-1]['stamp'] - self._joint_buffer[0]['stamp']
    
    def _joint_cb(self, msg):
        self._joint_buffer.append({
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'pos': list(msg.position),
        })
    
    def _force_cb(self, msg):
        f = msg.wrench.force
        self._force_buffer.append({
            'norm': np.sqrt(f.x**2 + f.y**2 + f.z**2)
        })
```

### 2.2 数据集多样性去重

```python
# data_flywheel/diversity_dedup.py
import hashlib
import numpy as np
from typing import List

class SimHashDeduplicator:
    """
    使用 SimHash 对 episode 状态序列去重，
    避免重复轨迹污染训练集。
    """
    
    def __init__(self, hash_bits: int = 64, threshold: int = 8):
        self.hash_bits = hash_bits
        self.threshold = threshold   # Hamming 距离阈值
        self._fingerprints: List[int] = []
    
    def compute_fingerprint(self, state_sequence: np.ndarray) -> int:
        """将状态序列哈希为 64-bit 指纹"""
        # 下采样到 32 个关键帧
        indices = np.linspace(0, len(state_sequence)-1, 32, dtype=int)
        keyframes = state_sequence[indices].flatten()
        
        # 量化并计算 SimHash
        bits = np.zeros(self.hash_bits)
        for i, val in enumerate(keyframes):
            quantized = int(val * 100)   # 量化到厘米精度
            h = int(hashlib.md5(
                f"{i}:{quantized}".encode()).hexdigest(), 16)
            for bit in range(self.hash_bits):
                bits[bit] += 1 if (h >> bit) & 1 else -1
        
        fingerprint = 0
        for bit in range(self.hash_bits):
            if bits[bit] > 0:
                fingerprint |= (1 << bit)
        return fingerprint
    
    def hamming_distance(self, a: int, b: int) -> int:
        return bin(a ^ b).count('1')
    
    def is_duplicate(self, fingerprint: int) -> bool:
        for fp in self._fingerprints:
            if self.hamming_distance(fp, fingerprint) <= self.threshold:
                return True
        return False
    
    def add(self, fingerprint: int):
        if not self.is_duplicate(fingerprint):
            self._fingerprints.append(fingerprint)
            return True  # 新样本，已添加
        return False  # 重复，跳过
```

---

## 三、持续学习（防灾难性遗忘）

### 3.1 三种方法比较

| 方法 | 原理 | 内存开销 | 实现复杂度 | 适用场景 |
|------|------|---------|---------|---------|
| **EWC**（Elastic Weight Consolidation） | Fisher 信息矩阵加权正则 | 2× 模型参数 | 中 | 任务序列不长，旧任务数据可重新计算 |
| **PackNet** | 参数剪枝 + 子网络 | 1× | 高 | 需要严格保留旧任务精度 |
| **Experience Replay** | 保留旧任务数据子集 | 数据集相关 | 低 | 存储充足，最实用 |

### 3.2 EWC 实现

```python
# continual_learning/ewc.py
import torch
import torch.nn as nn
from copy import deepcopy

class EWC:
    """
    Elastic Weight Consolidation — 在学习新任务时，
    通过惩罚改变重要参数来保护旧任务知识。
    """
    
    def __init__(self, model: nn.Module, dataloader, device: str = 'cuda'):
        self.model = model
        self.device = device
        self.params = {n: p.clone() for n, p in model.named_parameters()
                       if p.requires_grad}
        self.fisher = self._compute_fisher(dataloader)
    
    def _compute_fisher(self, dataloader) -> dict:
        """计算 Fisher 信息矩阵（对角线近似）"""
        fisher = {n: torch.zeros_like(p)
                  for n, p in self.model.named_parameters()
                  if p.requires_grad}
        
        self.model.eval()
        for obs, action in dataloader:
            obs, action = obs.to(self.device), action.to(self.device)
            self.model.zero_grad()
            output = self.model(obs)
            loss = nn.functional.mse_loss(output, action)
            loss.backward()
            
            for n, p in self.model.named_parameters():
                if p.requires_grad and p.grad is not None:
                    fisher[n] += p.grad.detach() ** 2
        
        # 归一化
        n_samples = len(dataloader.dataset)
        return {n: f / n_samples for n, f in fisher.items()}
    
    def penalty(self) -> torch.Tensor:
        """EWC 正则项：重要参数的变化惩罚"""
        loss = torch.tensor(0.0, device=self.device)
        for n, p in self.model.named_parameters():
            if p.requires_grad and n in self.fisher:
                loss += (self.fisher[n] * (p - self.params[n]) ** 2).sum()
        return loss

# 训练循环（新任务）
def train_with_ewc(model, new_dataloader, ewc: EWC,
                   ewc_lambda: float = 400.0):
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-4)
    
    for obs, action in new_dataloader:
        optimizer.zero_grad()
        output = model(obs)
        task_loss = nn.functional.mse_loss(output, action)
        ewc_loss = ewc_lambda * ewc.penalty()  # EWC 正则
        total_loss = task_loss + ewc_loss
        total_loss.backward()
        optimizer.step()
```

### 3.3 Experience Replay 实现

```python
# continual_learning/replay_buffer.py
import random
from collections import deque
from typing import Tuple
import numpy as np

class ReplayBuffer:
    """
    保留旧任务数据的 Ring Buffer，
    新任务训练时混入旧样本防止遗忘。
    """
    
    def __init__(self, capacity: int = 10000):
        self.buffer = deque(maxlen=capacity)
    
    def add_episode(self, episode: dict):
        """episode: {'obs': np.ndarray, 'action': np.ndarray}"""
        # 随机采样 episode 中的帧
        n_frames = len(episode['obs'])
        sample_indices = random.sample(
            range(n_frames),
            min(50, n_frames)  # 每个 episode 最多保留 50 帧
        )
        for idx in sample_indices:
            self.buffer.append({
                'obs': episode['obs'][idx],
                'action': episode['action'][idx],
            })
    
    def sample(self, batch_size: int) -> Tuple[np.ndarray, np.ndarray]:
        samples = random.sample(self.buffer, min(batch_size, len(self.buffer)))
        obs = np.stack([s['obs'] for s in samples])
        actions = np.stack([s['action'] for s in samples])
        return obs, actions
    
    def __len__(self) -> int:
        return len(self.buffer)
```

---

## 四、DVC + MLflow 版本管理

### 4.1 数据版本管理（DVC）

```bash
# 初始化 DVC
dvc init
dvc remote add -d storage s3://robot-data-bucket/dvc

# 追踪大型数据集（不存入 git）
dvc add data/real_world_episodes/
dvc add data/sim_episodes/

# 创建数据处理流水线
# dvc.yaml
stages:
  filter:
    cmd: python data_flywheel/episode_quality_filter.py
         --input data/raw_episodes/
         --output data/filtered_episodes/
    deps:
      - data/raw_episodes/
      - data_flywheel/episode_quality_filter.py
    outs:
      - data/filtered_episodes/
    
  mix_sim_real:
    cmd: python data_flywheel/mix_datasets.py
         --real data/filtered_episodes/
         --sim data/sim_episodes/
         --ratio 0.3
         --output data/mixed_dataset/
    deps:
      - data/filtered_episodes/
      - data/sim_episodes/
    outs:
      - data/mixed_dataset/
    params:
      - params.yaml:
          - sim_real_ratio

# 运行流水线
dvc repro
dvc push  # 上传到远端存储
```

### 4.2 实验追踪（MLflow）

```python
# training/train_vla.py
import mlflow
import mlflow.pytorch

def train(config: dict):
    mlflow.set_experiment("vla_policy_training")
    
    with mlflow.start_run(run_name=f"vla_{config['data_version']}"):
        # 记录超参数
        mlflow.log_params({
            'model': config['model_name'],
            'data_version': config['data_version'],
            'sim_real_ratio': config['sim_real_ratio'],
            'ewc_lambda': config['ewc_lambda'],
            'batch_size': config['batch_size'],
            'lr': config['lr'],
        })
        
        # 记录数据集版本（DVC hash）
        import subprocess
        dvc_hash = subprocess.check_output(
            ['dvc', 'status', '--json']).decode()
        mlflow.log_param('dvc_dataset_hash', dvc_hash[:16])
        
        # 训练循环
        for epoch in range(config['epochs']):
            train_loss = train_epoch(...)
            val_metrics = evaluate(...)
            
            mlflow.log_metrics({
                'train_loss': train_loss,
                'val_success_rate': val_metrics['success_rate'],
                'val_pick_accuracy': val_metrics['pick_accuracy'],
            }, step=epoch)
        
        # 模型注册
        mlflow.pytorch.log_model(
            model,
            artifact_path="policy_model",
            registered_model_name="VLAPickPolicy")
        
        # 标注模型版本（通过 registry）
        client = mlflow.tracking.MlflowClient()
        client.transition_model_version_stage(
            name="VLAPickPolicy",
            version=...,
            stage="Staging"  # Staging → Production 需人工审批
        )
```

### 4.3 完整 CI/CD 数据飞轮脚本

```bash
#!/bin/bash
# scripts/data_flywheel_ci.sh — GitHub Actions 中调用

set -euo pipefail

DATA_VERSION=${1:-"latest"}
MLFLOW_URI="http://mlflow.internal:5000"

echo "=== Step 1: 拉取最新 DVC 数据 ==="
dvc pull data/raw_episodes/

echo "=== Step 2: 数据质量过滤 ==="
dvc repro filter

echo "=== Step 3: Sim+Real 混合 ==="
dvc repro mix_sim_real

echo "=== Step 4: 持续学习训练（增量微调） ==="
python training/train_vla.py \
  --data data/mixed_dataset/ \
  --base-model models/current_production/ \
  --ewc-lambda 400 \
  --mlflow-uri "$MLFLOW_URI"

echo "=== Step 5: 自动评估 ==="
python evaluation/eval_in_sim.py \
  --model models/candidate/ \
  --tasks pick_place cube_stack \
  --n-episodes 100

echo "=== Step 6: 推送验证结果 ==="
dvc push
echo "=== 数据飞轮本次迭代完成 ==="
```

---

## 五、飞轮运营指标

| 指标 | 目标值 | 监控工具 |
|------|-------|---------|
| 每周新增高质量 episode | > 500 | DVC + Grafana |
| 筛选保留率 | 30-70%（过高说明没挑战性） | MLflow |
| 新版本 vs 旧版本成功率提升 | > 2% per cycle | MLflow A/B |
| 旧任务遗忘率（EWC 后） | < 3% | MLflow |
| OTA 部署成功率 | > 99% | Mender Dashboard |
| 完整飞轮周期时间 | < 48h | GitHub Actions |
