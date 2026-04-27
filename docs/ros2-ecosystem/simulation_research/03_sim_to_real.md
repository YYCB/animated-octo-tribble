# Sim-to-Real 迁移

## Sim-to-Real Gap 来源分析

| Gap 类型 | 具体表现 | 量化方法 |
|---------|---------|---------|
| 外观域差 (Appearance Gap) | 纹理/光照/阴影不真实 | FID (Fréchet Inception Distance) |
| 物理域差 (Dynamics Gap) | 摩擦/惯量/关节柔顺度偏差 | 开环轨迹跟踪误差 RMSE |
| 传感器噪声差 | 深度噪声、IMU 漂移不匹配 | Allan Variance 对比 |
| 延迟差 | 仿真无网络/处理延迟 | 闭环测试策略成功率下降率 |
| 控制器差 | PID 增益在实机与仿真不一致 | 阶跃响应对比 |

## Domain Randomization 策略

### 视觉随机化（Isaac Sim `omni.replicator`）

```python
import omni.replicator.core as rep

with rep.new_layer():
    # 光照随机化
    light = rep.create.light(light_type="Sphere")
    with light:
        rep.modify.pose(position=rep.distribution.uniform((-5,-5,3),(5,5,6)))
        rep.modify.attribute("intensity", rep.distribution.uniform(500, 2000))

    # 材质/纹理随机化
    objects = rep.get.prims(path_pattern="/World/Robot/.*")
    with objects:
        rep.randomizer.texture(
            textures=rep.utils.get_usd_files("/Isaac/Environments/Simple_Warehouse/textures")
        )

    # 相机噪声
    camera = rep.get.camera()
    with camera:
        rep.modify.attribute("focalLength", rep.distribution.uniform(17.0, 25.0))
```

### 动力学随机化

```python
# 关节摩擦随机化
from omni.isaac.core.articulations import Articulation

robot = Articulation("/World/Robot")
dof_props = robot.get_articulation_controller().get_dof_properties()
for i in range(robot.num_dof):
    dof_props["friction"][i] = np.random.uniform(0.01, 0.1)
    dof_props["damping"][i] *= np.random.uniform(0.8, 1.2)
robot.get_articulation_controller().set_dof_properties(dof_props)
```

## Sim-to-Real for VLA 完整流程

```
Isaac Sim (Domain Randomization)
    ↓ omni.replicator + ROS 2 Bridge
rosbag2 录制 (mcap格式)
    ↓ Phase 6 转换脚本
LeRobot HDF5 数据集
    ↓ VLA 训练 (π0 / RDT / OpenVLA)
Policy checkpoint
    ↓ TensorRT 量化
实机部署 (chassis_protocol + ros2_control)
    ↓ 评估
成功率反馈 → 更新仿真参数
```

## Real-to-Sim：数字孪生构建

### NeRF / 3DGS → USD 流程

1. **采集**：多角度 RGB 图像（≥50 张，均匀覆盖）
2. **重建**：`nerfstudio` 或 `gaussian-splatting` 训练
3. **导出**：NeRF → mesh → USD（`omni.kit.convert`）
4. **物理参数标定**：系统辨识（见下节）

### isaac_ros_nvblox 实时重建

```yaml
# nvblox_node 配置
nvblox_node:
  ros__parameters:
    voxel_size: 0.05          # 5cm 分辨率
    max_tsdf_update_hz: 10.0
    max_color_update_hz: 5.0
    map_clearing_radius_m: 5.0
```

## 系统辨识：关节摩擦/惯量

### 最小二乘辨识流程

```python
# 激励轨迹 → 采集力矩/位置/速度数据
# τ = M(q)q̈ + C(q,q̇)q̇ + g(q) + f(q̇)
# 线性化: τ = W(q, q̇, q̈) · π
# π = (WᵀW)⁻¹Wᵀτ  (最小二乘解)

import numpy as np
from scipy.optimize import least_squares

def residual(params, q, qd, qdd, tau_measured):
    # params: [inertia, friction_viscous, friction_coulomb, ...]
    tau_model = compute_torque(params, q, qd, qdd)
    return tau_measured - tau_model

result = least_squares(residual, x0=params_init, args=(q_data, qd_data, qdd_data, tau_data))
```

## 部署前验证 Checklist

| # | 验证项 | 通过标准 |
|---|-------|---------|
| 1 | 仿真开环轨迹重放 | RMSE < 5mm |
| 2 | 关节速度/加速度限制 | 无越限报警 |
| 3 | 碰撞检测覆盖 | FCL 模型与实机一致 |
| 4 | 传感器 topic 频率 | ±5% 偏差 |
| 5 | use_sim_time 切换 | 无时间戳异常 |
| 6 | E-Stop 响应时间 | < 50ms |
| 7 | 低速爬行稳定性 | 无震荡 |
| 8 | 策略在仿真成功率 | > 80% |
| 9 | 策略在实机成功率 | > 60% (首次) |
| 10 | 数据分布对比 | 关节角分布 KL < 0.2 |
