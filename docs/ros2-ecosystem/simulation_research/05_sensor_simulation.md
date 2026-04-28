# 传感器仿真精度与配置

> 本文档深入分析具身智能仿真中各类传感器的仿真精度、噪声模型及 ROS 2 集成方式，重点对比 Isaac Sim RTX 与 Gazebo Harmonic 两大平台的差异。

---

## 一、RGB 相机

### 1.1 渲染引擎对比

| 对比维度 | Isaac Sim RTX 光线追踪 | Gazebo Harmonic 栅格渲染 |
|---------|----------------------|------------------------|
| **渲染管线** | NVIDIA RTX（Path Tracing + Rasterization 混合） | Ogre2（PBR 栅格化） |
| **真实度（视觉质量）** | ⭐⭐⭐⭐⭐ 全局光照、焦散、反射 | ⭐⭐⭐ Blinn-Phong / PBR，无 GI |
| **GPU 计算开销** | 高（RTX 3080+，30–120ms/帧） | 低（GTX 1060+ 即可，2–5ms/帧） |
| **ROS 2 集成难度** | 中（OmniGraph + isaac_ros2_bridge） | 低（ros_gz_bridge 开箱即用） |
| **domain randomization** | ✅ omni.replicator 支持材质/光照/纹理 DR | ⚠️ 需手动脚本替换 SDF 材质 |
| **相机帧率上限** | 30–60 fps（RTX 模式）；Rasterization 模式可达 120 fps | 60–120 fps（取决于场景复杂度） |
| **Sim-to-Real 质量（FID 分数）** | FID ≈ 15–30（接近真实） | FID ≈ 40–80（风格差距明显） |

> **选型建议**：若目标任务依赖视觉泛化（VLA/ViT 模型），优先 Isaac Sim RTX；若仅做功能测试/控制逻辑验证，Gazebo Harmonic 更轻量。

### 1.2 `image_transport` 桥接

```yaml
# ros_gz_bridge 配置：Gazebo 相机话题 → ROS 2
- gz_topic_name: /camera/image
  ros_topic_name: /camera/color/image_raw
  gz_type_name: gz.msgs.Image
  ros_type_name: sensor_msgs/msg/Image
  direction: GZ_TO_ROS
- gz_topic_name: /camera/camera_info
  ros_topic_name: /camera/color/camera_info
  gz_type_name: gz.msgs.CameraInfo
  ros_type_name: sensor_msgs/msg/CameraInfo
  direction: GZ_TO_ROS
```

```python
# Isaac Sim OmniGraph 节点配置（伪代码）
import omni.graph.core as og

# 创建 ROS2CameraHelper 节点
graph = og.Controller.create_graph("/World/ActionGraph")
camera_helper = graph.create_node(
    "ROS2CameraHelper",
    "omni.isaac.ros2_bridge.ROS2CameraHelper"
)
og.Controller.set(camera_helper.get_attribute("topicName"), "/camera/color/image_raw")
og.Controller.set(camera_helper.get_attribute("frameId"), "camera_color_optical_frame")
og.Controller.set(camera_helper.get_attribute("renderProductPath"), "/World/Camera/RenderProduct")
```

### 1.3 CameraInfo 内参/畸变传递

仿真相机需精确配置内参，使 `camera_calibration` 标定工具可复用：

```xml
<!-- Gazebo SDF camera 配置（含畸变参数） -->
<sensor name="rgb_camera" type="camera">
  <camera>
    <horizontal_fov>1.3962634</horizontal_fov>  <!-- 80° -->
    <image>
      <width>1280</width>
      <height>720</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
    <distortion>
      <k1>0.0</k1>  <!-- 仿真中通常设为0（无畸变），可手动注入 -->
      <k2>0.0</k2>
      <k3>0.0</k3>
      <p1>0.0</p1>
      <p2>0.0</p2>
      <center>0.5 0.5</center>
    </distortion>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>  <!-- 模拟 CMOS 读取噪声 -->
    </noise>
    <lens>
      <type>stereographic</type>
    </lens>
  </camera>
  <plugin filename="gz-sim-ros2-sensors-system"
          name="gz::sim::systems::Sensors">
    <render_engine>ogre2</render_engine>
  </plugin>
</sensor>
```

---

## 二、深度相机（RGB-D）

### 2.1 噪声模型对比

| 噪声类型 | 适用传感器 | 物理原理 | 参数 |
|---------|-----------|---------|------|
| **高斯深度噪声** | 通用/ToF（飞行时间） | 光子计数误差 | `σ = a + b·d²`（d 为深度） |
| **结构光遮挡噪声** | Intel RealSense D435 等 | 投影光栅在遮挡边缘产生混叠 | 边缘宽度 2–5 像素 |
| **多径干扰（MPI）** | ToF 传感器 | 间接反射造成深度偏低估计 | 依赖场景反射率 |
| **散焦模糊** | 结构光 | 远距离散焦 | `σ(d) ∝ d^1.5 `（d > 1.5m 时显著） |

```xml
<!-- Gazebo Harmonic depth_camera SDF -->
<sensor name="depth_camera" type="depth_camera">
  <camera>
    <horizontal_fov>1.5184</horizontal_fov>  <!-- 87° RealSense D435 -->
    <image>
      <width>848</width>
      <height>480</height>
      <format>R_FLOAT32</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>5.0</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>  <!-- 1cm @ 1m，随距离二次增长 -->
    </noise>
  </camera>
  <update_rate>30</update_rate>
</sensor>
```

### 2.2 点云处理管线

```
/camera/depth/image_raw (16UC1, mm)
    │
    ▼
depth_image_proc::PointCloudXyzrgbNode
    │  参数：
    │  - queue_size: 5
    │  - use_exact_sync: false（ApproximateSync）
    │
    ▼
/camera/depth/points (sensor_msgs/PointCloud2)
    │
    ▼ (GPU 加速路径)
isaac_ros_depth_image_proc::DepthToPointCloudNode
    │  NITROS 零拷贝，延迟 < 2ms（vs CPU 路径 ~15ms）
    ▼
/camera/depth/points (PointCloud2, GPU→CPU 可选)
```

**isaac_ros_depth_image_proc GPU 加速配置：**

```python
# launch/depth_proc.launch.py
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

container = ComposableNodeContainer(
    name='depth_proc_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container_mt',
    composable_node_descriptions=[
        ComposableNode(
            package='isaac_ros_depth_image_proc',
            plugin='nvidia::isaac_ros::depth_image_proc::DepthToPointCloudNode',
            name='depth_to_pointcloud',
            parameters=[{
                'use_color': True,
                'skip_pixels_x': 1,
                'skip_pixels_y': 1,
            }],
            remappings=[
                ('depth/image_rect_raw', '/camera/depth/image_raw'),
                ('rgb/image_rect_color', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('points', '/camera/depth/points'),
            ]
        ),
    ],
)
```

---

## 三、LiDAR

### 3.1 Isaac Sim RTX Lidar vs Gazebo gpu_lidar

| 对比维度 | Isaac Sim RTX Lidar | Gazebo `gpu_lidar` |
|---------|--------------------|--------------------|
| **光线模型** | 物理光线追踪（折射/反射/散射） | 单次光线投射（无二次反射） |
| **多回波** | ✅ 支持（最多 N 次回波） | ❌ 单回波 |
| **强度通道** | ✅ 基于材质 BRDF 计算 | ⚠️ 固定强度或材质属性 |
| **雨雾仿真** | ✅ 粒子参与介质 | ❌ 不支持 |
| **线数支持** | 任意（常用 16/32/64/128 线） | 任意（栅格扫描） |
| **CPU 占用** | 低（GPU 计算） | 低（GPU 计算） |
| **ROS 2 消息格式** | `sensor_msgs/PointCloud2` 或 `LaserScan` | 同左 |

### 3.2 ros_gz_bridge LiDAR 话题映射

```yaml
# 64线 Ouster OS1 LiDAR 桥接配置
bridges:
  - gz_topic_name: /lidar/points
    ros_topic_name: /lidar/points
    gz_type_name: gz.msgs.PointCloudPacked
    ros_type_name: sensor_msgs/msg/PointCloud2
    direction: GZ_TO_ROS

  - gz_topic_name: /lidar/scan
    ros_topic_name: /scan
    gz_type_name: gz.msgs.LaserScan
    ros_type_name: sensor_msgs/msg/LaserScan
    direction: GZ_TO_ROS
```

### 3.3 Isaac Sim RTX Lidar 配置示例

```python
# 创建 RTX Lidar 传感器（Ouster OS1-64 Profile）
from omni.isaac.sensor import LidarRtx

lidar = LidarRtx(
    prim_path="/World/Robot/lidar_link/LidarRtx",
    name="os1_64",
    position=(0.0, 0.0, 0.2),
    orientation=(1.0, 0.0, 0.0, 0.0),
    config_file_name="OS1_64ch20hz512res"  # 内置 Profile 文件
)

# 通过 OmniGraph 发布 ROS 2 话题
og.Controller.set(
    og.Controller.node("/World/ActionGraph/ROS2LidarHelper")
        .get_attribute("topicName"), "/lidar/points"
)
```

---

## 四、IMU

### 4.1 Allan Variance 噪声参数

IMU 噪声由以下参数完整描述（符合 `robot_localization` / `imu_filter_madgwick` 接口要求）：

| 参数 | 符号 | 典型值（MEMS IMU） | 含义 |
|-----|------|-----------------|------|
| **角速度随机游走** | `σ_ω` | 0.005 rad/s/√Hz | 高斯白噪声密度 |
| **角速度偏置稳定性** | `β_ω` | 0.0001 rad/s | 慢变偏置（时间常数 ~100s） |
| **加速度随机游走** | `σ_a` | 0.01 m/s²/√Hz | 高斯白噪声密度 |
| **加速度偏置稳定性** | `β_a` | 0.001 m/s² | 慢变偏置 |

```xml
<!-- Gazebo Harmonic IMU SDF 噪声配置 -->
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>400</update_rate>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.005</stddev>      <!-- σ_ω -->
          <bias_mean>0.0001</bias_mean>
          <bias_stddev>0.00001</bias_stddev>
          <dynamic_bias_stddev>0.000005</dynamic_bias_stddev>
          <dynamic_bias_correlation_time>100.0</dynamic_bias_correlation_time>
        </noise>
      </x>
      <!-- y, z 同上 -->
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.01</stddev>       <!-- σ_a -->
          <bias_mean>0.001</bias_mean>
          <bias_stddev>0.0001</bias_stddev>
          <dynamic_bias_stddev>0.00005</dynamic_bias_stddev>
          <dynamic_bias_correlation_time>300.0</dynamic_bias_correlation_time>
        </noise>
      </x>
      <!-- y, z 同上 -->
    </linear_acceleration>
  </imu>
</sensor>
```

### 4.2 robot_localization EKF 接入仿真 IMU

```yaml
# ekf.yaml — 仿真 IMU + 里程计融合
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    two_d_mode: false
    use_sim_time: true          # 关键：仿真时间对齐

    odom0: /odom
    odom0_config: [true, true, false,  # x, y, z
                   false, false, true,  # roll, pitch, yaw
                   false, false, false, # vx, vy, vz
                   false, false, true,  # vroll, vpitch, vyaw
                   false, false, false] # ax, ay, az

    imu0: /imu/data
    imu0_config: [false, false, false,  # pose
                  true, true, true,     # orientation (roll/pitch/yaw)
                  false, false, false,  # velocity
                  false, false, true,   # angular velocity (yaw only for 2D)
                  true, true, false]    # linear acceleration (x/y)
    imu0_differential: false
    imu0_remove_gravitational_acceleration: true
```

---

## 五、力矩传感器（FT Sensor）

### 5.1 Gazebo FT Sensor SDF

```xml
<!-- 腕部力矩传感器（6轴 F/T） -->
<sensor name="ft_sensor" type="force_torque">
  <always_on>true</always_on>
  <update_rate>1000</update_rate>  <!-- 1kHz -->
  <force_torque>
    <frame>child</frame>
    <measure_direction>child_to_parent</measure_direction>
    <force>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.5</stddev>   <!-- 0.5N 力分辨率 -->
        </noise>
      </x>
      <!-- y, z 同上 -->
    </force>
    <torque>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.05</stddev>  <!-- 0.05Nm 力矩分辨率 -->
        </noise>
      </x>
      <!-- y, z 同上 -->
    </torque>
  </force_torque>
</sensor>
```

### 5.2 `force_torque_sensor_broadcaster` 对接

```yaml
# ros2_control YAML：FT sensor broadcaster 配置
controller_manager:
  ros__parameters:
    ft_sensor_broadcaster:
      type: force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster

ft_sensor_broadcaster:
  ros__parameters:
    sensor_name: ft_sensor_wrist     # hardware_interface 中注册的传感器名
    state_interface_names:
      - force.x
      - force.y
      - force.z
      - torque.x
      - torque.y
      - torque.z
    frame_id: tool0_ft_frame
    topic_name: /ft_sensor/wrench   # 发布到 geometry_msgs/WrenchStamped
```

---

## 六、传感器同步

### 6.1 use_sim_time 与 header.stamp 对齐

仿真环境中所有传感器节点必须使用仿真时钟，否则时间戳不一致将导致 `tf2` 查询失败及数据对齐错误：

```python
# launch 文件中统一设置 use_sim_time
from launch.actions import SetParameter

SetParameter(name='use_sim_time', value=True)
```

```cpp
// C++ 节点中使用仿真时钟
auto node = rclcpp::Node::make_shared("sensor_proc",
    rclcpp::NodeOptions().use_intra_process_comms(true));
// 仅需在 launch 传参，节点内部 now() 自动使用仿真时间
auto stamp = node->now();
```

### 6.2 ApproximateTimeSynchronizer

```python
import message_filters
from sensor_msgs.msg import Image, CameraInfo, PointCloud2

# 深度+RGB 对齐（Approximate，容许 30ms 时间差）
rgb_sub = message_filters.Subscriber(node, Image, '/camera/color/image_raw')
depth_sub = message_filters.Subscriber(node, Image, '/camera/depth/image_raw')
cloud_sub = message_filters.Subscriber(node, PointCloud2, '/lidar/points')

sync = message_filters.ApproximateTimeSynchronizer(
    [rgb_sub, depth_sub, cloud_sub],
    queue_size=10,
    slop=0.03,         # 30ms 时间窗
    allow_headerless=False
)
sync.registerCallback(synchronized_callback)
```

**时间差来源分析：**

| 传感器 | 典型 stamp 偏差（仿真） | 原因 |
|-------|----------------------|------|
| RGB 相机 | ±2ms | 渲染帧时间 |
| 深度相机 | ±5ms | 深度计算 + 噪声采样 |
| LiDAR | ±1ms | 单步扫描时间 |
| IMU | ±0.1ms | 高频，几乎对齐 |

---

## 七、渲染性能优化

### 7.1 Headless 渲染模式

```bash
# Isaac Sim headless 启动（无 GUI，节省 30–40% GPU 内存）
./python.sh -c "
import omni.isaac.core as core
from omni.isaac.kit import SimulationApp

app = SimulationApp({'headless': True, 'anti_aliasing': 0})
# ... 传感器配置
while True:
    app.update()
"

# Gazebo headless 模式
gz sim -s -r my_world.sdf   # -s: server only; -r: run immediately
```

### 7.2 多 GPU 分配策略

```python
# Isaac Sim：渲染 GPU + 物理 GPU 分离
# DISPLAY_GPU（RTX）负责 RTX 渲染，PHYSICS_GPU 负责 PhysX 并行仿真
# 环境变量配置
import os
os.environ['CUDA_VISIBLE_DEVICES'] = '0,1'  # GPU 0: 渲染, GPU 1: Isaac Lab 训练

# Isaac Lab 并行训练时禁用高质量渲染
app_cfg = {
    'headless': True,
    'renderer': 'RasterizationRenderer',  # 非 RTX，训练时用栅格化
    'num_frames': -1,
}
```

### 7.3 RTX 渲染 vs 训练吞吐权衡

| 渲染模式 | 图像质量（FID） | 每秒生成帧数（单 RTX 4090） | 适用场景 |
|---------|--------------|--------------------------|---------|
| RTX Path Tracing | FID ~15 | 5–10 fps | 高质量演示数据生成 |
| RTX Real-time | FID ~25 | 20–40 fps | 平衡质量/速度 |
| Rasterization | FID ~60 | 200–500 fps | 大规模 RL 训练 |
| Minimal（无渲染） | N/A | 5000+ fps | 纯物理仿真/RL |

---

## 八、精度评估方法

### 8.1 RMSE 对比（深度相机）

```python
import numpy as np
from scipy.stats import norm

def evaluate_depth_accuracy(sim_depth: np.ndarray, real_depth: np.ndarray,
                             mask: np.ndarray = None) -> dict:
    """
    评估仿真深度图与真实深度图的差异。
    sim_depth, real_depth: (H, W) float32 depth maps in meters
    mask: valid pixel mask (optional)
    """
    if mask is None:
        mask = (real_depth > 0.1) & (real_depth < 5.0)

    err = sim_depth[mask] - real_depth[mask]
    rmse = np.sqrt(np.mean(err**2))
    mae = np.mean(np.abs(err))
    rel_err = np.mean(np.abs(err) / real_depth[mask])

    # NLL（负对数似然），假设高斯误差分布
    sigma_est = np.std(err)
    nll = -np.mean(norm.logpdf(err, loc=0, scale=sigma_est))

    return {
        'rmse_m': rmse,
        'mae_m': mae,
        'rel_err': rel_err,
        'nll': nll,
        'sigma_est_m': sigma_est,
    }
```

### 8.2 LiDAR 点云精度评估

```python
import open3d as o3d

def evaluate_lidar_accuracy(sim_pcd: o3d.geometry.PointCloud,
                             real_pcd: o3d.geometry.PointCloud) -> dict:
    """Chamfer Distance + F-Score @ 10cm"""
    dists_sim_to_real = np.asarray(sim_pcd.compute_point_cloud_distance(real_pcd))
    dists_real_to_sim = np.asarray(real_pcd.compute_point_cloud_distance(sim_pcd))

    chamfer = np.mean(dists_sim_to_real) + np.mean(dists_real_to_sim)

    threshold = 0.10  # 10cm
    precision = np.mean(dists_sim_to_real < threshold)
    recall = np.mean(dists_real_to_sim < threshold)
    f_score = 2 * precision * recall / (precision + recall + 1e-8)

    return {'chamfer_m': chamfer, 'f_score_10cm': f_score,
            'precision': precision, 'recall': recall}
```

### 8.3 IMU 精度评估（Allan Variance 对比）

真实 IMU 与仿真 IMU 的 Allan Variance 曲线对比：通过 `allan_variance_ros` 包从仿真录制的 `/imu/data` bag 文件中提取 Allan Variance 曲线，与数据手册值或真实测量值对比。

```bash
# 录制仿真 IMU 数据
ros2 bag record -o sim_imu_static /imu/data --use-sim-time

# 计算 Allan Variance
ros2 run allan_variance_ros allan_variance sim_imu_static/ \
  --imu-topic /imu/data --output sim_allan_variance.csv
```

---

## 九、传感器仿真精度总结

| 传感器 | Isaac Sim 精度 | Gazebo Harmonic 精度 | 主要精度瓶颈 |
|-------|--------------|---------------------|------------|
| RGB 相机 | ⭐⭐⭐⭐⭐（RTX 光追） | ⭐⭐⭐（栅格化） | 材质/光照差异 |
| 深度相机 | ⭐⭐⭐⭐（结构光建模） | ⭐⭐⭐（高斯噪声近似） | 遮挡/多径噪声建模 |
| LiDAR | ⭐⭐⭐⭐⭐（RTX 多回波） | ⭐⭐⭐⭐（GPU 栅格） | 反射率/天气效果 |
| IMU | ⭐⭐⭐（Allan Variance 可调） | ⭐⭐⭐（同等建模能力） | 偏置漂移时间常数 |
| FT 传感器 | ⭐⭐⭐⭐（PhysX 接触力） | ⭐⭐⭐（DART 接触） | 接触模型精度 |
