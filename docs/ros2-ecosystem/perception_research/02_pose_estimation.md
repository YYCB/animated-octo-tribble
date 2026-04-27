# 6D 姿态估计

> 覆盖 NVIDIA FoundationPose zero-shot 6D 姿态、DOPE 训练流程、AprilTag 快速标定，以及手眼标定和与 MoveIt 2 抓取的完整对接。

---

## 一、FoundationPose（NVIDIA）

### 1.1 架构概述

FoundationPose（Wen et al., CVPR 2024）是 NVIDIA 推出的**零样本 6D 姿态估计**框架，无需对目标物体进行逐类训练：

```
FoundationPose 推理管线：

输入：RGB-D 图像 + 物体 CAD 模型（仅推理时需要）
         │
         ▼
    实例分割（SegmentAnything / 用户提供 mask）
         │                  │
         ▼                  ▼
  点云提取（深度图）    渲染参考视图（CAD模型，多姿态）
         │                  │
         └──────┬───────────┘
                ▼
     Transformer 特征匹配
     （几何特征 + 外观特征融合）
                │
                ▼
      6D 姿态假设生成（多假设池）
                │
                ▼
      姿态精细化（Refinement，Render+Compare 迭代）
                │
                ▼
    PoseArray (geometry_msgs/PoseArray)
    最优姿态：位置 (x,y,z) + 旋转四元数 (qx,qy,qz,qw)
```

**关键特点：**
- **零样本**：仅需目标物体的 CAD 模型（.obj/.ply），无需标注数据
- **精度**：AUC 0.1（LINEMOD benchmark）~92%，接近监督方法
- **模型-无关**：相同模型权重可直接处理任意新物体

### 1.2 `isaac_ros_foundationpose` 配置

```python
# launch/foundationpose.launch.py
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

container = ComposableNodeContainer(
    name='pose_estimation_container',
    package='rclcpp_components',
    executable='component_container_mt',
    composable_node_descriptions=[
        # 深度 → 点云（NITROS 加速）
        ComposableNode(
            package='isaac_ros_depth_image_proc',
            plugin='nvidia::isaac_ros::depth_image_proc::DepthToPointCloudNode',
            name='depth_to_pc',
        ),
        # FoundationPose 推理节点
        ComposableNode(
            package='isaac_ros_foundationpose',
            plugin='nvidia::isaac_ros::foundationpose::FoundationPoseNode',
            name='foundationpose',
            parameters=[{
                'mesh_file_path': '/models/target_object.obj',
                'texture_path': '/models/target_object_texture.png',
                'score_threshold': 0.5,
                'refine_iterations': 5,       # 精细化迭代次数（越多越准但越慢）
                'symmetry_plane_axes': '',    # 对称轴（对称物体）
            }],
            remappings=[
                ('pose_estimation/depth_image', '/camera/depth/image_raw'),
                ('pose_estimation/image', '/camera/color/image_rect_color'),
                ('pose_estimation/camera_info', '/camera/color/camera_info'),
                ('pose_estimation/segmentation', '/segmentation/mask'),
                ('pose_estimation/output', '/pose_estimation/poses'),
            ],
        ),
    ],
)
```

### 1.3 RGB-D 输入要求与 PoseArray 输出

```python
# 输入话题汇总
inputs = {
    '/camera/color/image_rect_color': 'sensor_msgs/Image',    # 校正后 RGB
    '/camera/depth/image_raw': 'sensor_msgs/Image',           # 16UC1 or 32FC1
    '/camera/color/camera_info': 'sensor_msgs/CameraInfo',    # 内参
    '/segmentation/mask': 'sensor_msgs/Image',                 # 分割掩码（8UC1）
}

# 输出：geometry_msgs/PoseArray
# .poses[0] = 最优姿态（世界坐标系或相机坐标系）
# header.frame_id = 相机光学坐标系
```

---

## 二、DOPE（Deep Object Pose Estimation）

### 2.1 架构与训练流程

DOPE（Tremblay et al., IROS 2018）是 NVIDIA 开源的监督式 6D 姿态估计，需要针对目标物体训练：

```
DOPE 训练流程：

1. 合成训练数据生成（NDDS / omni.replicator）
   ├── 生成带标注的 RGB 图像（8000张/物体）
   ├── 标注：8个关键点（包围盒角点）+ 中心点
   └── Domain Randomization（背景/光照/材质）

2. VGG-19 骨干网络 + 信念图预测头
   ├── 输出：9个关键点的 2D 信念图（heatmap）
   └── 亲和力向量场（关键点→中心点方向）

3. PnP 求解（OpenCV solvePnP）
   ├── 2D 关键点 + 3D CAD 模型 → 6D 姿态
   └── RANSAC 内点过滤

4. ROS 2 节点发布 PoseStamped / PoseArray
```

### 2.2 ROS 2 节点启动

```python
# launch/dope.launch.py
from launch_ros.actions import Node

dope_node = Node(
    package='dope',
    executable='dope',
    name='dope_node',
    parameters=[{
        'weights': {
            'Ketchup': '/models/dope/Ketchup.pth',
            'Mustard': '/models/dope/Mustard.pth',
        },
        'class_ids': {'Ketchup': 0, 'Mustard': 1},
        'draw_bbox': False,
        'thresh_angle': 0.5,
        'thresh_map': 0.01,
        'sigma': 4,
        'thresh_points': 0.1,
        'camera_frame': 'camera_color_optical_frame',
    }],
    remappings=[
        ('camera/image_color', '/camera/color/image_raw'),
        ('camera/camera_info', '/camera/color/camera_info'),
    ],
)
```

---

## 三、isaac_ros_apriltag

### 3.1 AprilTag 检测配置

AprilTag 是轻量、精确的 fiducial marker，适用于快速标定和工作坐标系初始化：

```python
# launch/apriltag.launch.py
ComposableNode(
    package='isaac_ros_apriltag',
    plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
    name='apriltag',
    parameters=[{
        'size': 0.10,           # tag 实际边长（米）
        'max_tags': 64,         # 最大检测数量
        'tile_size': 4,         # tag family 36h11（默认）
    }],
    remappings=[
        ('image', '/camera/color/image_rect_color'),
        ('camera_info', '/camera/color/camera_info'),
        ('tag_detections', '/apriltag/detections'),
    ],
)
```

### 3.2 AprilTagDetectionArray 消息

```python
# isaac_ros_apriltag_interfaces/msg/AprilTagDetectionArray
from isaac_ros_apriltag_interfaces.msg import AprilTagDetection, AprilTagDetectionArray

# 每个检测结果
detection = AprilTagDetection()
detection.family = "36h11"
detection.id = 0              # Tag ID
detection.corners = [...]     # 4个角点（像素坐标）
detection.center = Point2D()  # 中心点
detection.pose = PoseWithCovarianceStamped()  # 6D 姿态（相机坐标系）
```

---

## 四、手眼标定

### 4.1 标定配置（eye-in-hand vs eye-to-hand）

| 配置 | 描述 | 适用场景 | 标定方法 |
|------|------|---------|---------|
| **eye-in-hand** | 相机安装在机械臂末端 | 抓取点精确测量 | `easy_handeye2` AX=XB |
| **eye-to-hand** | 相机固定在工作台/基座 | 监视工作空间 | `easy_handeye2` AX=YB |

### 4.2 easy_handeye2 标定流程

```bash
# 安装
sudo apt install ros-humble-easy-handeye2

# 启动标定节点（eye-in-hand 示例）
ros2 launch easy_handeye2 calibrate.launch.py \
  eye_on_hand:=true \
  robot_base_frame:=base_link \
  robot_effector_frame:=tool0 \
  tracking_base_frame:=camera_color_optical_frame \
  tracking_marker_frame:=target_marker

# 标定过程：
# 1. 将 AprilTag/ChArUco board 放置在工作台
# 2. 移动机械臂到 15-20 个不同姿态
# 3. 每个姿态点击"采集"按钮
# 4. 完成后点击"计算"→ 输出 eye_to_hand.yaml
```

### 4.3 标定结果存储与加载

```yaml
# eye_on_hand_calibration.yaml
transformation:
  x: 0.0523
  y: -0.0187
  z: 0.0934
  qx: -0.0023
  qy: 0.7071
  qz: 0.0018
  qw: 0.7071
# 含义：从 tool0 到 camera_color_optical_frame 的刚体变换
```

```python
# 运行时加载标定结果
from geometry_msgs.msg import TransformStamped
import yaml

def load_handeye_calibration(yaml_path: str) -> TransformStamped:
    with open(yaml_path) as f:
        cal = yaml.safe_load(f)['transformation']
    tf = TransformStamped()
    tf.header.frame_id = 'tool0'
    tf.child_frame_id = 'camera_color_optical_frame'
    tf.transform.translation.x = cal['x']
    tf.transform.translation.y = cal['y']
    tf.transform.translation.z = cal['z']
    tf.transform.rotation.x = cal['qx']
    tf.transform.rotation.y = cal['qy']
    tf.transform.rotation.z = cal['qz']
    tf.transform.rotation.w = cal['qw']
    return tf
```

### 4.4 标定质量评估

| 评估指标 | 良好阈值 | 评估方法 |
|---------|---------|---------|
| 旋转误差 | < 0.5° | 固定点多次测量方差 |
| 平移误差 | < 3mm | 固定点多次测量标准差 |
| 重投影误差 | < 0.5px | 标定残差 |
| 可重复性 | < 1mm | 100次抓取成功率 |

---

## 五、姿态跟踪

### 5.1 FoundationPose 跟踪模式

FoundationPose 提供独立的跟踪模式（Tracking），在初始化后仅做精细化，速度快 3–5 倍：

```
初始化（第一帧）：完整推理（~200ms）
    │
    ▼
跟踪帧（后续帧）：
  ├── 从上一帧姿态出发
  ├── 渲染预测姿态的参考视图
  └── Render+Compare 精细化（~30ms/帧）
    │
    ▼
/pose_estimation/pose_tracking (PoseStamped, 30+ fps)
```

```python
# 跟踪节点参数（tracking mode）
parameters=[{
    'mode': 'track',          # 'pose_estimation' | 'track'
    'refine_iterations': 3,   # 跟踪时减少迭代次数
    'reset_on_lost': True,    # 丢失时自动重新初始化
    'lost_threshold': 0.2,    # 置信度低于此值认为丢失
}]
```

### 5.2 与 MoveIt 2 抓取对接

```python
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
import rclpy
from rclpy.action import ActionClient

class PoseToGrasp(rclpy.node.Node):
    """将 FoundationPose 输出的姿态转换为 MoveIt 2 抓取目标"""

    def __init__(self):
        super().__init__('pose_to_grasp')
        self.create_subscription(
            PoseArray, '/pose_estimation/poses',
            self.on_pose, 10
        )
        self._move_client = ActionClient(self, MoveGroup, '/move_action')
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

    def on_pose(self, msg: PoseArray):
        if not msg.poses:
            return
        # 取置信度最高的姿态
        target_pose = msg.poses[0]

        # 转换到 base_link 坐标系
        try:
            tf = self._tf_buffer.lookup_transform(
                'base_link', msg.header.frame_id,
                rclpy.time.Time()
            )
            pose_base = do_transform_pose(target_pose, tf)
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return

        # 计算抓取预姿态（沿 z 轴后退 10cm）
        pre_grasp = PoseStamped()
        pre_grasp.header.frame_id = 'base_link'
        pre_grasp.pose = pose_base
        pre_grasp.pose.position.z += 0.10

        self.move_to_pose(pre_grasp)

    def move_to_pose(self, target: PoseStamped):
        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.goal_constraints = [
            pose_to_motion_constraint(target, tolerance_pos=0.005, tolerance_ori=0.01)
        ]
        self._move_client.send_goal_async(goal)
```
