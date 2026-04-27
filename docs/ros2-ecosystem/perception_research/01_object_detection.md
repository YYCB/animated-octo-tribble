# 目标检测

> 覆盖 Isaac ROS RT-DETR / YOLOv8 部署、标准化 image_proc 流水线、3D 目标检测，以及与 MoveIt 2 动态障碍物规划的完整对接。

---

## 一、isaac_ros_rtdetr（RT-DETR）

### 1.1 RT-DETR 架构概述

RT-DETR（Real-Time Detection Transformer，百度 2023）是第一个端到端实时 Transformer 检测器，无需 NMS 后处理：

```
RT-DETR 推理管线：

输入图像 (H×W×3)
    │
    ▼
ResNet-50 / HGNetv2 骨干网络（特征提取）
    │
    ▼
混合编码器（Hybrid Encoder）
  ├── AIFI（Attention-based Intra-scale Feature Interaction）
  └── CCFF（Cross-scale CNN-based Feature Fusion）
    │
    ▼
IoU-aware Query Selection（选取 top-k decoder query）
    │
    ▼
Transformer 解码器（6层）
    │
    ▼
输出：Bounding Box (cx,cy,w,h) + 类别置信度
（无 NMS，端到端检测）
```

**优势：**
- 推理速度：RT-DETR-L 在 T4 GPU 上 114 fps（TensorRT FP16）
- COCO val mAP 53.0%（RT-DETR-X），优于 YOLOv8-X（53.9%，但后者 NMS 复杂度高）
- 支持灵活调整推理时的解码器层数（速度/精度权衡）

### 1.2 TensorRT 部署

```bash
# 1. 获取 ONNX 模型
pip install ultralytics
yolo export model=rtdetr-l.pt format=onnx imgsz=640

# 2. 转换 TensorRT engine（FP16）
/usr/src/tensorrt/bin/trtexec \
  --onnx=rtdetr-l.onnx \
  --saveEngine=rtdetr-l.engine \
  --fp16 \
  --minShapes=images:1x3x640x640 \
  --optShapes=images:1x3x640x640 \
  --maxShapes=images:4x3x640x640 \
  --workspace=4096

# 3. 验证吞吐
/usr/src/tensorrt/bin/trtexec \
  --loadEngine=rtdetr-l.engine \
  --batch=1 --iterations=100
```

### 1.3 `Detection2DArray` 消息

```python
# sensor_msgs/msg/Image → vision_msgs/msg/Detection2DArray
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D
from geometry_msgs.msg import Pose2D

# RT-DETR 输出解码示例
def decode_rtdetr_output(boxes: np.ndarray,   # (N, 4) cx,cy,w,h
                         scores: np.ndarray,   # (N,) 置信度
                         labels: np.ndarray,   # (N,) 类别 ID
                         score_threshold: float = 0.5,
                         img_w: int = 640, img_h: int = 480) -> Detection2DArray:
    det_array = Detection2DArray()
    det_array.header.stamp = rclpy.clock.Clock().now().to_msg()

    for i in range(len(boxes)):
        if scores[i] < score_threshold:
            continue
        det = Detection2D()
        cx, cy, w, h = boxes[i]
        det.bbox.center.position.x = float(cx * img_w)
        det.bbox.center.position.y = float(cy * img_h)
        det.bbox.size_x = float(w * img_w)
        det.bbox.size_y = float(h * img_h)
        result = ObjectHypothesisWithPose()
        result.hypothesis.class_id = str(labels[i])
        result.hypothesis.score = float(scores[i])
        det.results.append(result)
        det_array.detections.append(det)

    return det_array
```

### 1.4 isaac_ros_rtdetr launch 配置

```python
# launch/rtdetr_detection.launch.py
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

container = ComposableNodeContainer(
    name='detection_container',
    package='rclcpp_components',
    executable='component_container_mt',
    composable_node_descriptions=[
        # 图像预处理（NITROS 加速）
        ComposableNode(
            package='isaac_ros_image_proc',
            plugin='nvidia::isaac_ros::image_proc::RectifyNode',
            name='rectify_node',
            remappings=[('/image_raw', '/camera/color/image_raw')],
        ),
        # RT-DETR 推理
        ComposableNode(
            package='isaac_ros_rtdetr',
            plugin='nvidia::isaac_ros::rtdetr::RtDetrDecoderNode',
            name='rtdetr_decoder',
            parameters=[{
                'engine_file_path': '/models/rtdetr-l.engine',
                'input_tensor_names': ['images'],
                'output_tensor_names': ['labels', 'boxes', 'scores'],
                'score_threshold': 0.45,
                'num_classes': 80,
            }],
        ),
    ],
)
```

### 1.5 Orin AGX 性能数据

| 模型 | 精度（COCO mAP） | Orin AGX 延迟（ms） | Orin AGX fps | 功耗（W） |
|------|---------------|-------------------|-------------|---------|
| RT-DETR-R50 | 53.1% | 18.5 | 54 | 12W |
| RT-DETR-L | 53.0% | 22.3 | 45 | 14W |
| RT-DETR-X | 54.8% | 38.7 | 26 | 18W |

---

## 二、isaac_ros_yolov8

### 2.1 YOLOv8 TensorRT 部署

```bash
# 导出 YOLOv8 TensorRT（Ultralytics 官方 CLI）
yolo export model=yolov8s.pt format=engine device=0 \
  imgsz=640 half=True batch=1

# 或 ONNX 中间格式
yolo export model=yolov8s.pt format=onnx imgsz=640 opset=17
```

### 2.2 DetectionImage → Detection2DArray 转换节点

`isaac_ros_yolov8` 包内置 `YoloV8DecoderNode`，负责将原始张量输出转为 ROS 2 消息：

```
原始 YOLOv8 输出张量 (1, 84, 8400)
    │
    ▼
YoloV8DecoderNode
  ├── confidence_threshold 过滤
  ├── NMS（isaac_ros_tensor_proc）
  └── 坐标反归一化 + Detection2DArray 打包
    │
    ▼
/yolo/detections (vision_msgs/Detection2DArray)
```

### 2.3 YOLOv8 性能对比

| 模型 | mAP50（COCO） | Orin AGX fps（TRT FP16） | 模型大小 |
|------|-------------|------------------------|--------|
| YOLOv8n | 37.3% | 210 fps | 3.2MB |
| YOLOv8s | 44.9% | 140 fps | 11.2MB |
| YOLOv8m | 50.2% | 80 fps | 25.9MB |
| YOLOv8l | 52.9% | 52 fps | 43.7MB |
| YOLOv8x | 53.9% | 33 fps | 68.2MB |

---

## 三、image_proc 标准化流水线

### 3.1 完整流水线

```
标准化检测前处理流水线：

/camera/color/image_raw (sensor_msgs/Image, 畸变)
    │
    ▼
image_proc::RectifyNode
  参数：interpolation=1 (线性插值)
    │
    ▼
/camera/color/image_rect_color (校正后图像)
    │
    ▼
image_proc::CropDecimateNode （可选，降分辨率）
  参数：decimation_x=2, decimation_y=2
    │
    ▼
/camera/color/image_rect_color_small (检测输入)
    │
    ▼
RT-DETR / YOLOv8 推理节点
    │
    ▼
/detections (Detection2DArray)
```

### 3.2 image_transport 插件选择

| 传输插件 | 压缩比 | CPU 解码开销 | 适用场景 |
|---------|-------|------------|---------|
| `raw` | 1x | 无 | 本地 IPC / NITROS |
| `compressed` (JPEG) | 10–20x | 低 | 有线网络传输 |
| `theora` | 20–50x | 中（软件解码） | 低带宽场景 |
| `zstd` | 3–5x（无损） | 低 | 录制到磁盘 |
| `draco` | 10–20x（点云） | 低 | LiDAR 远端传输 |

---

## 四、3D 目标检测

### 4.1 PointPillars 架构

PointPillars（Lang et al., 2019）是激光雷达 3D 检测的主流方案：

```
PointCloud2 输入
    │
    ▼
Pillar Feature Net（点云 → 伪图像）
  ├── 将点云按 xy 网格分成 Pillar（竖向柱体）
  ├── 每个 Pillar 内取最多 N 个点，提取 9D 特征
  └── PointNet 轻量编码 → (C, H, W) 伪图像
    │
    ▼
Backbone（2D CNN，类 VGG）+ FPN 颈部
    │
    ▼
SSD 检测头
    │
    ▼
Detection3DArray（BBox3D + 类别 + 置信度）
```

### 4.2 `Detection3DArray` 消息

```python
from vision_msgs.msg import Detection3DArray, Detection3D, BoundingBox3D
from geometry_msgs.msg import Point, Quaternion, Vector3

det3d = Detection3D()
det3d.bbox.center.position = Point(x=1.5, y=0.3, z=0.8)   # 目标中心（雷达坐标系）
det3d.bbox.center.orientation = Quaternion(w=1.0)            # 朝向（偏航角）
det3d.bbox.size = Vector3(x=0.6, y=0.4, z=0.3)              # 包围盒大小（m）
```

### 4.3 CenterPoint（BEV 检测）

相比 PointPillars，CenterPoint（Yin et al., 2021）用热图中心点代替 anchor，更适合旋转目标：

```bash
# 安装 mmdet3d（CenterPoint 官方实现）
pip install mmdet3d

# 转换 TensorRT（Orin）
python tools/deployment/pytorch2onnx.py \
  configs/centerpoint/centerpoint_0075voxel_second_secfpn_circlenms_4x8_cyclic_20e_nus.py \
  checkpoint.pth \
  --out centerpoint.onnx --input-img sample_data.bin
```

---

## 五、与 MoveIt 2 的对接

### 5.1 DetectionToCollisionObject 流程

```
/detections (Detection2DArray)
    │ + /camera/depth/points (PointCloud2)
    │
    ▼
DetectionToCollisionObjectNode
  ├── 从 Detection2D bbox + DepthCloud 提取 3D 点簇
  ├── 计算包围盒（BoundingBox3D）
  └── 发布到 /planning_scene
    │
    ▼
MoveIt 2 PlanningScene
  ├── collision_objects: 静态障碍物（已知物体）
  └── acm: AllowedCollisionMatrix 更新
    │
    ▼
OMPL / Pilz 规划器（自动避障）
```

### 5.2 Python 示例：动态障碍物加入 planning scene

```python
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped
import rclpy

class DetectionToMoveIt(rclpy.node.Node):
    def __init__(self):
        super().__init__('detection_to_moveit')
        self.scene_pub = self.create_publisher(
            PlanningScene, '/planning_scene', 10
        )
        self.create_subscription(
            Detection2DArray, '/detections',
            self.detection_callback, 10
        )

    def detection_callback(self, msg: Detection2DArray):
        planning_scene = PlanningScene()
        planning_scene.is_diff = True

        for i, det in enumerate(msg.detections):
            obj = CollisionObject()
            obj.header = msg.header
            obj.id = f"detected_obj_{i}"
            obj.operation = CollisionObject.ADD

            # 从 2D bbox + 深度 → 3D box（近似）
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [
                det.bbox.size_x / 1000.0,   # 像素 → 近似米（需深度对应）
                det.bbox.size_y / 1000.0,
                0.1,                          # 假设高度 10cm
            ]
            obj.primitives.append(primitive)

            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = det.bbox.center.position.x / 1000.0
            pose.pose.position.y = det.bbox.center.position.y / 1000.0
            pose.pose.orientation.w = 1.0
            obj.primitive_poses.append(pose.pose)

            planning_scene.world.collision_objects.append(obj)

        self.scene_pub.publish(planning_scene)
```

---

## 六、性能对比总表

| 算法 | 精度（COCO mAP / AP50） | Orin AGX 延迟 | Orin 功耗 | 优势 | 劣势 |
|------|----------------------|-------------|---------|------|------|
| RT-DETR-L | 53.0% | 22ms | 14W | 高精度、无 NMS | 部署复杂度高 |
| YOLOv8n | 37.3% | 5ms | 4W | 极快、超轻量 | 小目标弱 |
| YOLOv8s | 44.9% | 7ms | 6W | 速度精度平衡 | 遮挡较弱 |
| YOLOv8m | 50.2% | 13ms | 9W | 工业首选 | 需 FP16 |
| PointPillars | AP 3D ~62%（nuScenes） | 30ms | 15W | LiDAR 原生 | 需 LiDAR |
| CenterPoint | AP 3D ~68%（nuScenes） | 45ms | 18W | 旋转目标强 | 更大 GPU 需求 |
