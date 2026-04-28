# 视觉语言理解

> 覆盖 CLIP/SigLIP 零样本分类、Grounding DINO 开放词汇检测、LangSAM/SAM 2 语言引导分割，以及与 Phase 3 LLM 的完整联动链路和推理延迟分析。

---

## 一、CLIP / SigLIP

### 1.1 CLIP 零样本分类

CLIP（OpenAI，2021）通过对比学习对齐视觉-语言特征空间，支持任意文本标签的零样本分类：

```python
# clip_ros2 节点实现（简化）
import rclpy
import torch
import clip
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Classification2D, ObjectHypothesis

class CLIPClassificationNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('clip_classification')
        self.declare_parameter('model', 'ViT-B/32')
        self.declare_parameter('device', 'cuda')

        model_name = self.get_parameter('model').value
        device = self.get_parameter('device').value

        self.model, self.preprocess = clip.load(model_name, device=device)
        self.device = device

        # 动态文本类别（可通过话题更新）
        self.categories = ['apple', 'bottle', 'cup', 'book']
        self._update_text_features()

        self.create_subscription(Image, '/camera/color/image_raw',
                                  self.on_image, 10)
        self.create_subscription(String, '/clip/set_categories',
                                  self.on_categories, 10)
        self.pub = self.create_publisher(
            Classification2D, '/clip/classification', 10
        )

    def _update_text_features(self):
        texts = clip.tokenize(self.categories).to(self.device)
        with torch.no_grad():
            self.text_features = self.model.encode_text(texts)
            self.text_features /= self.text_features.norm(dim=-1, keepdim=True)

    def on_image(self, msg: Image):
        # 转换 ROS Image → PIL → CLIP 预处理
        img = ros_image_to_pil(msg)
        img_tensor = self.preprocess(img).unsqueeze(0).to(self.device)

        with torch.no_grad():
            image_features = self.model.encode_image(img_tensor)
            image_features /= image_features.norm(dim=-1, keepdim=True)
            similarity = (100.0 * image_features @ self.text_features.T).softmax(dim=-1)

        scores = similarity[0].cpu().numpy()
        result = Classification2D()
        result.header = msg.header
        for i, (cat, score) in enumerate(zip(self.categories, scores)):
            hyp = ObjectHypothesis(class_id=cat, score=float(score))
            result.results.append(hyp)
        self.pub.publish(result)
```

### 1.2 SigLIP vs CLIP 对比

SigLIP（Zhai et al., Google 2023）用 Sigmoid 损失替代 Softmax，在小 batch 下效果更好：

| 对比维度 | CLIP (ViT-B/32) | SigLIP (So400M) | 备注 |
|---------|----------------|-----------------|------|
| ImageNet 零样本准确率 | 63.3% | 83.2% | SigLIP 显著优于 CLIP |
| 推理速度（Orin AGX） | ~15ms | ~35ms | SigLIP 模型更大 |
| 图像特征维度 | 512 | 1152 | |
| HuggingFace 支持 | ✅ `openai/clip-vit-base-patch32` | ✅ `google/siglip-so400m-patch14-384` | |
| ROS 2 集成 | ✅ clip_ros2 | ✅ 类 CLIP 节点可复用 | |

---

## 二、Grounding DINO

### 2.1 开放词汇目标检测架构

Grounding DINO（Liu et al., 2023）结合 DINO 检测器与语言理解，支持文本 prompt → BBox 检测：

```
文本 Prompt："red apple on the table"
    │
    ▼
BERT 文本编码器（语言特征）
    │
    │ 跨模态注意力融合
    ▼
Swin Transformer 图像骨干（视觉特征）
    │
    ▼
特征增强器（Feature Enhancer，双向注意力）
    │
    ▼
语言引导查询选择（Language-Guided Query Selection）
    │
    ▼
Grounding 解码器 → BBox + 文本匹配得分
    │
    ▼
Detection2DArray
  └── class_id = "red apple" / "table" （来自 prompt）
```

### 2.2 isaac_ros_groundingdino 配置

```python
# launch/groundingdino.launch.py
from launch_ros.actions import Node

groundingdino_node = Node(
    package='isaac_ros_groundingdino',
    executable='groundingdino_node',
    name='groundingdino',
    parameters=[{
        'model_dir': '/models/groundingdino/',
        'config_file': 'GroundingDINO_SwinB.cfg.py',
        'checkpoint': 'groundingdino_swinb_cogcoor.pth',
        'box_threshold': 0.35,
        'text_threshold': 0.25,
        # 检测文本 prompt（可运行时动态更新）
        'text_prompt': 'cup . bottle . apple .',  # `. ` 分隔多类别
    }],
    remappings=[
        ('image', '/camera/color/image_rect_color'),
        ('camera_info', '/camera/color/camera_info'),
        ('groundingdino/detections', '/detections'),
    ],
)
```

### 2.3 动态 prompt 更新接口

```python
# 运行时动态更新检测目标（来自 LLM 指令）
from std_msgs.msg import String

# LLM 发布检测目标描述
self.create_publisher(String, '/groundingdino/text_prompt', 10)

# 发布新 prompt
msg = String()
msg.data = "red mug on the left side of the desk ."
self.prompt_pub.publish(msg)
```

---

## 三、LangSAM / SAM 2

### 3.1 LangSAM：语言引导分割

LangSAM 将 Grounding DINO 的检测框与 SAM（Segment Anything）结合：

```
文本 Prompt
    │
    ▼
Grounding DINO → BBox（粗定位）
    │
    ▼
SAM 提示编码器（BBox → Prompt Embedding）
    │
    ▼
SAM 掩码解码器（Mask Decoder）
    │
    ▼
高精度实例分割 mask
    │
    ▼
mask → 3D 点云滤波 → 抓取点生成
```

### 3.2 SAM 2（视频分割）

SAM 2（META，2024）支持视频流中的实例跟踪分割：

```python
# SAM 2 ROS 2 集成（社区实现）
from sam2.sam2_image_predictor import SAM2ImagePredictor
from sam2.build_sam import build_sam2

class SAM2ROSNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('sam2_node')
        self.predictor = SAM2ImagePredictor(
            build_sam2("sam2_hiera_large.yaml",
                       "/models/sam2_hiera_large.pt")
        )
        self.create_subscription(Image, '/camera/color/image_rect_color',
                                  self.on_image, 10)
        # 接收 Grounding DINO 的检测框作为 prompt
        self.create_subscription(Detection2DArray, '/detections',
                                  self.on_detections, 10)

    def on_detections(self, det_msg: Detection2DArray):
        if self.current_image is None:
            return
        input_boxes = []
        for det in det_msg.detections:
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            w = det.bbox.size_x
            h = det.bbox.size_y
            # xyxy 格式
            input_boxes.append([cx - w/2, cy - h/2, cx + w/2, cy + h/2])

        with torch.inference_mode():
            self.predictor.set_image(self.current_image)
            masks, scores, _ = self.predictor.predict(
                box=np.array(input_boxes),
                multimask_output=False,
            )
        # 发布分割掩码
        self.publish_masks(masks, det_msg.header)
```

### 3.3 分割 mask → 抓取点生成

```python
import numpy as np

def mask_to_grasp_candidate(
    mask: np.ndarray,           # (H, W) bool
    depth_image: np.ndarray,    # (H, W) float32, meters
    camera_info: dict,          # {fx, fy, cx, cy}
) -> list[tuple]:
    """从分割 mask 生成 3D 抓取点候选"""
    # 找 mask 质心
    ys, xs = np.where(mask)
    if len(xs) == 0:
        return []

    cx_pix = int(np.median(xs))
    cy_pix = int(np.median(ys))
    z = float(depth_image[cy_pix, cx_pix])

    if z <= 0.05 or z > 2.0:
        return []

    # 反投影到 3D（相机坐标系）
    x = (cx_pix - camera_info['cx']) * z / camera_info['fx']
    y = (cy_pix - camera_info['cy']) * z / camera_info['fy']

    # 从 mask 边界框估计抓取旋转（主轴方向）
    from scipy.spatial import ConvexHull
    hull_points = np.stack([xs, ys], axis=1)
    if len(hull_points) > 10:
        hull = ConvexHull(hull_points)
        hull_pts = hull_points[hull.vertices]
        # 主轴 PCA
        pca_angle = np.arctan2(
            *np.linalg.svd(hull_pts - hull_pts.mean(axis=0))[2][0][::-1]
        )
    else:
        pca_angle = 0.0

    return [(x, y, z, pca_angle)]
```

---

## 四、与 Phase 3 LLM 的对接

### 4.1 LLM → 感知 → 操控完整链路

```
用户语言指令
"把桌上的红色杯子拿给我"
    │
    ▼
Phase 3 LLM（Qwen2.5 / Llama 3）
  ├── 实体提取：目标="红色杯子"，位置="桌上"
  └── 发布 /llm/perception_query (std_msgs/String)
    │
    ▼
Grounding DINO Node
  ├── text_prompt = "red cup on the table ."
  └── 发布 /detections (Detection2DArray)
    │
    ▼
LangSAM Node
  ├── 输入：Detection2DArray bbox + RGB-D
  └── 发布 /segmentation/mask (sensor_msgs/Image)
    │
    ▼
FoundationPose Node（精确 6D 姿态）
  └── 发布 /pose_estimation/poses (PoseArray)
    │
    ▼
抓取规划节点（PoseToGrasp）
  ├── 坐标系变换（相机 → base_link）
  └── MoveIt 2 execute_grasp()
    │
    ▼
chassis_protocol / ros2_control
（末端执行器 + 机械臂联合运动）
```

### 4.2 LLM → Grounding DINO 接口实现

```python
# perception_bridge.py
class PerceptionBridge(rclpy.node.Node):
    """LLM 感知查询 → Grounding DINO prompt 动态更新桥接节点"""

    def __init__(self):
        super().__init__('perception_bridge')
        self.create_subscription(
            String, '/llm/perception_query', self.on_query, 10
        )
        self.prompt_pub = self.create_publisher(
            String, '/groundingdino/text_prompt', 10
        )
        self.result_pub = self.create_publisher(
            String, '/perception/status', 10
        )

    def on_query(self, msg: String):
        """将 LLM 提取的实体转换为 Grounding DINO prompt 格式"""
        # 解析 JSON（LLM 输出结构化实体）
        import json
        query = json.loads(msg.data)
        entities = query.get('entities', [])

        # 构建 Grounding DINO 文本 prompt（. 分隔）
        prompt = ' . '.join(entities) + ' .'
        self.get_logger().info(f'Grounding DINO prompt: {prompt}')
        self.prompt_pub.publish(String(data=prompt))
```

---

## 五、性能与延迟分析

### 5.1 VLM 推理延迟对比

| 模型 | 推理方式 | 延迟（单张图像） | GPU 内存 | Orin 可用 |
|------|---------|--------------|---------|---------|
| CLIP ViT-B/32 | 本机 TRT FP16 | 5–8ms | 0.3GB | ✅ |
| SigLIP So400M | 本机 TRT FP16 | 20–35ms | 1.5GB | ✅ |
| Grounding DINO SwinT | 本机 PyTorch | 80ms | 1.8GB | ⚠️（慢） |
| Grounding DINO SwinB | 本机 TRT | 45ms | 3.5GB | ⚠️（需 Orin 64GB） |
| Grounding DINO SwinB | 远端 gRPC（A100） | 120ms（含网络） | 服务端 | ✅（低延迟网络） |
| SAM 2 Large | 本机 PyTorch | 120ms | 8GB | ❌（内存不足） |
| SAM 2 Small | 本机 TRT | 35ms | 3GB | ⚠️（Orin 64GB） |
| LangSAM（GDINO+SAM） | 本机 | 160ms | 5GB | ❌ |

### 5.2 与规划层的异步接口设计

VLM 推理延迟较高（45–160ms），不适合同步阻塞规划循环，需要异步接口：

```python
# 异步感知→规划解耦设计
import asyncio
from rclpy.callback_groups import ReentrantCallbackGroup

class AsyncPerceptionPlanning(rclpy.node.Node):
    def __init__(self):
        super().__init__('async_perception_planning')
        self.cb_group = ReentrantCallbackGroup()

        # 感知结果缓存（最新姿态）
        self.latest_pose: PoseArray | None = None
        self.pose_lock = asyncio.Lock()

        # 感知结果订阅（高频，非阻塞）
        self.create_subscription(
            PoseArray, '/pose_estimation/poses',
            self.on_pose, 10,
            callback_group=self.cb_group
        )

        # 规划定时器（独立于感知频率）
        self.create_timer(0.1, self.plan_loop,  # 10Hz 规划
                          callback_group=self.cb_group)

    def on_pose(self, msg: PoseArray):
        """感知回调：更新缓存，不阻塞"""
        self.latest_pose = msg

    def plan_loop(self):
        """规划循环：使用最新可用感知结果"""
        if self.latest_pose is None:
            return
        pose = self.latest_pose
        self.latest_pose = None  # 消费后清除，避免重复规划
        self.execute_grasp_plan(pose)
```

### 5.3 感知延迟分配建议

| 感知任务 | 推荐算法 | 目标延迟 | 部署策略 |
|---------|---------|---------|---------|
| 实时障碍物检测 | YOLOv8n（NITROS） | < 8ms | 本机 TRT，NITROS 零拷贝 |
| 抓取目标定位 | RT-DETR-L | < 25ms | 本机 TRT |
| 精确 6D 姿态 | FoundationPose | < 50ms | 异步，更新缓存 |
| 开放词汇检测 | Grounding DINO SwinT | < 80ms | 异步触发（LLM 指令时） |
| 语义分割 | SAM 2 Small | < 50ms | 按需触发（非持续） |
| 语言分类 | SigLIP | < 35ms | 持续运行 |
