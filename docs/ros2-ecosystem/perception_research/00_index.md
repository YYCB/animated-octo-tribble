# 多模态感知栈 — 总索引

> **调研范围**：面向具身智能的完整感知技术栈——从目标检测、6D 姿态估计、3D 重建，到视觉语言理解（VLM），再与 ROS 2 控制栈 / MoveIt 2 / Nav2 的深度集成。  
> **与 Phase 2 的关系**：Phase 2（Isaac ROS/NITROS）建立了 GPU 零拷贝通信框架；本章深化具体感知算法——哪些算法可利用 NITROS 加速、如何与 VLA/MoveIt 2 对接。

---

## 一、具身智能感知需求矩阵

| 感知任务 | 抓取操控 | 移动导航 | 人机交互 | 开放语义理解 | 数据采集 |
|---------|---------|---------|---------|------------|---------|
| **目标检测（2D）** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐ |
| **6D 姿态估计** | ⭐⭐⭐⭐⭐ | ⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐ |
| **语义分割** | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ |
| **3D 重建（Nvblox）** | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐ |
| **视觉语言理解（VLM）** | ⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐ |
| **深度估计** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐ | ⭐⭐ | ⭐⭐⭐⭐ |
| **人体检测 / 追踪** | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐ |
| **触觉/力觉感知** | ⭐⭐⭐⭐⭐ | ⭐ | ⭐⭐ | ⭐⭐ | ⭐⭐⭐⭐ |

> ⭐⭐⭐⭐⭐ = 关键需求 / ⭐ = 可选

---

## 二、完整感知栈架构图

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         具身智能完整感知栈                                     │
│                                                                             │
│  传感器层                                                                     │
│  ┌────────────────────────────────────────────────────────────────────────┐ │
│  │  RGB Camera    Depth Camera    LiDAR    IMU    FT Sensor    Tactile    │ │
│  │  (RealSense /  (D435/L515)     (OS1-64) (ICM)  (ATI Nano17) (GelSight)│ │
│  │   OAK-D / ZED)                                                         │ │
│  └─────────────────────────┬──────────────────────────────────────────────┘ │
│                            │ sensor_msgs: Image/PointCloud2/Imu/Wrench      │
│  传输与预处理层               │                                               │
│  ┌─────────────────────────▼──────────────────────────────────────────────┐ │
│  │  isaac_ros / image_pipeline                                            │ │
│  │  ┌──────────────┐  ┌──────────────┐  ┌───────────────────────────────┐│ │
│  │  │image_proc    │  │ NITROS       │  │point_cloud_transport           ││ │
│  │  │rectify       │  │ 零拷贝管线   │  │compressed/zstd/draco           ││ │
│  │  │crop_decimate │  │ (Phase 2)    │  │                                ││ │
│  │  └──────────────┘  └──────────────┘  └───────────────────────────────┘│ │
│  └─────────────────────────┬──────────────────────────────────────────────┘ │
│                            │                                               │
│  感知算法层                  │                                               │
│  ┌─────────────────────────▼──────────────────────────────────────────────┐ │
│  │                                                                        │ │
│  │  目标检测              姿态估计           3D 重建           VLM          │ │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────┐  ┌──────────────┐  │ │
│  │  │isaac_ros_    │  │isaac_ros_    │  │NvbloxNode│  │Grounding     │  │ │
│  │  │rtdetr        │  │foundationpose│  │          │  │DINO          │  │ │
│  │  │isaac_ros_    │  │DOPE          │  │nvblox_   │  │CLIP/SigLIP   │  │ │
│  │  │yolov8        │  │isaac_ros_    │  │human_det │  │LangSAM/SAM2  │  │ │
│  │  │PointPillars  │  │apriltag      │  │          │  │              │  │ │
│  │  └──────────────┘  └──────────────┘  └──────────┘  └──────────────┘  │ │
│  └─────────────────────────┬──────────────────────────────────────────────┘ │
│                            │                                               │
│  感知输出话题               │                                               │
│  ┌─────────────────────────▼──────────────────────────────────────────────┐ │
│  │  /detections (Detection2DArray)                                        │ │
│  │  /poses (PoseArray / PoseWithCovarianceStamped)                        │ │
│  │  /map (OccupancyGrid / nvblox_msgs/Mesh)                               │ │
│  │  /classifications (ClassificationResult)                               │ │
│  └─────────────────────────┬──────────────────────────────────────────────┘ │
│                            │                                               │
│  规划层                     │                                               │
│  ┌─────────────────────────▼──────────────────────────────────────────────┐ │
│  │  Nav2 (导航) │ MoveIt 2 (操控) │ BT Navigator │ VLA Policy             │ │
│  └────────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 三、与 Isaac ROS（Phase 2）的关系

| 感知组件 | Phase 2 NITROS 支持 | 本章深化内容 |
|---------|-------------------|------------|
| `image_proc` 节点 | ✅ NITROS 加速版本 | 与检测算法的标准化流程 |
| `isaac_ros_rtdetr` | ✅ TensorRT + NITROS | RT-DETR 架构、性能调优、与 MoveIt 2 对接 |
| `isaac_ros_yolov8` | ✅ NITROS | YOLO 部署全流程、模型量化 |
| `isaac_ros_foundationpose` | ✅ NITROS 零拷贝 | zero-shot 6D 姿态、跟踪模式 |
| `nvblox_ros` | ✅ GPU 加速 | 参数调优、Nav2 costmap 集成 |
| `Grounding DINO` | ⚠️ 无官方 NITROS | gRPC 远端推理 vs 本机 TRT |
| `CLIP/SigLIP` | ❌ | clip_ros2 社区节点 |

---

## 四、文档导航表

| 文件 | 内容摘要 |
|------|---------|
| [00_index.md](./00_index.md) | 本文档：需求矩阵、感知栈架构图、与 Phase 2 关系、导航表 |
| [01_object_detection.md](./01_object_detection.md) | 目标检测：RT-DETR / YOLOv8 / PointPillars、image_proc 流水线、MoveIt 2 对接 |
| [02_pose_estimation.md](./02_pose_estimation.md) | 6D 姿态估计：FoundationPose / DOPE / AprilTag、手眼标定、姿态跟踪 |
| [03_nvblox_3d_reconstruction.md](./03_nvblox_3d_reconstruction.md) | Nvblox 3D 重建：TSDF/ESDF/颜色层、Nav2 集成、人体感知、性能数据 |
| [04_visual_language.md](./04_visual_language.md) | 视觉语言：CLIP/Grounding DINO/LangSAM、与 LLM 联动、推理延迟分析 |
| [05_integration.md](./05_integration.md) | 与本仓库对接：端到端延迟、NITROS 路径、VLA 数据飞轮、RT 隔离、选型矩阵 |

---

## 五、感知栈版本矩阵

| 工具 | 推荐版本 | ROS 2 Humble | ROS 2 Jazzy | 备注 |
|------|---------|-------------|-------------|------|
| isaac_ros_rtdetr | 3.x | ✅ | ✅ | 需 JetPack 6.0 / x86+TRT |
| isaac_ros_yolov8 | 3.x | ✅ | ✅ | YOLOv8n/s/m/l/x |
| isaac_ros_foundationpose | 3.x | ✅ | ✅ | 需大 GPU 显存（≥ 8GB） |
| nvblox_ros | 2.x | ✅ | ✅ | Orin 推荐 voxel_size ≥ 0.03m |
| isaac_ros_apriltag | 3.x | ✅ | ✅ | 36h11 tag family |
| Grounding DINO | 1.x (HuggingFace) | ⚠️ 社区 | ⚠️ 社区 | `isaac_ros_groundingdino` |
| SAM 2 | 2.0 | ⚠️ 社区 | ⚠️ 社区 | 需 GPU ≥ 16GB |
| clip_ros2 | 0.x | ⚠️ 社区 | ⚠️ 社区 | SigLIP 更快 |
