# Isaac ROS 主要包族 — DNN / vSLAM / Nvblox / AprilTag

> 本节对每个包族给出：核心功能、输入/输出 Topic、性能指标、典型配置、与 NITROS 的集成深度

---

## 一、Isaac ROS DNN 推理框架

### 1.1 包结构

```
isaac_ros_dnn_inference/          ← 推理基础框架
  ├── DnnInferenceNode            ← 通用推理节点（接 TensorRT / Triton）
  ├── TensorRTNode                ← 直接调用 TensorRT（本机）
  └── TritonNode                  ← 通过 gRPC 调用 Triton Inference Server

isaac_ros_tensor_list_interfaces/ ← TensorList 消息定义
isaac_ros_dnn_image_encoder/      ← 图像预处理（resize + normalize + HWC→NCHW）
```

### 1.2 数据流

```
原始图像（NitrosImage）
  ↓
DnnImageEncoderNode
  · GPU 上完成 resize（例：1920×1080 → 640×640）
  · 归一化（mean/std subtract）
  · 格式转换（BGR8 → RGB32F, HWC → NCHW）
  ↓ NitrosTensorList（[1,3,640,640] float32 CUDA）
TensorRTNode
  · 加载 .engine 文件（INT8 / FP16 量化）
  · TensorRT inference（CUDA 流）
  ↓ NitrosTensorList（[1,84,8400] float32，YOLO 输出格式）
DetectionDecoderNode（如 isaac_ros_detectnet_decoder）
  · GPU NMS → Detection2DArray
  ↓ vision_msgs::Detection2DArray（CPU，因下游通常是 CPU 节点）
```

### 1.3 TensorRTNode 关键参数

```yaml
tensorrt_node:
  ros__parameters:
    model_file_path: /workspaces/isaac_ros-dev/models/yolov8s.onnx
    engine_file_path: /workspaces/isaac_ros-dev/models/yolov8s.plan
    input_binding_names: ["images"]
    output_binding_names: ["output0"]
    verbose: false
    force_engine_update: false    # true = 每次启动重新转 .plan（慢）
    # 量化
    int8_calib_cache_path: ""     # 空 = 使用 FP16
```

### 1.4 Triton Server 模式（多模型/远端推理）

```yaml
triton_node:
  ros__parameters:
    model_name: yolov8s
    model_repository_paths: [/models]
    input_binding_names: ["images"]
    output_binding_names: ["output0"]
    model_version: -1             # -1 = 最新版本
    # gRPC 地址（本机或远端）
    server_url: localhost:8001
```

---

## 二、Isaac ROS Visual SLAM（cuVSLAM）

### 2.1 核心技术

`isaac_ros_visual_slam` 封装了 NVIDIA 的 **cuVSLAM** 库：

- **特征提取**：GPU 加速的 ORB 特征
- **特征跟踪**：CUDA 光流（Lucas-Kanade GPU）
- **Bundle Adjustment**：GPU 并行优化（基于 g2o）
- **回环检测**：基于 FBoW 的词袋（GPU 加速）
- **地图**：稀疏 3D 点云地图

### 2.2 输入/输出

```
输入：
  /left/image_rect (NitrosImage, mono8 / rgb8)
  /right/image_rect (NitrosImage, mono8)
  /left/camera_info (NitrosCameraInfo)
  /right/camera_info (NitrosCameraInfo)
  /imu (sensor_msgs/Imu, CPU)   ← 可选，提升精度

输出：
  /visual_slam/tracking/odometry (nav_msgs/Odometry)
  /visual_slam/tracking/slam_path (nav_msgs/Path)
  /visual_slam/status (isaac_ros_visual_slam_interfaces/VisualSlamStatus)
  /visual_slam/map_points (sensor_msgs/PointCloud2, 可选)
  /tf (odom → base_link, camera_frame → odom)
```

### 2.3 性能（Jetson AGX Orin）

| 指标 | 数值 |
|------|------|
| 延迟（端到端） | ~8ms |
| 处理帧率 | 30 Hz（1080P 双目） |
| GPU 占用 | ~15%（Orin iGPU） |
| 内存占用 | ~800MB |

### 2.4 典型配置

```yaml
visual_slam_node:
  ros__parameters:
    # 相机配置
    num_cameras: 2
    min_num_images: 2
    enable_imu_fusion: false
    rectified_images: true        # 已经去畸变
    
    # SLAM 模式
    enable_localization_n_mapping: true
    map_frame: map
    odom_frame: odom
    base_frame: base_link
    
    # 性能
    image_buffer_size: 10
    imu_buffer_size: 200
```

---

## 三、Nvblox — 实时 3D 体素地图

### 3.1 核心技术

- **TSDF**（Truncated Signed Distance Field）：毫米级 3D 重建
- **ESDF**（Euclidean Signed Distance Field）：到障碍物最近距离场（用于路径规划）
- **voxel hashing**：稀疏体素哈希表，GPU 并行更新
- **Mesh 提取**：Marching Cubes on GPU

### 3.2 输入/输出

```
输入：
  /depth_image (NitrosImage, depth32f)
  /color_image (NitrosImage, rgb8)        ← 可选（用于彩色重建）
  /camera_info (NitrosCameraInfo)
  /tf（camera → world 变换，由 vSLAM 或 AMCL 提供）

输出：
  /nvblox_node/mesh (nvblox_msgs/Mesh)           ← 3D 网格（可视化）
  /nvblox_node/static_esdf_pointcloud (PointCloud2) ← ESDF 切片（Nav2 使用）
  /nvblox_node/map_slice (nav_msgs/OccupancyGrid) ← 2D 代价地图（Nav2 直接使用）
```

### 3.3 与 Nav2 集成

```
Nvblox ESDF/Slice
  → nav2_nvblox_costmap_plugin（Nvblox 提供的 Nav2 costmap 插件）
  → Nav2 LocalCostmap / GlobalCostmap
  → 路径规划 + 局部避障
```

这使得具身智能机器人可以在**未知环境中实时构图并同时导航**（Active SLAM）。

---

## 四、Isaac ROS AprilTag

### 4.1 特点

- 全 GPU 加速（OpenCL → CUDA）的 AprilTag 检测
- 支持 Tag36h11 / Tag25h9 / Tagstandard41h12
- 在 Jetson AGX Orin 上：1080P @ 60 Hz，延迟 < 5ms

### 4.2 输入/输出

```
输入：
  /image (NitrosImage)
  /camera_info (NitrosCameraInfo)

输出：
  /tag_detections (isaac_ros_apriltag_interfaces/AprilTagDetectionArray)
    包含：tag_id, pose (PoseWithCovarianceStamped), size, corners
  /tf（若启用 tf_publish）
```

### 4.3 使用场景（具身智能）

- 手眼标定：机械臂末端安装相机，标定 `tool0 → camera_optical_frame`
- 工件定位：已知 AprilTag 贴在工件上，实时获取工件位姿
- 环境标记：在作业区域放置 AprilTag 作为定位锚点

---

## 五、Isaac ROS H.264 编解码

### 5.1 Jetson 硬件编解码

`isaac_ros_h264_encoder` / `isaac_ros_h264_decoder` 利用 Jetson 的 **NVENC/NVDEC** 硬件单元：

```
isaac_ros_h264_encoder：
  输入：/image_raw (NitrosImage, bgr8, 4K @ 30Hz)
  输出：/image_compressed (CompressedImage, H.264)
  码率：约 10-15 Mbps（相比未压缩 ~3 GB/s，压缩比 200:1+）
  延迟：< 2ms（硬件编码）

isaac_ros_h264_decoder：
  输入：/image_compressed (CompressedImage)
  输出：/image_raw (NitrosImage, 可以直接进 DNN 推理管线)
```

### 5.2 远程传输场景

```
机器人（Jetson）                    上位机（X86）
相机（4K @ 30Hz）
  → H.264 编码（NVENC，2ms）
  → 压缩 topic（~12 Mbps）
  ──────────── Wi-Fi / 以太网 ─────────► H.264 解码（NVDEC）
                                          → 显示 / 录制
```

---

## 六、包族功能矩阵（具身智能场景适用性评估）

| 包族 | 室内移动机器人 | 桌面操作臂 | 人形机器人 | 所需 GPU |
|------|:---:|:---:|:---:|------|
| DNN 推理（TensorRT） | ✅ | ✅ | ✅ | Jetson / dGPU |
| Visual SLAM（cuVSLAM） | ✅ | ⚠️（需双目） | ✅ | Jetson 优先 |
| Nvblox（3D 建图） | ✅ | ⚠️（小范围） | ✅ | Jetson 优先 |
| AprilTag | ✅ | ✅（标定） | ✅（标定） | Jetson / dGPU |
| FoundationPose（6-DOF 位姿） | — | ✅ | ✅ | Jetson AGX |
| H.264 编解码 | ✅（传输） | — | ✅（头部相机） | Jetson |
| Body Pose Estimation | — | — | ✅ | Jetson AGX |

⚠️ = 可用但非主要场景；— = 不适用
