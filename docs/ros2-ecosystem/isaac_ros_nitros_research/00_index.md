# Isaac ROS / NITROS 调研 — 总索引

> **调研版本**：Isaac ROS `3.x`（Humble / Iron / Jazzy 兼容）  
> **参考仓库**：https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros  
> **关联文档**：  
> - 本仓库 `docs/ros2-core/ros2_topic_communication_research/`（IPC / Loaned / SHM 章节）  
> - 本仓库 `docs/ros2-roadmaps/ros2_custom_fork_roadmap/`（性能优化路线图）  
> - REP-2007（Type Adaptation）https://ros.org/reps/rep-2007.html  
> - REP-2009（Type Negotiation）https://ros.org/reps/rep-2009.html

---

## 一、为什么调研 Isaac ROS / NITROS

具身智能系统的感知路径（双目 + 深度 + LiDAR + DNN 推理）面临的核心瓶颈：

```
传感器驱动（CPU）
  → 图像 memcpy 到 ROS message（CPU ↔ GPU 搬运）
  → DDS 序列化（CPU 序列化 + 拷贝）
  → 订阅端反序列化
  → GPU 上传（再一次 CPU→GPU 搬运）
  → DNN 推理（GPU）
```

**每帧 4K 图像（12 MB）在上述路径上可能发生 3~5 次 memcpy**，在 30Hz 下意味着 > 1 GB/s 的无效带宽消耗，且引入 5~20ms 额外延迟。

Isaac ROS / NITROS 的目标：**在保持 ROS 2 Topic 语义的前提下，让图像/点云数据直接在 GPU 显存中流转，零 CPU 拷贝。**

与本仓库 `ros2_custom_fork_roadmap` 的关系：
- custom-fork 路线图提出了 IPC / LoanedMessage / SHM 等优化路径
- NITROS 是 NVIDIA 对这些优化路径的官方 GPU 专向实现
- 本调研将**逐项对照**：哪些优化 NITROS 已做，哪些 NITROS 未覆盖

---

## 二、整体架构总览

```
┌────────────────────────────────────────────────────────────────────┐
│                      应用层 / AI 算法层                             │
│  DNN 推理节点  /  vSLAM  /  Nvblox  /  AprilTag  /  用户自定义节点  │
└────────────────────┬───────────────────────────────────────────────┘
                     │ NitrosPublisher / NitrosSubscriber
                     │（保持 ROS 2 Topic 语义）
┌────────────────────▼───────────────────────────────────────────────┐
│                    NITROS 传输层                                     │
│  ┌────────────────────────────────────────────────────────────┐    │
│  │  Type Negotiation（REP-2009）                               │    │
│  │  发布端与订阅端协商最优内存格式：                            │    │
│  │  NitrosCpuType / NitrosCudaType / NvSciType / DmaBufType   │    │
│  └────────────────────────┬───────────────────────────────────┘    │
│  ┌────────────────────────▼───────────────────────────────────┐    │
│  │  Type Adaptation（REP-2007）                                │    │
│  │  rclcpp::TypeAdapter<NitrosImage, sensor_msgs::Image>       │    │
│  │  → 发布端直接发 GPU buffer，无需先转成 ROS msg              │    │
│  └────────────────────────┬───────────────────────────────────┘    │
└────────────────────────────┼───────────────────────────────────────┘
                             │
         ┌───────────────────┼───────────────────┐
         │                   │                   │
   ┌─────▼──────┐    ┌───────▼─────┐    ┌────────▼────────┐
   │ CPU 内存    │    │ CUDA 显存   │    │ NvSci / DMA-BUF │
   │ (Host)     │    │ (Device)    │    │ (跨进程 GPU buf) │
   └────────────┘    └─────────────┘    └─────────────────┘
         │
   ┌─────▼──────────────────────────────────────────────────────┐
   │  传统 ROS 2 DDS（自动降级，兼容非 NITROS 节点）              │
   └────────────────────────────────────────────────────────────┘
```

**向后兼容原则**：当 topic 的某一端不支持 NITROS 时，自动降级为标准 ROS 2 消息传输，不破坏现有节点。

---

## 三、Isaac ROS 包族全景

| 包族 | 代表包 | 功能 |
|------|--------|------|
| **核心传输** | `isaac_ros_nitros` | NitrosPublisher/Subscriber、Type Negotiation 引擎 |
| **类型定义** | `isaac_ros_nitros_interfaces` | NitrosImage、NitrosPointCloud 等标准类型 |
| **图像处理** | `isaac_ros_image_proc` | 去畸变、resize、颜色转换（全 GPU） |
| **深度感知** | `isaac_ros_stereo_image_proc` | 双目视差、深度图（ELAS/SGM on GPU） |
| **3D 重建** | `isaac_ros_nvblox` | 实时 TSDF/ESDF 地图（voxel hashing on GPU） |
| **视觉定位** | `isaac_ros_visual_slam` | cuVSLAM（GPU 加速 ORB/特征跟踪） |
| **DNN 推理** | `isaac_ros_dnn_inference` | TensorRT / Triton 推理节点 |
| **目标检测** | `isaac_ros_detectnet` / `isaac_ros_yolov8` | GPU 加速检测 |
| **姿态估计** | `isaac_ros_foundationpose` / `isaac_ros_dope` | 6-DOF 物体位姿 |
| **人体感知** | `isaac_ros_body_pose_estimation` | 骨骼关键点 |
| **标记检测** | `isaac_ros_apriltag` | GPU 加速 AprilTag |
| **压缩传输** | `isaac_ros_h264_encoder/decoder` | 硬件 H.264（Jetson 编解码器） |
| **点云** | `isaac_ros_pointcloud_utils` | PointCloud2 处理（GPU） |
| **传感器桥接** | `isaac_ros_image_pipeline` | 与 camera_info、camera_calibration 集成 |

---

## 四、文档目录

| 文件 | 内容 |
|------|------|
| [01_architecture.md](./01_architecture.md) | NITROS 分层架构详解、包内部结构、与 rclcpp 的关系 |
| [02_type_adaptation.md](./02_type_adaptation.md) | REP-2007、`rclcpp::TypeAdapter` 源码级剖析 |
| [03_type_negotiation.md](./03_type_negotiation.md) | REP-2009、NITROS 协商协议、NvSci / DMA-BUF |
| [04_nitros_bridge.md](./04_nitros_bridge.md) | NITROS 桥接节点实现、GPU buffer 在 topic 上的流转 |
| [05_managed_pub_sub.md](./05_managed_pub_sub.md) | NitrosPublisher/Subscriber、NitrosImage/PointCloud 类型 |
| [06_package_ecosystem.md](./06_package_ecosystem.md) | 主要包族（DNN/vSLAM/Nvblox/AprilTag）使用指南 |
| [07_transport_plugins.md](./07_transport_plugins.md) | 与 image_transport / point_cloud_transport 的关系 |
| [08_data_flows.md](./08_data_flows.md) | 端到端 GPU 零拷贝路径时序（含 CPU 降级对比） |
| [09_integration.md](./09_integration.md) | 与 custom-fork 路线图逐项对照（已覆盖 / 未覆盖） |

---

## 五、关键调用链速查

### 5.1 GPU 零拷贝图像流（NITROS 路径）

```
相机驱动节点（realsense_ros / argus_camera）
  → 直接在 CUDA 显存分配 NitrosImage
  → NitrosPublisher::publish(nitros_image)
      → Type Negotiation：双方已协商 CudaType
      → 无序列化，直接传递 CUDA 内存句柄（GpuID + offset）
      → DDS 传输的仅是 20~100 字节的元数据（指针/handle）

DNN 推理节点（TensorRT）
  → NitrosSubscriber 收到元数据
  → 重建 CUDA tensor 指针（同一 GPU 内：直接映射）
  → TensorRT::execute(input_tensor)  ← 零拷贝！
      → 推理结果（检测框 / 分割 mask）写入 CUDA 显存
  → NitrosPublisher::publish(nitros_detection_result)
```

### 5.2 CPU 降级路径（与非 NITROS 节点交互）

```
NITROS 节点 → 发现订阅端不支持 NitrosType
  → 自动触发 TypeAdapter::convert_to_ros_message()
      → CUDA→CPU memcpy（cudaMemcpy）
      → 填充 sensor_msgs::Image
      → 标准 DDS publish
  → 非 NITROS 节点收到标准 ROS 消息
```

---

## 六、与本仓库已有工作的关联

| 已有工作 | Isaac ROS / NITROS 相关位置 | 关系 |
|---------|---------------------------|------|
| `ros2_topic_communication_research/07_loaned_messages.md` | Type Adaptation 是 LoanedMessage 的 GPU 扩展 | 深化 |
| `ros2_topic_communication_research/09_shm_transport.md` | NvSci/DMA-BUF 是 SHM 在 GPU 侧的对应方案 | 并行对比 |
| `ros2_custom_fork_roadmap/05_phase3_rmw_zero_copy.md` | NITROS 是对 rmw 零拷贝在 GPU 侧的完整实现 | 核心对照 |
| `ros2_custom_fork_roadmap/06_phase4_dds_shm_iceoryx.md` | NITROS 绕过 DDS 完全不同路径 | 竞争方案 |
| `realsense-ros-4.57.7-analysis/` | realsense_ros 可作为 NITROS 相机驱动的上游输入 | 集成点 |
