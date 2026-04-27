# 感知栈与本仓库的对接

> 本文档将 Phase 10 感知技术栈与本仓库已有组件（`chassis_protocol`、Phase 2 NITROS、Phase 3 VLA、Phase 7 RT 隔离）进行具体对接，给出端到端延迟分析和选型决策矩阵。

---

## 一、感知→控制端到端延迟分析

### 1.1 完整链路时序

```
/camera/color/image_raw
（RealSense D435, 30fps, 1280×720 RGB）
    │ 传输延迟: ~2ms（IPC 共享内存）
    ▼
image_proc::RectifyNode（NITROS 加速）
    │ 校正: ~1ms（GPU, CUDA kernel）
    ▼
isaac_ros_rtdetr::RtDetrDecoderNode
    │ TensorRT FP16 推理: ~22ms（Orin AGX）
    ▼
/detections (vision_msgs/Detection2DArray)
    │ 话题发布: ~0.5ms（NITROS 零拷贝）
    ▼
BT Navigator（Nav2 / 自定义 BT 节点）
    │ 规划决策: ~5ms
    ▼
/cmd_vel (geometry_msgs/Twist)
    │ 话题发布: ~0.5ms
    ▼
chassis_node（ros2_adapter/chassis_node.cpp）
    │ ROS 2 回调处理: ~1ms
    ▼
ChassisHal::write()
    │ RS485 帧编码 + 发送: ~3ms
    ▼
底盘驱动器执行

─────────────────────────────────────────
总端到端延迟（相机曝光 → 运动执行）：
  约 35ms（含 30fps 帧延迟 33ms → 约 68ms worst case）
```

### 1.2 各链路延迟分配表

| 链路段 | 延迟 | 影响因素 | 优化空间 |
|-------|------|---------|---------|
| 相机帧延迟（30fps） | 33ms | 帧率 | 升至 60fps → 16ms |
| RectifyNode（NITROS） | 1ms | GPU 计算 | 已最优 |
| RT-DETR-L 推理 | 22ms | TRT 引擎 | 换 YOLOv8n → 5ms |
| Detection2DArray 发布 | 0.5ms | IPC | NITROS 零拷贝 |
| BT 决策 | 5ms | 行为树复杂度 | 简化 BT 节点 |
| ros2_control → chassis | 3ms | RS485 | iceoryx2 替代 DDS |
| chassis_protocol RS485 | 3ms | 串口波特率 | 1MHz → 2ms |
| **总计（best case）** | **~35ms** | | |
| **总计（worst case，帧末尾触发）** | **~68ms** | | |

### 1.3 感知频率与控制频率解耦

感知（30Hz）与控制（1kHz）频率相差 33 倍，需要解耦设计：

```cpp
// chassis_node.cpp 中的感知-控制解耦
// 感知结果（DetectionResult）通过 realtime_tools::RealtimeBuffer 传递
// RT 控制环仅读缓存，不阻塞等待感知更新

#include <realtime_tools/realtime_buffer.h>
#include <vision_msgs/msg/detection2_d_array.hpp>

class ChassisNode : public rclcpp_lifecycle::LifecycleNode {
    // 感知结果缓存（线程安全，RT 友好）
    realtime_tools::RealtimeBuffer<DetectionResult> detection_buffer_;

    // 感知回调（非 RT 线程）
    void on_detection(const Detection2DArray::SharedPtr msg) {
        DetectionResult result = convert_detections(*msg);
        detection_buffer_.writeFromNonRT(result);  // 无锁写入
    }

    // RT 控制环（1kHz，SCHED_FIFO）
    void control_loop_update() {
        // 读取最新感知结果（O(1)，无锁）
        auto det = *detection_buffer_.readFromRT();
        // 基于感知结果调整速度指令
        update_cmd_vel_from_detection(det);
    }
};
```

---

## 二、Phase 2 NITROS 加速路径

### 2.1 已有 NITROS 实现的感知节点

| 感知节点 | NITROS 零拷贝 | 性能提升 | 话题类型 |
|---------|-------------|---------|---------|
| `image_proc::RectifyNode` | ✅（`isaac_ros_image_proc`） | 3–5× | NitrosImage |
| `image_proc::CropDecimateNode` | ✅ | 2–3× | NitrosImage |
| `isaac_ros_rtdetr` | ✅（输入 NitrosImage） | 2× | NitrosImage → Detection2DArray |
| `isaac_ros_yolov8` | ✅ | 2× | NitrosImage |
| `isaac_ros_foundationpose` | ✅ | 3× | NitrosImage + NitrosPointCloud |
| `nvblox_ros` | ✅（`NitrosDepthImage`） | 4× | NitrosDepthImage |
| `isaac_ros_apriltag` | ✅ | 2× | NitrosImage |
| `Grounding DINO` | ❌（无官方 NITROS） | — | 标准 Image 话题 |
| `SAM 2` | ❌ | — | 标准 Image 话题 |

### 2.2 NITROS 零拷贝路径配置

```python
# 将多个 NITROS 节点放入同一 ComponentContainer 以启用零拷贝
ComposableNodeContainer(
    name='nitros_perception_container',
    package='rclcpp_components',
    executable='component_container_mt',
    composable_node_descriptions=[
        # 1. 图像校正（NITROS）
        ComposableNode(
            package='isaac_ros_image_proc',
            plugin='nvidia::isaac_ros::image_proc::RectifyNode',
            name='rectify',
        ),
        # 2. RT-DETR 检测（NITROS，接收 NitrosImage）
        ComposableNode(
            package='isaac_ros_rtdetr',
            plugin='nvidia::isaac_ros::rtdetr::RtDetrDecoderNode',
            name='rtdetr',
        ),
        # 3. Nvblox（NITROS 深度图零拷贝）
        ComposableNode(
            package='nvblox_ros',
            plugin='nvblox::NvbloxNode',
            name='nvblox',
        ),
    ],
    # 关键：同一容器内 NITROS 节点自动启用 GPU 共享内存零拷贝
)
```

---

## 三、Phase 3 VLA 数据飞轮

### 3.1 感知结果作为 VLA 输入特征

```
VLA 推理输入特征：

感知输入（多模态）：
  /camera/color/image_raw  → RGB 图像（224×224 或 640×480）
  /joint_states            → 关节角向量（N_joints,）
  /pose_estimation/poses   → 目标姿态（可选，作为额外条件）
  /clip/classification     → 语义标签 embedding（512维）

                ▼
VLA Policy（π0 / OpenVLA / RDT）
  ├── 视觉 Encoder：ViT → 图像 token
  ├── 本体感知 Encoder：MLP → 状态 token
  ├── （可选）语义 Encoder：CLIP text embedding
  └── Action Decoder → 关节轨迹（T_future, N_joints）

                ▼
/vla/joint_trajectory (trajectory_msgs/JointTrajectory)
                ▼
ros2_control JointTrajectoryController
                ▼
chassis_protocol 执行
```

### 3.2 感知-VLA 数据录制（对齐保证）

```yaml
# 录制时保证感知话题时间戳对齐
# 使用 ApproximateTimeSynchronizer（slop=50ms）

topics_to_record:
  - /camera/color/image_raw          # 30 Hz
  - /camera/depth/image_raw          # 30 Hz
  - /joint_states                    # 1000 Hz
  - /pose_estimation/poses           # ~10 Hz（FoundationPose 按需）
  - /detections                      # 30 Hz（RT-DETR）
  - /wrench                          # 1000 Hz
  - /vla/predicted_action            # 10 Hz（VLA 推理频率）
  - /tf                              # 100 Hz

# LeRobot 训练格式：感知特征作为观测空间一部分
observation_keys:
  images: [/camera/color/image_raw]
  state: [/joint_states]
  # 可选：感知结果作为附加特征
  perception: [/detections, /pose_estimation/poses]
```

---

## 四、Phase 7 RT 隔离

### 4.1 感知推理进程与控制进程隔离

```
CPU 核心分配（Jetson Orin 12核）：

Core 0-1  ── 系统后台（OS、驱动）
Core 2-5  ── 感知推理进程（非 RT）
              ├── NITROS container（RT-DETR / nvblox）
              ├── Grounding DINO（PyTorch，按需）
              └── VLA 推理节点（TRT，10Hz）
Core 6    ── controller_manager RT（SCHED_FIFO 90）
Core 7    ── hardware_interface RT（SCHED_FIFO 85）
Core 8    ── RS485 驱动线程（SCHED_FIFO 80）
Core 9    ── ROS 2 通信线程（SCHED_OTHER）
Core 10-11── Nav2 / BT Navigator（SCHED_OTHER）
```

```bash
# 感知进程 CPU 绑定（非 RT 核，避免抢占控制线程）
taskset -c 2-5 ros2 launch perception_stack perception_full.launch.py

# 控制进程 CPU 绑定（RT 核）
taskset -c 6-8 ros2 launch chassis_protocol chassis_bringup.launch.py
```

---

## 五、选型决策矩阵

| 感知任务 | 推荐算法 | 硬件要求 | 延迟目标 | NITROS 支持 | 备注 |
|---------|---------|---------|---------|-----------|------|
| **通用目标检测（实时）** | YOLOv8s (TRT) | Orin NX 8GB | < 10ms | ✅ | 功耗/速度最优 |
| **高精度目标检测** | RT-DETR-L (TRT) | Orin AGX 32GB | < 25ms | ✅ | 操控场景推荐 |
| **3D 目标检测（LiDAR）** | CenterPoint (TRT) | Orin AGX 64GB | < 50ms | ⚠️ | 移动导航 |
| **6D 姿态估计（已知物体）** | DOPE (TRT) | Orin NX 16GB | < 30ms | ⚠️ | 需训练数据 |
| **6D 姿态估计（零样本）** | FoundationPose | Orin AGX 64GB | < 50ms | ✅ | 通用物体 |
| **快速标定/定位** | AprilTag 36h11 | 任何 GPU | < 5ms | ✅ | 不依赖物体识别 |
| **3D 场景重建/导航** | Nvblox (0.05m) | Orin AGX 32GB | < 15ms/帧 | ✅ | Nav2 集成首选 |
| **开放词汇检测** | Grounding DINO SwinT | Orin AGX 32GB | < 80ms（异步） | ❌ | LLM 联动场景 |
| **语义分割（人体）** | PeopleSemSegNet | Orin AGX | < 15ms | ✅（nvblox） | 安全导航 |
| **零样本分类** | SigLIP So400M | Orin AGX | < 35ms | ❌ | 场景理解 |
| **语言引导抓取** | GDINO+SAM 2 Small | Orin AGX 64GB | < 100ms（异步） | ❌ | 复杂指令抓取 |

**本仓库推荐最小感知栈（Orin NX 16GB）：**

```
YOLOv8s（NITROS）+ AprilTag + Nvblox(0.05m)
延迟：< 15ms 目标检测，< 10ms/帧重建
功耗：~25W GPU 峰值
```

**完整感知栈（Orin AGX 64GB）：**

```
RT-DETR-L + FoundationPose + Nvblox(0.03m) + SigLIP + Grounding DINO（异步）
总延迟：< 50ms（感知到运动执行）
功耗：~50W GPU 峰值
```
