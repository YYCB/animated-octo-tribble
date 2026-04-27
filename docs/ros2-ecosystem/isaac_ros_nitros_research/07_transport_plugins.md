# NITROS 与 image_transport / point_cloud_transport 的关系

> 相关包：`image_transport`、`point_cloud_transport`、`isaac_ros_nitros`

---

## 一、`image_transport` 回顾

`image_transport` 是 ROS 生态中**图像传输的抽象层**，允许发布端发布一次，订阅端通过不同插件（压缩、JPEG、Theora 等）接收：

```
发布端（单次 publish）
  image_transport::Publisher::publish(sensor_msgs::Image)
  ↓
image_transport 框架（根据已注册插件）
  ├── 原始 topic：/camera/image_raw (sensor_msgs::Image)
  ├── 压缩 topic：/camera/image_raw/compressed (CompressedImage, JPEG)
  ├── Theora：/camera/image_raw/theora (...)
  └── zstd（自定义插件）：/camera/image_raw/zstd (...)
```

订阅端选择最合适的 topic 订阅，发布端不需要知道有多少种消费方式。

---

## 二、NITROS 与 `image_transport` 的关系

### 2.1 两者的定位差异

| 特性 | `image_transport` | NITROS |
|------|------------------|--------|
| 解决的问题 | 图像**编码格式**多样性（压缩方案） | 图像**内存位置**多样性（CPU/GPU） |
| 数据流向 | 发布端 → 多个订阅端（fan-out） | 节点链路（pipeline，fan-in/fan-out） |
| GPU 感知 | ❌（只处理 CPU 内存） | ✅（核心特性） |
| 跨节点协商 | ❌（订阅端选择） | ✅（双方协商） |
| 与 DDS 关系 | 完全依赖 DDS | 绕过 DDS 核心数据路径 |
| rosbag2 支持 | ✅（记录压缩格式） | 降级（需 GPU→CPU 拷贝）|

### 2.2 在 NITROS 中使用 `image_transport`

NITROS 节点可以同时支持两种订阅方式：

```python
# 方式 1：NITROS 原生（GPU 零拷贝，推荐）
sub_nitros = node.create_subscription(
    NitrosImageAdapter, "/camera/image_raw", nitros_callback, 10)

# 方式 2：image_transport（CPU，兼容性好）
it = image_transport.create_subscription(
    node, "/camera/image_raw", cpu_callback, "raw")
    # 或 "compressed"，自动订阅压缩 topic
```

### 2.3 `isaac_ros_image_transport`

NVIDIA 提供了 `isaac_ros_image_transport` 包，将 NITROS 包装为 `image_transport` 插件：

```
发布端使用 image_transport 发布：
  image_transport::Publisher::publish(sensor_msgs::Image)
  ├── topic：/camera/image_raw（原始）
  └── topic：/camera/image_raw/nitros（NITROS 插件！）
              → 订阅端收到 NitrosImage（GPU buffer）
```

这样**不支持 NITROS API 的老代码（只用 image_transport）也能受益于 GPU 零拷贝**，只需安装 `isaac_ros_image_transport` 插件。

---

## 三、`point_cloud_transport` 与 NITROS

`point_cloud_transport` 的设计模式与 `image_transport` 完全相同，提供点云的多格式传输：

| 插件 | 格式 | 压缩率 |
|------|------|--------|
| `raw` | sensor_msgs::PointCloud2 | 1× |
| `draco` | Google Draco 压缩 | 5-20× |
| `zstd` | zstd 通用压缩 | 2-5× |

NITROS 对点云的处理类似图像：

```
LiDAR 驱动（CPU 点云）
  → NitrosSubscriber 接收（convert_from_ros_message：CPU→GPU）
  → GPU 上执行：去噪 / 降采样 / 地面滤除（pcl_gpu / thrust）
  → NitrosPublisher 发布 NitrosPointCloud
  → Nvblox 消费（GPU 零拷贝）
```

---

## 四、降级链路：NITROS 与传统节点的互联

当 NITROS 管线中夹着一个不支持 NITROS 的传统节点（如 RVIZ2、rosbag2、自定义 CPU 算法）：

```
全 NITROS 路径（理想）：
  cam_node ──[NitrosImage]──► dnn_node ──[NitrosTensorList]──► postprocess_node

插入传统节点（RVIZ2）：
  cam_node ──[NitrosImage]──► dnn_node
                │
                └──[sensor_msgs::Image]──► rviz2
                    （自动降级：Type Negotiation 检测到 rviz2 不支持 NitrosImage）
                    （NitrosPublisher 额外创建一个 sensor_msgs::Image publisher）
                    （cudaMemcpy D→H，每帧发生一次）
```

**建议**：将可视化、录制等"非实时路径"节点与 NITROS 推理链路分开部署在不同 Container 中，避免降级拷贝影响推理延迟。

---

## 五、与 `camera_calibration` / `camera_info_manager` 的集成

NITROS 完全兼容 ROS 2 标准的相机标定流程：

```
相机标定（一次性）：
  ros2 run camera_calibration cameracalibrator \
    --size 8x6 --square 0.025 \
    image:=/camera/image_raw camera:=/camera

  → 生成 camera_info.yaml
  → 由 camera_info_manager 加载
  → 通过 /camera_info (NitrosCameraInfo) topic 发布

NITROS 节点接收：
  NitrosSubscriber<NitrosCameraInfo> 接收标定参数
  → 直接在 GPU 上做去畸变（利用 CUDA texture memory 存储 K 矩阵）
```

---

## 六、NITROS 与 `camera_info` 的时间同步

在多相机系统（如双目、RGB-D+LiDAR）中，时间同步至关重要：

```cpp
// NITROS 支持通过 message_filters 进行 Topic 同步
// 但推荐使用 GXF 内部的时间戳同步（延迟更低）

// GXF TimestampSynchronizer（精确到微秒级）
- name: sync
  type: nvidia::gxf::Synchronization
  parameters:
    inputs: [left_image_receiver, right_image_receiver]
    max_delay_ns: 5000000   # 5ms 内的帧视为同步
```

对于 NITROS 节点外部（与传统 ROS 2 节点交互）的时间同步，仍使用标准 `message_filters::TimeSynchronizer`，但此时数据已在 CPU 内存中（降级路径）。
