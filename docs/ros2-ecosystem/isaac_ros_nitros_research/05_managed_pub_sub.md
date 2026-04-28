# NitrosPublisher/Subscriber 与 NITROS 类型系统

> 源码：`isaac_ros_nitros_interfaces/`、`isaac_ros_nitros/include/isaac_ros_nitros/types/`

---

## 一、NITROS 类型层次

```
NitrosTypeBase                          ← 所有 NITROS 类型的公共基类
  │  包含：
  │  - gxf::Entity 句柄（指向 GXF 内部的 GPU buffer）
  │  - std_msgs::msg::Header（timestamp + frame_id）
  │  - 协商格式标识符（string）
  │
  ├── NitrosImage
  │     对应 ROS：sensor_msgs::msg::Image
  │     GXF 内部：gxf::VideoBuffer
  │     支持格式：rgb8 / bgr8 / mono8 / mono16 / nv12 / nv24 / rgba8888
  │
  ├── NitrosCompressedImage
  │     对应 ROS：sensor_msgs::msg::CompressedImage
  │     GXF 内部：gxf::VideoBuffer（压缩格式）
  │
  ├── NitrosCameraInfo
  │     对应 ROS：sensor_msgs::msg::CameraInfo
  │     GXF 内部：gxf::CameraModel
  │
  ├── NitrosPointCloud
  │     对应 ROS：sensor_msgs::msg::PointCloud2
  │     GXF 内部：gxf::Tensor（N×4 float32：x,y,z,intensity）
  │
  ├── NitrosTensorList
  │     对应 ROS：isaac_ros_tensor_list_interfaces::msg::TensorList
  │     GXF 内部：vector<gxf::Tensor>（任意形状 CUDA tensor）
  │     用途：DNN 输入/输出、VLA 动作 token
  │
  ├── NitrosDisparityImage
  │     对应 ROS：stereo_msgs::msg::DisparityImage
  │     GXF 内部：gxf::VideoBuffer（float32 视差图）
  │
  ├── NitrosOccupancyGrid
  │     对应 ROS：nav_msgs::msg::OccupancyGrid
  │     GXF 内部：gxf::Tensor（2D int8）
  │
  └── NitrosDetection2DArray
        对应 ROS：vision_msgs::msg::Detection2DArray
        GXF 内部：gxf::Tensor（检测框列表）
```

---

## 二、`NitrosTypeBase` 数据结构

```cpp
// isaac_ros_nitros/include/isaac_ros_nitros/types/nitros_type_base.hpp
class NitrosTypeBase {
public:
  // GXF Entity 句柄（持有实际 GPU buffer 的引用计数对象）
  gxf::Entity handle{};

  // 数据格式标识（与协商协议匹配）
  std::string data_format_name{};  // e.g., "nitros_image_bgr8"

  // ROS header（时间戳 + 坐标系）
  builtin_interfaces::msg::Time stamp{};
  std::string frame_id{};

  // 统计信息（可选）
  int64_t receiver_tag{-1};   // 接收延迟追踪
  int64_t publisher_tag{-1};

  // GXF 上下文（用于创建/销毁 Entity）
  std::shared_ptr<NitrosContext> nitros_context_{};
};
```

### 关键点：NitrosTypeBase 是**轻量包装**

- `handle`（gxf::Entity）内部通过引用计数管理 GPU 内存的生命周期
- 传递 `NitrosTypeBase` 对象本质上是传递一个**智能指针**，GPU buffer 不移动
- 两个进程通过 `NvSciBufObjIpcExport/Import` 各自持有指向同一物理内存的句柄

---

## 三、`NitrosImage` 详解

### 3.1 数据格式（`data_format_name`）

NITROS 定义了一套标准化的格式字符串，用于协商：

| 格式名 | 说明 | 对应 ROS encoding |
|--------|------|-------------------|
| `nitros_image_rgb8` | CPU / GPU RGB 8bit | `rgb8` |
| `nitros_image_bgr8` | CPU / GPU BGR 8bit | `bgr8` |
| `nitros_image_mono8` | 灰度 8bit | `mono8` |
| `nitros_image_mono16` | 灰度 16bit | `mono16` |
| `nitros_image_nv12` | YUV420 NV12（Jetson 相机原生输出） | — |
| `nitros_image_rgb32f` | RGB float32（DNN 输入常用） | `32FC3` |
| `nitros_image_depth32f` | 深度图 float32 | `32FC1` |

### 3.2 访问 GPU buffer

```cpp
// 在 NitrosSubscriber 回调或 GXF Codelet 中访问 NitrosImage
void processNitrosImage(const nvidia::isaac_ros::nitros::NitrosImage & nitros_img) {
  // 获取 GXF VideoBuffer
  auto video_buffer_handle = nitros_img.handle.get<gxf::VideoBuffer>("video_buffer");
  auto * video_buffer = video_buffer_handle.value().get();

  // 获取 CUDA device pointer
  void * cuda_ptr = video_buffer->pointer();
  uint32_t width  = video_buffer->video_frame_info().width;
  uint32_t height = video_buffer->video_frame_info().height;
  size_t stride   = video_buffer->video_frame_info().color_planes[0].stride;

  // 直接调用 CUDA kernel
  myProcessingKernel<<<grid, block, 0, cuda_stream>>>(
    static_cast<uint8_t*>(cuda_ptr), width, height, stride);
}
```

---

## 四、`NitrosTensorList` — DNN 推理的核心类型

`NitrosTensorList` 是 Isaac ROS 中**最重要的类型之一**，用于 DNN 模型的输入/输出：

### 4.1 数据结构

```cpp
// isaac_ros_tensor_list_interfaces/msg/TensorList.msg
TensorList:
  Tensor[] tensors

// isaac_ros_tensor_list_interfaces/msg/Tensor.msg
Tensor:
  string name          # 张量名称（对应模型 I/O binding）
  int32[] shape        # 形状，如 [1, 3, 480, 640]
  string data_type     # "float32" / "int8" / "float16"
  uint8[] data         # CPU 模式下的实际数据
```

在 NITROS 路径下，`data` 字段为空，实际数据在 `handle` 的 GXF Tensor 中（CUDA 显存）。

### 4.2 使用示例（TensorRT 推理节点输出）

```python
# 订阅 TensorRT 推理结果
def tensor_callback(tensor_list: TensorList):
    # 在 CPU 路径下使用
    output_tensor = tensor_list.tensors[0]
    scores = np.frombuffer(output_tensor.data, dtype=np.float32)
    scores = scores.reshape(output_tensor.shape)
    # 处理检测结果...
```

```cpp
// 在 NITROS 路径下（GPU 直接处理）
void tensorCallback(const NitrosTensorList & nitros_tensor_list) {
  auto tensor_handle = nitros_tensor_list.handle.get<gxf::Tensor>("output_tensor_0");
  float * scores_cuda = static_cast<float*>(tensor_handle.value()->pointer());
  int num_detections = tensor_handle.value()->shape().dimension(1);
  // 在 GPU 上直接做 NMS（Non-Maximum Suppression）
  nms_kernel<<<grid, block>>>(scores_cuda, num_detections, nms_threshold_);
}
```

---

## 五、`NitrosPointCloud` — 激光雷达 / 深度相机点云

### 5.1 格式

```
GXF 内部格式：gxf::Tensor，形状 [N, 4]，dtype float32
  每行：[x, y, z, intensity]（或 [x, y, z, ring]）
对应：sensor_msgs::msg::PointCloud2（XYZI 格式）
```

### 5.2 Nvblox 集成

`isaac_ros_nvblox` 直接消费 `NitrosDepthImage` 和 `NitrosCameraInfo`，**全程 GPU**：

```
RealSense 深度图（NitrosDepthImage）+ CameraInfo
  → Nvblox Integrator（GPU TSDF 更新）
  → 实时 3D ESDF 地图（GPU voxel hash）
  → Mesh 输出（用于可视化）
  → Slice Map 输出（用于 Nav2 cost map）
```

---

## 六、NitrosPublisher/Subscriber 配置参数

```cpp
// NitrosPublisherSubscriberConfig 结构
struct NitrosPublisherSubscriberConfig {
  std::string topic_name;            // ROS topic 名称
  std::string frame_id;              // TF frame
  NitrosPublisherSubscriberType type;// INPUT 或 OUTPUT

  // 协商参数
  double expected_fps{0.0};          // 0 = 不强制帧率
  bool use_compatible_format_only{false}; // true = 跳过协商，强制使用 ROS 格式

  // QoS
  rclcpp::QoS qos_profile{rclcpp::SensorDataQoS()};

  // 回调
  std::function<void(const gxf::Entity &)> callback{nullptr};
};
```

关键参数 `use_compatible_format_only`：
- `true`：强制走 CPU 路径（sensor_msgs），用于调试或与传统节点通信
- `false`（默认）：优先协商 GPU 格式

---

## 七、NITROS 类型的 rosbag2 录制与回放

由于 rosbag2 只能处理 ROS 消息，录制 NITROS 话题时会自动降级：

```
录制时：NitrosPublisher → TypeAdapter::convert_to_ros_message → rosbag2
         （GPU→CPU 拷贝，按帧率持续发生）

回放时：rosbag2 发布 sensor_msgs → NitrosSubscriber 接收
       → TypeAdapter::convert_from_ros_message（CPU→GPU 拷贝）
       → GXF 处理
```

**建议**：若需记录原始 GPU 数据用于训练，考虑使用专用数据录制格式（如 mcap + NvSci handle 序列化），避免 CPU 降级带来的帧率限制。
