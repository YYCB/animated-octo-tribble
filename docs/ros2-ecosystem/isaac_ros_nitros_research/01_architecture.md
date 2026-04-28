# NITROS 分层架构详解

> 参考仓库：https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros  
> 参考版本：`release-3.2`  
> 关键源码：`isaac_ros_nitros/src/`、`isaac_ros_nitros/include/`

---

## 一、NITROS 的设计哲学

NITROS（NVIDIA Isaac Transport for ROS）不是替换 DDS，而是在 ROS 2 的**类型系统层面**做手术：

1. **REP-2007（Type Adaptation）**：允许发布/订阅双方使用"自定义类型"（如 CUDA buffer）而无需先转成 ROS 消息
2. **REP-2009（Type Negotiation）**：让发布/订阅双方在运行时协商使用哪种内存表示（CPU / CUDA / NvSci）
3. **NvSci（NVIDIA Science Buffer）**：NVIDIA 的跨进程、跨硬件内存共享框架，支持 GPU / VIC / DLA 共享同一块物理内存

NITROS 的本质是：**让 ROS 2 的 Topic 机制成为一个"内存句柄传递总线"，而不是"数据拷贝总线"**。

---

## 二、分层架构图

```
用户层（算法节点）
  NitrosNode（继承 rclcpp::Node）
  ├── NitrosPublisher<NitrosImage>
  │     └── rclcpp::Publisher<NitrosImage::BaseType>（TypeAdapter 包装）
  └── NitrosSubscriber<NitrosImage>
        └── rclcpp::Subscription<NitrosImage::BaseType>（TypeAdapter 包装）

NITROS 框架层
  ├── NitrosTypeManager        ← 注册/查找 NitrosType 的工厂
  ├── NegotiatedPublisher      ← Type Negotiation 实现（REP-2009）
  │     └── negotiated::NegotiatedPublisher（外部依赖）
  ├── NitrosContext            ← NvSciSync / NvSciBuf 上下文管理（RAII）
  └── NitrosStatistics         ← 延迟/丢帧统计

类型系统层
  ├── NitrosType<T>            ← 所有 NITROS 类型的模板基类
  │     ├── NitrosCpuType<T>  ← 数据在 CPU 内存
  │     ├── NitrosCudaType<T> ← 数据在 CUDA 显存
  │     └── NvSciType<T>      ← 数据在 NvSciBuf（跨进程）
  ├── NitrosImage              ← 对应 sensor_msgs::Image
  ├── NitrosCompressedImage    ← 对应 sensor_msgs::CompressedImage
  ├── NitrosCameraInfo         ← 对应 sensor_msgs::CameraInfo
  ├── NitrosPointCloud         ← 对应 sensor_msgs::PointCloud2
  ├── NitrosTensorList         ← 对应 isaac_ros_tensor_list_interfaces::TensorList
  └── （更多自定义类型）

底层依赖
  ├── NvSci（NvSciBuf + NvSciSync）← NVIDIA 跨进程内存共享
  ├── CUDA Runtime               ← GPU 内存管理
  └── GXF（Graph eXecution Framework）← NVIDIA 的计算图调度框架
```

---

## 三、关键包结构

### 3.1 `isaac_ros_nitros` 核心包

```
isaac_ros_nitros/
├── include/isaac_ros_nitros/
│   ├── nitros_node.hpp            ← 所有 NITROS 节点的基类
│   ├── nitros_publisher.hpp       ← NitrosPublisher 模板
│   ├── nitros_subscriber.hpp      ← NitrosSubscriber 模板
│   ├── nitros_type/
│   │   ├── nitros_type.hpp        ← NitrosType 基类
│   │   └── nitros_type_manager.hpp← 类型注册工厂
│   ├── types/
│   │   ├── nitros_image.hpp
│   │   ├── nitros_point_cloud.hpp
│   │   └── ...
│   └── utils/
│       ├── nitros_context.hpp     ← NvSci 上下文 RAII
│       └── nitros_statistics.hpp
├── src/
│   ├── nitros_node.cpp
│   ├── nitros_publisher.cpp
│   ├── nitros_subscriber.cpp
│   └── types/
│       ├── nitros_image.cpp       ← convert_to/from ROS message 实现
│       └── ...
└── CMakeLists.txt
```

### 3.2 `negotiated` 包（Type Negotiation 实现）

`negotiated` 是 ROS 2 生态中实现 REP-2009 的独立包：

```
negotiated/
├── include/negotiated/
│   ├── negotiated_publisher.hpp    ← 协商发布端
│   └── negotiated_subscription.hpp ← 协商订阅端
└── src/
    ├── negotiated_publisher.cpp    ← 协商协议实现
    └── negotiated_subscription.cpp
```

`NitrosPublisher` / `NitrosSubscriber` 组合了 `negotiated` 包和 `TypeAdapter`。

### 3.3 GXF（Graph eXecution Framework）

NITROS 的内部调度依赖 NVIDIA 的 **GXF**，这是 Isaac SDK 的计算图框架：

```
GXF
├── gxf_core        ← 计算图调度、实体/组件模型
├── gxf_std         ← 标准消息通道（MessageRouter、Vault）
├── gxf_cuda        ← CUDA 缓冲区、流（stream）管理
└── gxf_multimedia  ← 图像/视频 buffer（VideoBuffer）
```

NITROS 节点内部将 ROS 2 的 Callback 触发模型映射到 GXF 的 tick 机制，以实现对 GPU 流水线的精细控制。

---

## 四、`NitrosNode` 基类

```cpp
// isaac_ros_nitros/include/isaac_ros_nitros/nitros_node.hpp
class NitrosNode : public rclcpp::Node {
public:
  explicit NitrosNode(const rclcpp::NodeOptions & options,
                      const std::string & app_yaml_filename,    // GXF graph YAML
                      const NitrosPublisherSubscriberConfigMap & config_map,
                      const std::vector<std::string> & extension_spec_filenames,
                      const std::vector<std::string> & generator_rule_filenames,
                      const std::vector<std::pair<std::string,std::string>> & extensions,
                      const std::vector<std::string> & package_specs_yaml_filenames);

protected:
  // 子类可重写：在 GXF graph 启动前/后执行
  virtual void preLoadGraphCallback() {}
  virtual void postLoadGraphCallback() {}

  // 获取 NitrosPublisher/Subscriber 实例
  std::shared_ptr<NitrosPublisher> findNitrosPublisher(const GXFIOGroupPointerType & gxf_io_group);
  std::shared_ptr<NitrosSubscriber> findNitrosSubscriber(const GXFIOGroupPointerType & gxf_io_group);

private:
  // GXF Runtime（图执行引擎）
  std::shared_ptr<NitrosContext> nitros_context_;
  // 协商好的 Publisher/Subscriber 集合
  NitrosPublisherSubscriberGroupsType nitros_pub_sub_groups_;
};
```

**关键设计**：`NitrosNode` 在构造时加载 GXF 的计算图 YAML，将 GPU 算子（codelet）串联成流水线，ROS 2 的 publisher/subscriber 只是该流水线的输入/输出桩。

---

## 五、NITROS 与标准 rclcpp 节点的对比

| 特性 | 标准 rclcpp::Node | NitrosNode |
|------|------------------|-----------|
| 数据流驱动模型 | ROS 2 Callback（Executor） | GXF 计算图（tick） |
| GPU 数据传输 | 需手动 cudaMemcpy | 自动 GPU buffer 共享 |
| 跨节点延迟 | ~1ms（含序列化） | ~100µs（仅传句柄） |
| 兼容非 NITROS 节点 | 天然兼容 | 自动降级（有拷贝开销） |
| 配置方式 | 参数/launch | 参数/launch + GXF YAML |
| 学习曲线 | 低 | 高（需了解 GXF） |
| 硬件要求 | 任意 | Jetson AGX / x86 + dGPU |

---

## 六、Jetson 平台特化

NITROS 在 NVIDIA Jetson（AGX Orin / Orin NX / Xavier）上有额外优化：

| 优化 | 说明 |
|------|------|
| **iGPU 统一内存** | CPU 与 GPU 共享物理内存（UVA），真正零拷贝 |
| **NvSci 跨进程** | 通过 `/dev/nvscibuf` 设备在进程间共享 GPU 内存，避免 IPC 序列化 |
| **VIC（Video Image Compositor）** | 图像缩放/格式转换由专用硬件完成，不占 CUDA 核心 |
| **DLA（Deep Learning Accelerator）** | TensorRT 可将推理任务调度到 DLA，CUDA 核心专注其他任务 |
| **NVENC/NVDEC** | 硬件 H.264/H.265 编解码，`isaac_ros_h264_encoder` 直接调用 |

在 x86 + dGPU 平台，NvSci 跨进程共享不可用（PCIe 无统一内存），降级为 CUDA IPC（cudaIpcGetMemHandle）。
