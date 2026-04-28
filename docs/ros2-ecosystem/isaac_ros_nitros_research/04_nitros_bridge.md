# NITROS 桥接节点实现 — GPU Buffer 在 ROS 2 Topic 上的流转

> 源码：`isaac_ros_nitros/src/nitros_publisher.cpp`、`nitros_subscriber.cpp`  
> 相关：`isaac_ros_nitros/src/nitros_node.cpp`（GXF 集成）

---

## 一、NITROS 桥接节点的角色

"NITROS 桥接节点"（`NitrosNode`）是连接 **GXF 计算图**（GPU 流水线）与 **ROS 2 Topic 系统**的桥梁：

```
ROS 2 Topic 世界                          GXF 计算图世界
  （标准 pub/sub）                          （GPU 算子流水线）

  /image_raw ──────────────────────────► GXF Codelet A (预处理)
                  NitrosSubscriber            │
                  (ROS→GXF 桥)               ▼
                                         GXF Codelet B (TensorRT)
  /detections ◄───────────────────────────── │
                  NitrosPublisher        GXF Codelet C (后处理)
                  (GXF→ROS 桥)
```

GXF 负责内部 GPU 数据流转（零拷贝），NitrosPublisher/Subscriber 负责在 ROS 2 边界上处理格式转换（必要时）。

---

## 二、`NitrosPublisher` 详解

### 2.1 类结构

```cpp
// isaac_ros_nitros/include/isaac_ros_nitros/nitros_publisher.hpp
class NitrosPublisher {
public:
  NitrosPublisher(
    rclcpp::Node & node,
    const NitrosPublisherSubscriberConfig & config,  // topic 名、帧率等
    const std::vector<std::string> & supported_data_formats, // 支持的类型列表
    std::shared_ptr<NitrosTypeManager> type_manager);

  // 从 GXF entity（GPU buffer）发布
  void publish(const gxf::Entity & entity);

  // 供 TypeAdapter 调用的内部接口
  void publishNitrosType(const NitrosTypeBase & nitros_msg);
  void publishRosMessage(const rclcpp::SerializedMessage & serialized_msg);

private:
  // 协商发布端（包含多个底层 rclcpp::Publisher）
  std::shared_ptr<negotiated::NegotiatedPublisher> negotiated_pub_;

  // 统计
  std::shared_ptr<NitrosStatistics> statistics_;

  // GXF 图访问句柄
  gxf::Handle<gxf::Vault> vault_;  // GXF 消息出口
};
```

### 2.2 `publish()` 内部流程

```
NitrosPublisher::publish(gxf::Entity entity)
  ├── 提取 entity 中的元数据（timestamp、frame_id）
  ├── 检查协商结果
  │     ├── 若某订阅端需要 NitrosCudaType：
  │     │     → 直接从 entity 获取 CUDA buffer 句柄
  │     │     → 打包成 NitrosImage（仅元数据，~100 字节）
  │     │     → 通过 NitrosCudaType publisher 发布
  │     │
  │     └── 若某订阅端需要 sensor_msgs::Image：
  │           → TypeAdapter::convert_to_ros_message()
  │               → cudaMemcpy Device→Host（! 此处有拷贝 ！）
  │           → 通过 sensor_msgs::Image publisher 发布
  │
  └── 更新统计（发布帧数、延迟分布）
```

### 2.3 GXF Vault 与 ROS 2 的同步

GXF 的计算图是 **pull-based**（算子主动 tick），而 ROS 2 callback 是 **push-based**（消息到达时触发）。NITROS 通过 **GXF Vault** 解决这个阻抗匹配问题：

```
GXF 计算图的最后一个 Codelet
  → 将输出 Entity 存入 gxf::Vault（类似队列）

NITROS 轮询线程（或 GXF Scheduler 触发）
  → 从 Vault 取出 Entity
  → 调用 NitrosPublisher::publish(entity)
  → 进入 ROS 2 发布路径
```

对于输入方向（NitrosSubscriber → GXF）：
```
ROS 2 Topic 消息到达
  → NitrosSubscriber callback
  → 若是 NitrosType：直接包装为 GXF Entity（零拷贝）
  → 若是 sensor_msgs：TypeAdapter::convert_from_ros_message（有 CPU→GPU 拷贝）
  → 注入 GXF Receiver（类似入队）
  → 触发下游 GXF Codelet tick
```

---

## 三、`NitrosSubscriber` 详解

```cpp
// 简化结构
class NitrosSubscriber {
  // 协商订阅端（包含多个底层 rclcpp::Subscription）
  std::shared_ptr<negotiated::NegotiatedSubscription> negotiated_sub_;

  // GXF 图访问：把收到的消息注入 GXF
  gxf::Handle<gxf::Receiver> receiver_;

  // 用户回调（可选，也可由 GXF 直接处理）
  std::function<void(gxf::Entity)> on_receive_callback_;
};
```

NitrosSubscriber 在收到消息时的处理逻辑：

```
收到 NitrosCudaImage 消息（零拷贝路径）：
  → 从消息中提取 CUDA buffer 指针（或 NvSci handle）
  → 包装为 gxf::VideoBuffer（仅包装，不拷贝）
  → 创建 gxf::Entity 并附加 VideoBuffer
  → 注入 GXF Receiver
  → 调用用户回调（如果注册了的话）

收到 sensor_msgs::Image（降级路径）：
  → TypeAdapter::convert_from_ros_message()
      → 分配 CUDA 显存
      → cudaMemcpy Host→Device
  → 包装为 gxf::VideoBuffer
  → 注入 GXF Receiver
```

---

## 四、GXF Codelet — GPU 算子的基本单元

GXF 中，每个处理步骤是一个 **Codelet**，类似 ROS 2 的 Node：

```cpp
// GXF Codelet 示例（TensorRT 推理）
class TensorRTInferenceCodelet : public gxf::Codelet {
  gxf::Parameter<gxf::Handle<gxf::Receiver>> input_tensor_;    // 输入
  gxf::Parameter<gxf::Handle<gxf::Transmitter>> output_tensor_; // 输出
  gxf::Parameter<std::string> model_file_path_;

  gxf_result_t start() override {
    // 加载 TensorRT engine
    engine_ = loadTensorRTEngine(model_file_path_);
    return GXF_SUCCESS;
  }

  gxf_result_t tick() override {
    // 从 Receiver 取输入（GPU buffer，零拷贝）
    auto input = input_tensor_.get()->receive().value();
    auto tensor = input.get<gxf::Tensor>().value();

    // TensorRT 推理（全在 GPU 上）
    engine_->infer(tensor->pointer(), output_buffer_);

    // 发送输出（GPU buffer，零拷贝）
    gxf::Entity output_entity = gxf::Entity::New(context()).value();
    output_entity.add<gxf::Tensor>("output_tensor", output_buffer_);
    output_tensor_.get()->publish(output_entity);
    return GXF_SUCCESS;
  }
};
```

NITROS 节点是 **GXF Codelet + ROS 2 Publisher/Subscriber 的混合体**，在计算图的边界上完成格式协商和（必要时的）转换。

---

## 五、NITROS 节点的 YAML 配置

每个 NitrosNode 子类通常带有一个 GXF 计算图 YAML：

```yaml
# 示例：isaac_ros_apriltag/config/apriltag_graph.yaml
---
name: apriltag_graph
components:
  - name: input_image
    type: nvidia::gxf::DoubleBufferReceiver  # 输入队列
    
  - name: apriltag_detector
    type: nvidia::isaac_ros::apriltag::AprilTagDetector
    parameters:
      receiver: input_image
      transmitter: output_detections
      max_tags: 64
      
  - name: output_detections  
    type: nvidia::gxf::DoubleBufferTransmitter # 输出队列

  - name: ros_input_bridge
    type: nvidia::isaac_ros::nitros::NitrosImageSubscriberBridge
    parameters:
      receiver: input_image
      topic_name: image_rect
      
  - name: ros_output_bridge
    type: nvidia::isaac_ros::nitros::AprilTagDetectionArrayPublisherBridge
    parameters:
      transmitter: output_detections
      topic_name: tag_detections
```

---

## 六、NITROS 节点的典型使用模式

### 6.1 直接使用官方 NITROS 节点

```python
# launch/apriltag.launch.py
from launch_ros.actions import ComposableNode, ComposableNodeContainer

container = ComposableNodeContainer(
    name='nitros_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container_mt',  # 多线程 Container
    composable_node_descriptions=[
        # 相机驱动（需支持 NitrosImage 输出）
        ComposableNode(
            package='isaac_ros_realsense',
            plugin='nvidia::isaac_ros::realsense::RealSenseNode',
            name='realsense',
        ),
        # AprilTag 检测（全 GPU 路径）
        ComposableNode(
            package='isaac_ros_apriltag',
            plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
            name='apriltag',
            parameters=[{'max_tags': 64}]
        ),
    ]
)
```

### 6.2 将自定义 CUDA 算法接入 NITROS

```cpp
// 继承 NitrosNode，在 GXF 图中插入自定义 CUDA 算子
class MyProcessingNode : public nvidia::isaac_ros::nitros::NitrosNode {
public:
  MyProcessingNode(const rclcpp::NodeOptions & options)
    : NitrosNode(
        options,
        "my_graph.yaml",          // GXF 计算图
        CREATE_NITROS_CONFIG_MAP(  // 输入输出映射
          {"input_image", NitrosImageCudaConfig},
          {"output_image", NitrosImageCudaConfig}),
        {}) {}

  void postLoadGraphCallback() override {
    // 在图加载后设置自定义参数
    setGraphConfig("my_codelet", "threshold", 0.5);
  }
};

RCLCPP_COMPONENTS_REGISTER_NODE(MyProcessingNode)
```
