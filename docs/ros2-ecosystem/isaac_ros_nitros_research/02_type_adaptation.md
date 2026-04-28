# REP-2007 与 `rclcpp::TypeAdapter` — Type Adaptation 源码级剖析

> REP-2007 原文：https://ros.org/reps/rep-2007.html  
> 参考源码：`rclcpp/include/rclcpp/type_adapter.hpp`  
> Isaac ROS 实现：`isaac_ros_nitros/include/isaac_ros_nitros/types/`

---

## 一、Type Adaptation 解决的问题

在 REP-2007 之前，ROS 2 发布端的数据流：

```
用户数据（如 cv::Mat 或 CUDA buffer）
  → 手动转换为 sensor_msgs::Image
  → publisher->publish(ros_msg)  ← 内部再拷贝一次用于序列化
```

REP-2007 之后：

```
用户数据（cv::Mat / CUDA buffer）
  → publisher->publish(user_data)  ← 直接发布，TypeAdapter 在需要时才转换
```

关键洞察：**在同一进程内的两个节点通过 IntraProcess 通信时，根本不需要序列化，TypeAdapter 让编译器知道这一点并直接传递用户类型。**

---

## 二、`rclcpp::TypeAdapter` 模板

### 2.1 基础定义

```cpp
// rclcpp/include/rclcpp/type_adapter.hpp
template<typename CustomType, typename RosMessageType = void>
struct TypeAdapter {
  using is_specialized = std::false_type;  // 默认未特化
  using custom_type = CustomType;
  using ros_message_type = RosMessageType;
};
```

使用者需要对这个模板做**完全特化**，实现两个转换函数：

```cpp
template<>
struct rclcpp::TypeAdapter<MyCustomType, sensor_msgs::msg::Image> {
  using is_specialized = std::true_type;
  using custom_type = MyCustomType;
  using ros_message_type = sensor_msgs::msg::Image;

  // 自定义类型 → ROS 消息（仅在跨进程通信或与非 TypeAdapter 节点通信时调用）
  static void convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    // 例如：CUDA buffer → sensor_msgs::Image（涉及 cudaMemcpy）
    cudaMemcpy(destination.data.data(), source.cuda_ptr, source.size, cudaMemcpyDeviceToHost);
    destination.width = source.width;
    destination.height = source.height;
    destination.encoding = source.encoding;
  }

  // ROS 消息 → 自定义类型（接收来自非 TypeAdapter 节点的消息时调用）
  static void convert_from_ros_message(
    const ros_message_type & source,
    custom_type & destination)
  {
    // 例如：sensor_msgs::Image → CUDA buffer（涉及 cudaMemcpy H→D）
    cudaMalloc(&destination.cuda_ptr, source.data.size());
    cudaMemcpy(destination.cuda_ptr, source.data.data(), source.data.size(), cudaMemcpyHostToDevice);
    destination.width = source.width;
    destination.height = source.height;
  }
};
```

### 2.2 使用方式（create_publisher / create_subscription）

```cpp
// 用 RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE 宏简化声明（可选）
using MyImageAdapter = rclcpp::TypeAdapter<MyCustomType, sensor_msgs::msg::Image>;

// 发布端：直接发布 MyCustomType，无需先转 sensor_msgs::Image
auto pub = node->create_publisher<MyImageAdapter>("image", 10);
MyCustomType my_img = allocate_cuda_buffer(width, height);
pub->publish(my_img);  // ← 内部：若是 IntraProcess 则零拷贝传递 MyCustomType
                       //          若是跨进程则调用 convert_to_ros_message()

// 订阅端：callback 直接接收 MyCustomType
auto sub = node->create_subscription<MyImageAdapter>(
  "image", 10,
  [](const MyCustomType & img) {
    // 直接使用 CUDA buffer，无 CPU 拷贝
    launch_cuda_kernel(img.cuda_ptr, img.width, img.height);
  });
```

---

## 三、`rclcpp` 内部如何处理 TypeAdapter

### 3.1 Publisher 内部路径

```cpp
// rclcpp/src/rclcpp/publisher_base.cpp（简化）
template<typename MessageT, typename AllocatorT>
void Publisher<MessageT, AllocatorT>::publish(const MessageT & msg) {
  if constexpr (rclcpp::TypeAdapter<MessageT>::is_specialized::value) {
    // MessageT 是 custom type，需要决定是否转换
    if (has_intra_process_connections_) {
      // IntraProcess：直接传递 custom type，不转换
      intra_process_manager_->do_intra_process_publish_and_return_shared(
        intra_process_publisher_id_, msg, allocator_);
    }
    if (has_inter_process_connections_) {
      // 跨进程：必须转成 ROS 消息才能序列化
      typename TypeAdapter::ros_message_type ros_msg;
      TypeAdapter::convert_to_ros_message(msg, ros_msg);
      rcl_publish(&publisher_handle_, &ros_msg, nullptr);
    }
  } else {
    // 标准路径
    rcl_publish(&publisher_handle_, &msg, nullptr);
  }
}
```

**关键路径优化**：
- **同进程（IntraProcess）+ TypeAdapter**：完全零拷贝，直接传 `shared_ptr<CustomType>`
- **跨进程 + TypeAdapter**：调用一次 `convert_to_ros_message`，然后走标准 DDS 路径
- **跨进程 + LoanedMessage + TypeAdapter**：先借出 DDS 内存，直接序列化到 loaned buffer（省去中间拷贝）

### 3.2 Subscription 内部路径

```cpp
// rclcpp 订阅端（简化）
void do_inter_process_receive(const rmw_message_info_t & info, const void * ros_msg_ptr) {
  if constexpr (TypeAdapter::is_specialized::value) {
    // 转换 ROS 消息到 custom type
    typename TypeAdapter::custom_type custom_msg;
    TypeAdapter::convert_from_ros_message(
      *static_cast<const typename TypeAdapter::ros_message_type *>(ros_msg_ptr),
      custom_msg);
    user_callback_(custom_msg);
  } else {
    user_callback_(*static_cast<const MessageT *>(ros_msg_ptr));
  }
}
```

---

## 四、NITROS 的 TypeAdapter 实现

以 `NitrosImage` 为例（对应 `sensor_msgs::Image`）：

### 4.1 `NitrosImage` 数据结构

```cpp
// isaac_ros_nitros/include/isaac_ros_nitros/types/nitros_image.hpp
struct NitrosImageView {
  // GPU buffer（通过 NvSci 或 CUDA IPC 共享）
  gxf::Handle<gxf::VideoBuffer> video_buffer;
  // 元数据
  uint32_t width;
  uint32_t height;
  std::string encoding;             // "rgb8" / "bgr8" / "mono8" / "nv12"
  builtin_interfaces::msg::Time stamp;
  std::string frame_id;
};

// NitrosType 包装（含协商格式信息）
using NitrosImage = NitrosTypeBase<NitrosImageView>;
```

### 4.2 TypeAdapter 特化

```cpp
// isaac_ros_nitros/src/types/nitros_image.cpp
template<>
struct rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosImage,
  sensor_msgs::msg::Image>
{
  using is_specialized = std::true_type;
  using custom_type = NitrosImage;
  using ros_message_type = sensor_msgs::msg::Image;

  static void convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    // GPU → CPU（降级路径，会发生 cudaMemcpy）
    RCLCPP_DEBUG(..., "Converting NitrosImage to sensor_msgs::Image (GPU→CPU copy)");
    auto video_buffer = source.GetHandle<gxf::VideoBuffer>();
    destination.height = video_buffer->video_frame_info().height;
    destination.width  = video_buffer->video_frame_info().width;
    destination.encoding = NitrosImageEncodingToRosEncoding(
      video_buffer->video_frame_info().color_format);
    destination.step = destination.width * get_bytes_per_pixel(destination.encoding);
    destination.data.resize(destination.step * destination.height);
    // 关键：GPU → CPU 拷贝
    cudaMemcpy(
      destination.data.data(),
      video_buffer->pointer(),        // CUDA device pointer
      destination.data.size(),
      cudaMemcpyDeviceToHost);
  }

  static void convert_from_ros_message(
    const ros_message_type & source,
    custom_type & destination)
  {
    // CPU → GPU（升级路径）
    auto video_buffer = allocate_gxf_video_buffer(source.width, source.height, source.encoding);
    cudaMemcpy(
      video_buffer->pointer(),
      source.data.data(),
      source.data.size(),
      cudaMemcpyHostToDevice);
    destination.SetHandle(video_buffer);
    destination.header.stamp = source.header.stamp;
    destination.header.frame_id = source.header.frame_id;
  }
};
```

**关键点**：`convert_to_ros_message` 和 `convert_from_ros_message` **只在必须与非 NITROS 节点通信时才被调用**。在全 NITROS 节点链路中，这两个函数永远不会被调用。

---

## 五、TypeAdapter 的使用限制与注意事项

| 限制 | 说明 |
|------|------|
| **IntraProcess 才能零拷贝** | 跨进程必须转 ROS 消息；若需跨进程零拷贝，需 NvSci（见 03_type_negotiation.md）|
| **回调参数类型** | 订阅回调接收 `const CustomType &` 或 `shared_ptr<const CustomType>` |
| **QoS 兼容性** | TypeAdapter 不改变 QoS 语义，Deadline/Liveliness 正常工作 |
| **录包（rosbag2）** | rosbag2 只能录 ROS 消息，会触发 `convert_to_ros_message`（有拷贝） |
| **跨 DDS 域** | 跨 ROS_DOMAIN_ID 仍走 DDS，无法享受 TypeAdapter 零拷贝 |

---

## 六、TypeAdapter vs LoanedMessage（对比）

| 特性 | LoanedMessage | TypeAdapter |
|------|--------------|------------|
| 解决的问题 | 避免发布端额外拷贝到 DDS 内存 | 允许使用非 ROS 类型（如 CUDA buffer） |
| 适用场景 | CPU 大消息（点云、图像）零拷贝发布 | GPU buffer 直接在节点间传递 |
| 跨进程支持 | ✅（DDS SHM 或 Loaned DDS buffer） | ✅（但需转换为 ROS 消息） |
| GPU 感知 | ❌ | ✅ |
| 组合使用 | TypeAdapter + LoanedMessage 可以组合 | — |
| rclcpp 版本 | Foxy+ | Humble+（REP-2007 正式实现） |
