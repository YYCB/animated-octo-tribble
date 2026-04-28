# REP-2009 与 Type Negotiation — 运行时内存格式协商

> REP-2009 原文：https://ros.org/reps/rep-2009.html  
> 参考实现：`negotiated` 包（https://github.com/ros-ros2/negotiated）  
> NITROS 实现：`isaac_ros_nitros/src/nitros_publisher.cpp`、`nitros_subscriber.cpp`

---

## 一、Type Negotiation 解决的问题

REP-2007（Type Adaptation）解决了"可以用自定义类型"，但没有解决"**用哪种表示最优**"的问题：

```
场景：发布端是 Jetson 相机驱动（数据在 NV12 GPU buffer）
      订阅端 A：Isaac ROS DNN 节点（需要 CUDA RGB8 buffer）
      订阅端 B：RVIZ2 可视化（需要 CPU RGB8 sensor_msgs::Image）
      订阅端 C：rosbag2 录包（需要 CPU sensor_msgs::Image）
```

没有协商机制时，发布端不知道应该发哪种格式，可能需要多次转换。  
**REP-2009 允许发布端和所有订阅端在连接建立时协商出一种"代价最小"的格式。**

---

## 二、协商协议原理

### 2.1 参与方

- **NegotiatedPublisher**：维护支持的类型列表（按偏好排序）
- **NegotiatedSubscription**：维护支持的类型列表（按偏好排序）
- **协商 Topic**（内部，自动创建）：用于双方交换支持类型列表

### 2.2 协商过程（5步）

```
Step 1 - 发布端广播支持类型
  NegotiatedPublisher 在隐藏 topic（`<base_topic>/_negotiated`）发布：
  {
    supported_types: [
      {type: "NitrosCudaImage", priority: 1.0},     // 最偏好（GPU直传）
      {type: "NitrosCpuImage",  priority: 0.5},     // 次选
      {type: "sensor_msgs/Image", priority: 0.1},   // 最差（标准 ROS 消息）
    ]
  }

Step 2 - 订阅端发布自己支持的类型
  每个 NegotiatedSubscription 在另一个隐藏 topic 发布：
  订阅 A（DNN 节点）：[NitrosCudaImage(1.0), NitrosCpuImage(0.3)]
  订阅 B（RVIZ2）：   [sensor_msgs/Image(1.0)]
  订阅 C（rosbag2）：  [sensor_msgs/Image(1.0)]

Step 3 - 发布端计算最优类型集合
  对每个订阅端，选择发布端与订阅端支持类型的"最高分交集"：
  结果：
    → 订阅 A 使用 NitrosCudaImage（双方都支持，GPU 零拷贝）
    → 订阅 B 使用 sensor_msgs/Image（RVIZ2 不支持 NITROS）
    → 订阅 C 使用 sensor_msgs/Image

Step 4 - 发布端创建多个底层发布者
  NegotiatedPublisher 为每种实际使用的类型各创建一个 rclcpp::Publisher：
    rclcpp::Publisher<NitrosCudaImage>
    rclcpp::Publisher<sensor_msgs::Image>

Step 5 - 发布时广播（按需转换）
  publish(cuda_image):
    → 直接发布到 NitrosCudaImage publisher（订阅 A）
    → 调用 convert_to_ros_message() → 发布到 sensor_msgs publisher（订阅 B/C）
```

### 2.3 协商失败处理

若协商找不到任何公共类型，发布端可以选择：
- 只发布自己支持的最高优先级类型（订阅端自行处理不匹配）
- 回落到 `sensor_msgs::Image` 基准类型（所有节点都支持）

---

## 三、`negotiated` 包的 C++ API

```cpp
// 引入
#include "negotiated/negotiated_publisher.hpp"
#include "negotiated/negotiated_subscription.hpp"

// 发布端：注册支持的类型
auto neg_pub = std::make_shared<negotiated::NegotiatedPublisher>(
  *node, "image_raw");

// 添加支持的类型（优先级 0.0~1.0，越高越偏好）
neg_pub->add_supported_type<NitrosCudaImageAdapter>(
  1.0,  // priority
  rclcpp::QoS(10));

neg_pub->add_supported_type<sensor_msgs::msg::Image>(
  0.1,
  rclcpp::QoS(10));

// 启动协商（等待订阅端注册）
neg_pub->start();

// 发布（内部自动路由到正确的底层 publisher）
neg_pub->publish<NitrosCudaImageAdapter>(cuda_image);

// ──────────────────────────────────────────────────────

// 订阅端：注册支持的类型
auto neg_sub = std::make_shared<negotiated::NegotiatedSubscription>(
  *node, "image_raw");

neg_sub->add_supported_callback<NitrosCudaImageAdapter>(
  1.0,
  rclcpp::QoS(10),
  [](const NitrosCudaImage & img) {
    // 处理 GPU 图像
  });

neg_sub->add_supported_callback<sensor_msgs::msg::Image>(
  0.5,
  rclcpp::QoS(10),
  [](const sensor_msgs::msg::Image & img) {
    // 降级处理 CPU 图像
  });

neg_sub->start();
```

---

## 四、NvSci（NVIDIA Science Buffer）—— 跨进程 GPU 零拷贝

### 4.1 为什么需要 NvSci

CUDA IPC（`cudaIpcGetMemHandle`）只能在**同一 GPU** 的不同进程间共享显存，且要求进程在同一用户下。

**NvSci** 是 NVIDIA 提供的更完整跨进程内存共享框架：

| 特性 | CUDA IPC | NvSci |
|------|----------|-------|
| 跨进程 GPU 内存共享 | ✅ | ✅ |
| 跨硬件（GPU/DLA/VIC）| ❌ | ✅ |
| 跨不同用户进程 | ❌ | ✅（通过 /dev/nvscibuf） |
| 同步原语 | CUDA Event | NvSciSync |
| 内存对齐约束 | 手动 | 自动（根据使用者协商） |
| Jetson 专属 | 否 | 是（完整功能） |

### 4.2 `NvSciBuf` 分配流程

```cpp
// NITROS 内部（简化）
NvSciBufModule buf_module;
NvSciBufModuleOpen(&buf_module);

// 创建分配请求（列出访问者需求）
NvSciBufAttrList attr_list;
NvSciBufAttrListCreate(buf_module, &attr_list);

// 设置需求（图像格式、访问者列表）
NvSciBufAttrValAccessPerm perm = NvSciBufAccessPerm_ReadWrite;
NvSciBufAttrListSetAttrs(attr_list, ...);

// 分配：NvSci 自动选择满足所有需求的最优内存位置
NvSciBufObj buf_obj;
NvSciBufObjAlloc(attr_list, &buf_obj);
// → 若所有访问者都是同一 GPU，分配在显存
// → 若有 DLA 访问者，分配在 CV 内存（可被 GPU + DLA 访问）

// 导出句柄（用于 ROS topic 传输，仅 8~16 字节！）
NvSciBufObjIpcExportDescriptor export_desc;
NvSciBufObjIpcExport(buf_obj, NvSciBufAccessPerm_Read, &export_desc);
// export_desc 可以通过任何 IPC 机制传递（ROS msg / Unix Socket 等）

// 接收端导入
NvSciBufObj imported_obj;
NvSciBufObjIpcImport(peer_module, export_desc, NvSciBufAccessPerm_Read, &imported_obj);
// → 现在 imported_obj 指向发送端分配的同一块物理内存！
```

### 4.3 NvSci 在 NITROS 中的使用

NITROS 使用 NvSci 作为进程间 GPU buffer 传输的底层机制：

```
进程 A（相机驱动）               进程 B（DNN 推理）
NvSciBufObj（GPU 内存）
  → NvSciBufObjIpcExport → 导出句柄（20 字节）
      → 通过 DDS topic 传输（仅传元数据！）
                                    → 接收到 20 字节元数据
                                    → NvSciBufObjIpcImport → NvSciBufObj
                                    → 直接映射到同一块 GPU 物理内存
                                    → TensorRT 推理（零拷贝）
```

**实际跨进程数据量**：
- 传统 ROS（4K 图像 @ 30Hz）：~360 MB/s
- NITROS + NvSci：~0.6 MB/s（仅元数据）

---

## 五、DMA-BUF（Linux 内核内存共享）

在 x86 + dGPU 平台，NvSci 完整功能不可用，NITROS 可回落到 **DMA-BUF**：

```
DMA-BUF：Linux 内核的通用跨驱动内存共享机制
  → GPU 驱动（nvidia.ko）通过 DRM/KMS 导出 GPU buffer 的 dma_buf fd
  → 通过 SCM_RIGHTS（Unix socket）传递 fd 到另一进程
  → 另一进程 mmap(dma_buf_fd) 映射到自己的地址空间
  → 通过该映射直接访问 GPU buffer（若是 Unified Memory）
```

DMA-BUF 在 Jetson（UVA 统一地址）上效果最好，在 dGPU（分离显存）上仍需 PCIe 传输，但至少省去了 CPU 拷贝的开销。

---

## 六、Type Negotiation 状态机

NITROS 节点启动时的协商状态：

```
初始化
  ↓
NegotiatedPublisher::start()
  ↓ 发布支持类型列表（/_negotiated topic）
等待订阅端注册
  ↓（设置超时，默认 2s）
收到所有订阅端的类型列表
  ↓
计算最优类型集合
  ↓
创建底层 Publisher（每种选定类型一个）
  ↓
通知所有订阅端（协商完成，使用 XXX 类型）
  ↓
READY 状态，开始正常发布

（运行中：新订阅端加入）
  ↓ 重新触发协商（可能更换类型）
  ↓（对当前活跃订阅端的影响最小化）
```

**注意**：协商完成前，发布端会缓冲消息或丢弃（取决于配置），不影响实时性保证。
