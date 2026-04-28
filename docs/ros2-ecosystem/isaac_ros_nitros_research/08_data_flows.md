# NITROS 端到端数据流与延迟分析

> 本文通过三个典型场景，展示 GPU 零拷贝路径与 CPU 降级路径的完整时序对比。

---

## 一、场景 A：全 NITROS 推理管线（GPU 零拷贝）

**场景**：RealSense D435i → 目标检测 → 位姿估计（全在同一 Jetson AGX Orin 进程）

```
T=0ms
│  相机硬件曝光完成
│  → V4L2/argus_camera 驱动中断
│
T=0.2ms
│  argus_camera_node::captureCallback()
│  → 图像数据在 Jetson iGPU 内存（CPU/GPU 共享物理内存）
│  → NitrosPublisher::publish(NitrosImage{cuda_ptr, width, height, stamp})
│     ↓ Type Negotiation 已完成：订阅端接受 NitrosCudaImage
│     → 仅传递 GXF Entity 句柄（~100 字节元数据）
│     → 无 memcpy
│
T=0.3ms
│  isaac_ros_image_proc::ImageFormatConvertNode::tick()
│  → 从 NitrosImage 取 CUDA pointer（零拷贝）
│  → CUDA kernel：BGR8 → RGB32F + resize（640×640）+ normalize
│     GPU 执行时间：~0.5ms（Orin iGPU）
│
T=0.8ms
│  TensorRTNode::tick()
│  → 从 NitrosTensorList 取输入 CUDA tensor（零拷贝）
│  → TensorRT FP16 推理（YOLOv8s, 640×640）
│     GPU 执行时间：~3ms（Orin iGPU）
│
T=3.8ms
│  DetectionDecoderNode::tick()
│  → GPU 上执行 NMS（Non-Maximum Suppression）
│     GPU 执行时间：~0.2ms
│  → 将检测结果 cudaMemcpy D→H（仅检测框，<1KB）
│  → 发布 vision_msgs::Detection2DArray（CPU 消息）
│
T=4.0ms
│  下游节点（如 6-DOF 位姿估计）接收检测结果
│
总延迟：~4ms（相机到检测结果）
```

**关键**：整个推理链路中，只在最终输出时发生一次 CPU 拷贝（<1KB 检测框），图像数据（~2MB @ 1080P）**全程在 GPU 内存中，零 CPU 拷贝**。

---

## 二、场景 B：插入传统 CPU 节点（降级路径）

**场景**：相机 → 传统 CPU 图像处理节点（OpenCV）→ RVIZ2 可视化

```
T=0ms
│  argus_camera_node::captureCallback()
│  → NitrosPublisher::publish(NitrosImage)
│     ↓ Type Negotiation：
│       订阅端 A（CPU 处理节点）：只支持 sensor_msgs::Image
│       → 触发降级：convert_to_ros_message()
│
T=0.1ms
│  convert_to_ros_message() 执行：
│  → cudaMemcpy Device→Host（1080P RGB8：~6MB）
│     时间：~2ms（Jetson LPDDR5 内存带宽 ~68 GB/s）
│  → 填充 sensor_msgs::Image（宽/高/编码/步长）
│  → DDS publish（sensor_msgs::Image）
│
T=2.2ms
│  CPU 处理节点接收 sensor_msgs::Image（标准 DDS 反序列化）
│  → OpenCV 处理：~5ms
│  → 发布结果 topic
│
T=7.2ms
│  RVIZ2 收到结果，显示
│
总延迟：~7ms（其中 2ms 是 GPU→CPU 拷贝开销）
```

**对比**：插入一个 CPU 节点导致每帧增加 ~2ms 延迟（GPU→CPU 拷贝），在 30Hz 下额外消耗 **~180 MB/s CPU-GPU 内存带宽**。

---

## 三、场景 C：跨进程 NvSci 传输（Jetson 专属）

**场景**：相机驱动进程（A）→ DNN 推理进程（B）跨进程 GPU 零拷贝

```
进程 A（相机驱动）                 进程 B（DNN 推理）

T=0ms
图像在 NvSciBuf GPU 内存
  → NvSciBufObjIpcExport(buf_obj)
     返回 export_desc（20 字节句柄）
  → NitrosPublisher::publish()
     → DDS 传输 export_desc（20 字节！）

                                T=0.05ms
                                DDS 消息到达（仅 20 字节）
                                → NitrosSubscriber 回调
                                → NvSciBufObjIpcImport(export_desc)
                                   → 映射到进程 B 的地址空间
                                   → 获得 CUDA pointer（指向同一物理内存）
                                → TensorRT 推理（从同一物理内存读）

跨进程传输数据量：20 字节
传输延迟（DDS，本机）：~50µs
跨进程内存拷贝：0 字节 ✅
```

**对比传统跨进程**（同样 4K 图像）：
- 传统 DDS：序列化 + 发布 ~12MB → 对端反序列化，~20ms + 12MB 拷贝
- NITROS NvSci：20 字节元数据，~50µs，0MB 拷贝

---

## 四、内存带宽消耗对比

| 场景 | 数据量/帧 | 带宽（@30Hz） | GPU 参与 |
|------|----------|------------|---------|
| 传统 ROS（1080P RGB） | 6MB | 180 MB/s（序列化）+ 180 MB/s（反序列化）| 否 |
| TypeAdapter + IntraProcess | 0B（指针传递） | ~0 | 否 |
| NITROS NvSci（同进程） | 0B | ~0 | ✅ |
| NITROS NvSci（跨进程，Jetson） | 20B（句柄） | ~0 | ✅ |
| NITROS 降级（到 CPU 节点） | 6MB（D→H） | 180 MB/s | 部分 |
| CUDA IPC（跨进程，x86+dGPU） | 32B（IPC handle） | ~0（PCIe 不传数据）| ✅ |

---

## 五、延迟分解汇总

```
NITROS 全 GPU 路径（同一进程，Jetson AGX Orin，1080P）：
┌──────────────────────────────────────────────────────────┐
│  相机曝光  │ 驱动回调 │ 图像预处理  │ TRT 推理 │ 后处理  │
│    ~33ms   │  ~0.2ms  │    ~0.5ms  │  ~3ms    │ ~0.2ms  │
└──────────────────────────────────────────────────────────┘
算法总延迟（曝光后）：~4ms

传统 ROS CPU 路径（同样硬件）：
┌────────────────────────────────────────────────────────────────┐
│ 曝光 │ 驱动回调 │ CPU 拷贝 │ 序列化 │ DDS │ 反序列化 │ OpenCV │
│ ~33ms│  ~0.2ms  │  ~2ms   │  ~2ms  │~1ms │  ~2ms    │  ~5ms  │
└────────────────────────────────────────────────────────────────┘
算法总延迟：~12ms（拷贝开销 ~6ms，算法本身 ~5ms）
```

---

## 六、GXF 调度延迟（内部）

GXF 的 Greedy Scheduler（贪心调度器）在每个 tick 的开销：

| 操作 | 延迟 |
|------|------|
| Codelet tick 触发（调度器开销） | ~10µs |
| GXF Entity 传递（队列 push/pop） | ~5µs |
| NvSciBuf 句柄 export/import | ~50µs（首次），~5µs（已缓存） |
| GXF→ROS 桥接（发布元数据） | ~20µs |
| ROS→GXF 桥接（注入 Entity） | ~20µs |

**关键结论**：GXF 内部开销是**微秒级**，主要延迟来自 GPU kernel 执行时间和（降级路径的）cudaMemcpy。
