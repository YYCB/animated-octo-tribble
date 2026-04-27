# 推理服务部署方式对比 — 本机 GPU / gRPC / Triton

> 覆盖：VLA 模型部署的三种主要模式及对 ROS 2 节点拓扑的影响

---

## 一、三种部署模式概述

```
┌─────────────────────────────────────────────────────────────────────┐
│  模式 A：本机 GPU（Jetson / x86 + dGPU）                             │
│  VLA 推理 ← TensorRT / llama.cpp / vLLM → ROS 2 节点（同一进程/机器）│
│  延迟：10-100ms，带宽：GPU 内部，适合：边缘部署                        │
├─────────────────────────────────────────────────────────────────────┤
│  模式 B：远端 gRPC（边-云协同）                                       │
│  ROS 2 节点 → gRPC 客户端 → 网络 → gRPC 服务器（云/局域网 GPU 服务器）│
│  延迟：50-500ms（含网络），适合：大模型 + 低频决策                      │
├─────────────────────────────────────────────────────────────────────┤
│  模式 C：Triton Inference Server（本机或远端）                        │
│  ROS 2 节点 → Triton gRPC/HTTP → 模型仓库（TensorRT/ONNX/TF）       │
│  延迟：5-50ms（本机），适合：多模型、高并发、生产部署                   │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 二、模式 A：本机 GPU 直接推理

### 2.1 适用场景

- Jetson AGX Orin（64GB）或 Jetson Orin NX + 小型 VLA 模型
- 对延迟要求高（π0 的 50Hz 控制）
- 无稳定网络连接（野外 / 工厂内）

### 2.2 技术栈

```
TensorRT（NVIDIA 官方）
  → 最优性能，需要将模型转换为 .plan 文件
  → 支持 INT8 量化（Jetson Orin 显著加速）
  → VLA 推理节点直接调用 TensorRT C++ API

llama.cpp（GGUF 格式，仅 LLM 部分）
  → 支持 Jetson / 普通 GPU / CPU
  → 通过 llama.cpp 的 OpenAI 兼容服务器接入 rai
  → 启动：`llama-server -m qwen2.5-7b.gguf --port 8080`

vLLM（高吞吐 LLM 服务）
  → 适合 x86 + dGPU
  → PagedAttention，高并发多请求
```

### 2.3 ROS 2 节点集成（TensorRT 路径）

```python
# vla_inference_node.py（本机 TensorRT 推理）
class VlaInferenceNode(Node):
    def __init__(self):
        # 加载 TensorRT engine（在节点启动时）
        self.engine = load_trt_engine('/models/openvla_fp16.plan')
        self.context = self.engine.create_execution_context()
        
        # 订阅：来自相机的 NitrosTensorList（预处理后的图像 tensor）
        self.img_sub = create_subscription(
            NitrosTensorList, '/dnn_encoder/tensor_pub',
            self.on_image_tensor, SensorDataQoS())
        
        # 订阅：语言条件
        self.lang_sub = create_subscription(
            String, '/vla/language_instruction', self.on_language, 10)
        
        # 发布：VLA 动作输出
        self.action_pub = create_publisher(
            Float32MultiArray, '/vla/action_output', SensorDataQoS())
    
    def on_image_tensor(self, tensor_list: NitrosTensorList):
        if self.current_language is None:
            return
        # GPU 推理（CUDA 零拷贝路径：来自 NITROS 的 tensor 直接作为输入）
        input_ids = self.tokenize_language(self.current_language)
        action_tokens = self.context.execute_v2([
            tensor_list.get_cuda_ptr(0),  # 图像特征
            input_ids.cuda_ptr,           # 语言 token
        ])
        # 反量化 7 个动作 token → 连续动作值
        action = self.detokenize_actions(action_tokens)
        self.action_pub.publish(Float32MultiArray(data=action.tolist()))
```

### 2.4 本机部署的延迟预算（Jetson AGX Orin）

```
图像预处理（NITROS/GPU）：~0.5ms
VLA 推理（TensorRT FP16，OpenVLA 7B）：~40-80ms
动作反量化 + 发布：~0.5ms
总端到端：~42-82ms → 支持 12-24 Hz 控制频率
```

---

## 三、模式 B：远端 gRPC 推理服务

### 3.1 适用场景

- LLM 模型太大（GPT-4o、Llama-70B），Jetson 无法运行
- 高层任务规划（0.5-10s 延迟可接受）
- 存在可靠局域网连接（10GbE / Wi-Fi 6）

### 3.2 ROS 2 → gRPC 桥接节点

```python
# llm_grpc_bridge_node.py
import grpc
from openai import OpenAI  # OpenAI SDK（也支持 vLLM 的 OpenAI 兼容接口）

class LlmGrpcBridgeNode(Node):
    def __init__(self):
        # 支持两种后端：
        # 1. OpenAI API（云端 GPT-4o）
        # 2. 本地 vLLM / llama.cpp server（局域网）
        server_url = declare_parameter('llm_server_url', 'http://192.168.1.100:8080/v1')
        self.client = OpenAI(base_url=server_url, api_key="dummy")
        
        # Action Server：接收高层任务指令
        self.action_server = ActionServer(
            self, ChatAction, '/llm/chat',
            self.handle_chat_goal)
    
    async def handle_chat_goal(self, goal_handle):
        goal = goal_handle.request
        feedback = ChatAction.Feedback()
        
        # 流式调用 LLM（支持 token-by-token 反馈）
        stream = self.client.chat.completions.create(
            model=self.model_name,
            messages=[{"role": "user", "content": goal.user_message}],
            tools=self.tools_schema,
            stream=True
        )
        
        result = ChatAction.Result()
        for chunk in stream:
            if chunk.choices[0].delta.content:
                feedback.partial_message += chunk.choices[0].delta.content
                goal_handle.publish_feedback(feedback)
        
        # 处理工具调用（若有）
        result.assistant_message = self._finalize_response(stream)
        goal_handle.succeed()
        return result
```

### 3.3 网络延迟对任务规划的影响

```
局域网（千兆以太网，RTT ~1ms）：
  LLM 推理 500ms + 网络 1ms ≈ 501ms → 可接受（任务规划层）

Wi-Fi 6（良好信号，RTT ~10ms）：
  LLM 推理 500ms + 网络 10ms ≈ 510ms → 可接受

4G LTE（RTT ~50ms）：
  LLM 推理 500ms + 网络 50ms ≈ 550ms → 勉强接受（低频规划）

互联网（RTT ~100-500ms）：
  GPT-4o API：500-2000ms → 仅适合高层规划，不适合实时控制

结论：VLA（Layer 3，高频控制）必须本机部署；LLM（Layer 4，任务规划）可以远端。
```

---

## 四、模式 C：Triton Inference Server

### 4.1 为什么用 Triton

Triton 解决了**多模型管理**问题：

```
没有 Triton 的部署：
  每个推理节点自己加载模型 → 显存碎片化、无动态装卸
  
有 Triton 的部署：
  所有模型统一由 Triton 管理
  ├── OpenVLA（TensorRT，GPU）
  ├── FoundationPose（TensorRT，GPU）
  ├── 自定义感知模型（ONNX，GPU）
  └── 小型 NLP 分类器（ONNX，CPU）
  
  ROS 2 节点通过 gRPC 调用 → Triton 按需调度显存
```

### 4.2 Isaac ROS + Triton 集成

`isaac_ros_triton` 是 Isaac ROS 提供的 Triton 客户端节点：

```yaml
# triton_node 参数
triton_node:
  ros__parameters:
    model_name: "openvla"              # 对应 Triton 模型仓库中的目录名
    server_url: "localhost:8001"        # Triton gRPC 地址
    input_binding_names: ["image", "input_ids"]
    output_binding_names: ["action_logits"]
    # Triton 特有：动态批处理（multi-query batching）
    max_batch_size: 1                  # VLA 通常 batch=1
```

### 4.3 Triton 模型仓库配置

```
model_repository/
└── openvla/
    ├── config.pbtxt
    └── 1/
        └── model.plan    # TensorRT engine 文件

# config.pbtxt（Triton 模型配置）
name: "openvla"
backend: "tensorrt"
max_batch_size: 1
input [
  { name: "image"     data_type: TYPE_FP32  dims: [3, 224, 224] },
  { name: "input_ids" data_type: TYPE_INT32 dims: [-1] }          # 动态长度
]
output [
  { name: "action_logits" data_type: TYPE_FP32 dims: [7, 256] }
]
instance_group [{ kind: KIND_GPU, count: 1 }]
```

---

## 五、部署方案选型决策矩阵

| 因素 | 本机 TensorRT | 远端 gRPC | Triton（本机/远端）|
|------|:---:|:---:|:---:|
| 推理延迟 | ⭐⭐⭐⭐⭐ | ⭐⭐（低频）| ⭐⭐⭐⭐ |
| 高频控制（>10Hz）| ✅ | ❌ | ✅（本机）|
| 大模型（>13B）| ❌（Jetson）| ✅ | ⭕ |
| 多模型并发 | ❌（需各自加载）| ✅ | ✅ |
| 无网络离线运行 | ✅ | ❌ | ✅（本机）|
| 工程复杂度 | 低 | 中 | 高 |
| 生产可靠性 | 高 | 依赖网络 | 高 |

**推荐架构（具身智能机器人）**：
- **Layer 3 VLA（高频）**：本机 TensorRT（Jetson AGX Orin）
- **Layer 4 LLM（低频）**：本机 llama.cpp（7B/13B 量化模型） 或 远端 vLLM（70B+）
- **生产部署**：本机 Triton 统一管理所有推理模型
