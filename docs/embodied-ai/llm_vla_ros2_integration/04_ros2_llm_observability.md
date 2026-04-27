# `ros2_llm` 与 Foxglove + LLM 可观测性集成

> 参考项目：https://github.com/mgonzs13/ros2_llm  
> 参考工具：Foxglove Studio、PlotJuggler、ros2_tracing

---

## 一、`ros2_llm` 包概览

`ros2_llm` 是一个将 LLM 集成进 ROS 2 节点图的轻量框架，主要特点：

| 特性 | 说明 |
|------|------|
| 后端支持 | OpenAI（GPT-4o）、Ollama（本地）、Google Gemini |
| ROS 2 接口 | Action Server（长时推理）+ Topic（流式输出）|
| 多模态 | 支持图像输入（vision）|
| 内存 | 对话历史（ConversationBuffer）|
| 工具调用 | 通过 langchain 的 Tool 机制 |

### 1.1 包结构

```
ros2_llm/
├── ros2_llm/               ← Python 节点
│   ├── llm_node.py         ← LLM Action Server（核心）
│   ├── chat_node.py        ← 对话管理 + 多轮历史
│   └── tool_executor.py    ← 工具执行调度
├── ros2_llm_interfaces/    ← 自定义消息/服务/动作
│   ├── action/Chat.action  ← LLM 对话 Action
│   ├── msg/ChatMessage.msg ← 单条消息（role + content）
│   └── srv/GetModels.srv   ← 查询可用模型
└── ros2_llm_tools/         ← 内置 ROS 2 工具
    ├── topic_reader.py
    └── service_caller.py
```

### 1.2 `Chat.action` 接口定义

```
# ros2_llm_interfaces/action/Chat.action
# Goal
string user_message
sensor_msgs/Image[] images   # 可选，多模态输入
string[] tool_names          # 允许使用的工具子集

---
# Result
string assistant_message
string[] tool_calls_made     # 记录实际调用的工具
bool success

---
# Feedback
string partial_message       # 流式输出（token-by-token）
string current_tool          # 当前正在执行的工具名
```

---

## 二、可观测性的必要性

LLM + VLA 系统的调试与监控比传统机器人系统**难得多**：

```
传统 ROS 2 系统的可观测需求：
  - 话题频率是否正常？（hz / bandwidth）
  - 消息内容是否正确？（echo / rviz）
  → ros2 topic hz / ros2 topic echo / rviz2 足够

LLM + VLA 系统额外需要观测：
  - LLM 在每个时刻做了什么决策？为什么？
  - VLA 推理输出了什么动作？与实际执行是否吻合？
  - 工具调用序列是否符合预期？耗时多少？
  - LLM 幻觉（hallucination）是否导致了错误操作？
  - 某次任务失败是因为：感知错误 / LLM 误判 / VLA 执行偏差 / 硬件问题？
```

---

## 三、Foxglove Studio + LLM 可观测性

Foxglove Studio 是 ROS 2 的 Web-based 可视化工具，支持自定义消息 + 自定义 Panel。

### 3.1 LLM 思维链可视化 Panel

```typescript
// Foxglove 自定义 Panel（TypeScript）
// 显示 LLM 的思维链（tool calls sequence）

import { PanelExtensionContext, Topic, MessageEvent } from "@foxglove/studio";

function LLMReasoningPanel({ context }: { context: PanelExtensionContext }) {
  const [toolCalls, setToolCalls] = useState<ToolCall[]>([]);

  useEffect(() => {
    // 订阅 LLM 工具调用记录 topic
    context.subscribe([{ topic: "/rai/tool_call_log" }]);
    context.onRender = (renderState) => {
      const msgs = renderState.currentFrame?.get("/rai/tool_call_log");
      msgs?.forEach((event) => {
        setToolCalls(prev => [...prev, {
          timestamp: event.receiveTime,
          tool: event.message.tool_name,
          args: JSON.parse(event.message.args_json),
          result: JSON.parse(event.message.result_json),
          duration_ms: event.message.duration_ms,
        }]);
      });
    };
  }, [context]);

  return (
    <div className="llm-reasoning-panel">
      <h3>LLM Tool Call Timeline</h3>
      {toolCalls.map((call, i) => (
        <div key={i} className={`tool-call ${call.result.success ? "success" : "failure"}`}>
          <span className="tool-name">{call.tool}</span>
          <span className="duration">{call.duration_ms}ms</span>
          <pre>{JSON.stringify(call.args, null, 2)}</pre>
        </div>
      ))}
    </div>
  );
}
```

### 3.2 需要发布的 LLM 可观测 Topic

在 `rai` / `ros2_llm` 节点中需要额外发布以下调试 Topic：

```python
# 工具调用日志（每次工具调用发布一条）
tool_call_log_pub = create_publisher(
    ToolCallLog,  # 自定义消息
    '/rai/tool_call_log',
    rclcpp_py.QoSProfile(depth=1000))  # 保留历史，用于 rosbag2 录制

# LLM 推理链（思维过程，流式）
reasoning_pub = create_publisher(
    String, '/rai/reasoning_stream', 10)

# 当前任务状态
task_status_pub = create_publisher(
    TaskStatus, '/rai/task_status', 10)

# VLA 动作可视化（发布 VLA 输出的动作向量，用于 PlotJuggler 绘图）
vla_action_debug_pub = create_publisher(
    Float32MultiArray, '/vla/action_debug', 10)
```

### 3.3 `ToolCallLog` 消息定义

```
# custom_msgs/msg/ToolCallLog.msg
std_msgs/Header header
string tool_name
string args_json          # JSON 序列化的工具参数
string result_json        # JSON 序列化的工具结果
float64 duration_ms       # 工具执行耗时
bool success
string error_message      # 若失败
string llm_model          # 使用的模型（gpt-4o / llama3-70b）
```

---

## 四、PlotJuggler — VLA 动作监控

PlotJuggler 擅长绘制多维时序数据，非常适合监控 VLA 的 7 维动作输出：

```
显示配置示例：
  /vla/action_debug/data[0] → Δx（m/s）
  /vla/action_debug/data[1] → Δy（m/s）
  /vla/action_debug/data[2] → Δz（m/s）
  /vla/action_debug/data[6] → gripper（0-1）

  /joint_states/position → 实际关节位置（7 维）
  /joint_states/velocity → 实际关节速度

  对比 VLA 命令 vs 实际执行：快速发现跟踪误差
```

---

## 五、`ros2_tracing` — 端到端延迟追踪

`ros2_tracing` 使用 LTTng 内核/用户空间 tracepoint，可以测量消息从发布到接收的延迟：

```bash
# 开启 tracing，记录 rclcpp 的 publish/callback tracepoint
ros2 trace --session-name llm_vla_trace \
  --ros2 \
  --kernel sched_switch  # 追踪线程切换

# 分析：从相机帧到 VLA 动作输出的端到端延迟
ros2 run tracetools_analysis analyze \
  --session llm_vla_trace \
  --end-to-end /camera/image_raw:/vla/action_output
```

关键延迟指标：
- `T_camera → T_vla_input`：图像到 VLA 推理输入的延迟（目标：< 5ms，NITROS 路径）
- `T_vla_input → T_vla_output`：VLA 推理延迟（目标：< 100ms @ TensorRT FP16）
- `T_vla_output → T_joint_cmd`：动作桥接延迟（目标：< 2ms）
- `T_joint_cmd → T_actual_motion`：控制器响应延迟（目标：< 10ms）

---

## 六、LLM 调用的可重放性（rosbag2 集成）

在调试 LLM + VLA 系统时，**完整的可重放性**至关重要：

```bash
# 录制所有相关 topic（用于离线调试和训练数据采集）
ros2 bag record \
  /camera/image_raw \              # 原始图像
  /rai/tool_call_log \             # LLM 工具调用序列
  /rai/task_status \               # 任务状态
  /vla/action_debug \              # VLA 动作输出
  /joint_states \                  # 实际关节状态
  /joint_trajectory_controller/state  # 控制器状态

# 回放（用于离线分析 / 数据标注）
ros2 bag play recording/ \
  --topics /camera/image_raw /rai/tool_call_log \
  --rate 0.5  # 半速回放便于分析
```

**关键注意**：录制时图像数据量大，建议：
1. 使用 `mcap` 格式 + `zstd` 压缩（比 sqlite3 小 3-5×）
2. 或者只录制压缩图像（`/camera/image_raw/compressed`）
3. LLM 相关的文本 topic 务必录制（体积小但价值高）
