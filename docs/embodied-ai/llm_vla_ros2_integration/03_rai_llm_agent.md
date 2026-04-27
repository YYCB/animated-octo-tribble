# `rai`（Robec）框架 — LLM Agent + ROS 2 工具调用模式

> 项目地址：https://github.com/RobecStar/rai  
> 定位：让 LLM 通过"工具调用"直接与 ROS 2 机器人交互

---

## 一、rai 框架概述

### 1.1 核心思想

`rai` 将 ROS 2 的**服务（Service）、动作（Action）、话题（Topic）**包装为 LLM 可调用的"工具"（Tool），让 GPT-4o / Llama3 / Qwen 等 LLM 通过标准的 Function Calling 接口控制机器人。

```
用户指令："把桌上的红色杯子移到抽屉里"
    ↓
LLM（GPT-4o / 本地模型）
  → 分析：需要 1) 检测红色杯子位置 2) 规划抓取 3) 导航到抽屉 4) 放置
  → 调用工具：detect_object("red cup")
    ↓ (ROS 2 Service 调用)
  ← 返回：{object_id: "cup_0", pose: {x: 0.3, y: 0.1, z: 0.8}}
  → 调用工具：pick_object("cup_0")
    ↓ (ROS 2 Action 调用，等待完成)
  ← 返回：{success: true}
  → 调用工具：navigate_to("drawer_area")
  ...
```

### 1.2 架构组件

```
┌─────────────────────────────────────────────────────────────┐
│  rai Agent（Python 进程）                                     │
│  ├── LLM Client（OpenAI SDK / LiteLLM / Ollama）             │
│  ├── Tool Registry（ROS 2 接口 → LLM 工具定义）               │
│  ├── Memory（对话历史 + 任务状态）                             │
│  └── Safety Governor（工具调用白名单/黑名单）                  │
├─────────────────────────────────────────────────────────────┤
│  ROS 2 Bridge Layer（rai_ros2 包）                            │
│  ├── ServiceToolWrapper                                      │
│  ├── ActionToolWrapper                                       │
│  └── TopicReaderTool（查询最新 topic 值）                     │
├─────────────────────────────────────────────────────────────┤
│  ROS 2 机器人系统                                             │
│  （perception / manipulation / navigation 节点）              │
└─────────────────────────────────────────────────────────────┘
```

---

## 二、工具定义与注册

### 2.1 将 ROS 2 Service 包装为工具

```python
# rai 工具定义（基于 Pydantic + OpenAI Function Calling schema）
from rai.tools import ROS2ServiceTool, ServiceCallConfig

detect_object_tool = ROS2ServiceTool(
    name="detect_object",
    description="Detect an object by name/color in the current scene. Returns object pose.",
    service_name="/perception/detect_object",
    service_type="vision_msgs/srv/Detection3D",  # 或自定义 srv
    input_schema={
        "type": "object",
        "properties": {
            "object_description": {
                "type": "string",
                "description": "Description of the object to detect, e.g., 'red cup', 'blue box'"
            }
        },
        "required": ["object_description"]
    },
    # 响应解析：将 ROS 2 srv Response 转为 LLM 易读的字典
    response_parser=lambda resp: {
        "found": resp.detected,
        "pose": {
            "x": resp.pose.position.x,
            "y": resp.pose.position.y,
            "z": resp.pose.position.z
        } if resp.detected else None
    },
    timeout=5.0  # ROS 2 service call timeout (s)
)
```

### 2.2 将 ROS 2 Action 包装为工具

```python
from rai.tools import ROS2ActionTool

pick_object_tool = ROS2ActionTool(
    name="pick_object",
    description="Pick up an object. Use object_id returned by detect_object.",
    action_server="/manipulation/pick",
    action_type="my_robot_msgs/action/Pick",
    input_schema={...},
    # Action 是异步的，rai 内部等待 result（或 timeout）
    wait_for_result=True,
    timeout=30.0,
    # Feedback 处理（可选，发给 LLM 作为 observation）
    feedback_handler=lambda fb: f"Pick progress: {fb.progress:.0%}"
)
```

### 2.3 Topic Reader（感知快照）

```python
from rai.tools import ROS2TopicReaderTool

get_joint_states_tool = ROS2TopicReaderTool(
    name="get_joint_states",
    description="Get current robot joint positions (radians).",
    topic="/joint_states",
    msg_type="sensor_msgs/JointState",
    response_parser=lambda msg: {
        "joints": dict(zip(msg.name, [round(p, 3) for p in msg.position]))
    },
    cache_timeout=0.1  # 最多使用 0.1s 前的缓存，避免频繁订阅
)
```

---

## 三、Agent 循环（ReAct 模式）

`rai` 使用 **ReAct（Reasoning + Acting）** 模式：LLM 交替"思考"和"行动"。

```python
# rai Agent 核心循环（简化）
class RaiAgent:
    def __init__(self, llm_client, tools: list, system_prompt: str):
        self.llm = llm_client
        self.tools = {t.name: t for t in tools}
        self.messages = [{"role": "system", "content": system_prompt}]

    def run(self, user_instruction: str) -> str:
        self.messages.append({"role": "user", "content": user_instruction})

        while True:
            # LLM 推理：决定下一步（工具调用 or 最终回答）
            response = self.llm.chat.completions.create(
                model="gpt-4o",
                messages=self.messages,
                tools=[t.openai_schema for t in self.tools.values()],
                tool_choice="auto"
            )

            message = response.choices[0].message
            self.messages.append(message)

            # 没有工具调用 → 任务完成
            if not message.tool_calls:
                return message.content

            # 执行工具调用
            for tool_call in message.tool_calls:
                tool = self.tools[tool_call.function.name]
                args = json.loads(tool_call.function.arguments)

                # 安全检查（白名单验证）
                if not self.safety_governor.allow(tool_call.function.name, args):
                    result = {"error": "Action blocked by safety governor"}
                else:
                    # 调用 ROS 2 接口
                    result = tool.execute(**args)

                # 将工具结果反馈给 LLM
                self.messages.append({
                    "role": "tool",
                    "tool_call_id": tool_call.id,
                    "content": json.dumps(result)
                })
```

---

## 四、Safety Governor（安全治理）

在 LLM 控制机器人时，**安全治理是非可选项**：

```python
class SafetyGovernor:
    # 白名单：允许 LLM 调用的工具
    ALLOWED_TOOLS = {
        "detect_object",
        "get_joint_states",
        "navigate_to",
        "pick_object",
        "place_object",
    }

    # 黑名单参数规则
    BLOCKED_ARGS_RULES = [
        # 不允许导航到危险区域
        lambda tool, args: tool == "navigate_to" and args.get("location") in DANGER_ZONES,
        # 不允许以超高速度移动
        lambda tool, args: tool == "move_joints" and any(abs(v) > 1.0 for v in args.get("velocities", [])),
    ]

    def allow(self, tool_name: str, args: dict) -> bool:
        if tool_name not in self.ALLOWED_TOOLS:
            RCLPY_LOGGER.warning(f"Tool {tool_name} not in whitelist")
            return False
        for rule in self.BLOCKED_ARGS_RULES:
            if rule(tool_name, args):
                RCLPY_LOGGER.warning(f"Tool call blocked by rule: {tool_name}({args})")
                return False
        return True
```

---

## 五、与 ROS 2 生命周期的集成

`rai` Agent 可以作为 ROS 2 **LifecycleNode** 运行，确保与机器人系统启停同步：

```python
class RaiAgentNode(LifecycleNode):
    def on_activate(self, state):
        # 建立 LLM Client，注册所有工具
        self.agent = RaiAgent(llm_client=..., tools=self._build_tools())
        # 订阅人类指令 topic
        self.instruction_sub = self.create_subscription(
            String, '/rai/instruction', self.on_instruction, 10)
        return TransitionCallbackReturn.SUCCESS

    def on_instruction(self, msg: String):
        # 异步执行，避免阻塞 ROS 2 spin
        self.executor.submit(self.agent.run, msg.data)

    def on_deactivate(self, state):
        # 停止所有进行中的 LLM 调用
        self.agent.cancel_all()
        return TransitionCallbackReturn.SUCCESS
```

---

## 六、与 VLA 的结合（LLM → VLA 调用）

`rai` 可以将 VLA 推理包装为一个高层工具：

```python
# VLA 技能被包装为 rai 工具
grasp_with_vla_tool = ROS2ActionTool(
    name="grasp_with_vla",
    description="""Use Visual Language Action model to grasp an object.
    The VLA uses camera images and language to guide fine manipulation.
    Use this when precise grasping is needed.""",
    action_server="/vla/grasp",
    action_type="vla_msgs/action/VlaGrasp",
    input_schema={
        "type": "object",
        "properties": {
            "instruction": {"type": "string", "description": "Grasp instruction, e.g., 'grasp the handle'"},
            "object_pose": {"type": "object", "description": "Approximate object pose from detection"}
        }
    },
    timeout=60.0  # VLA 推理可能较慢
)
```

这样 LLM 的决策链是：
```
LLM → detect_object（感知）→ grasp_with_vla（精细操作，触发 VLA）→ navigate_to（移动）→ place_object
```

**优势**：LLM 处理任务逻辑和常识推理，VLA 处理精细视觉运动控制，各司其职。
