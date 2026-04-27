# 回放与可视化生态

> **参考来源**：Foxglove Studio docs；PlotJuggler GitHub；mcap CLI；  
> `foxglove/ros-foxglove-bridge@main`；`PlotJuggler/PlotJuggler@master`

---

## 一、回放生态总览

```
rosbag2 bag 文件
    │
    ├── Foxglove Studio ────────── WebSocket bridge（实时）
    │     ├── 原生 MCAP 支持（本地打开）
    │     ├── 3D 视图 / 图像 / 时序图
    │     └── VLA rollout 调试面板
    │
    ├── PlotJuggler ─────────────── ros2 plugin（实时/离线）
    │     ├── 高性能时序图
    │     ├── 关节轨迹分析
    │     └── 数值对比面板
    │
    ├── rviz2 ───────────────────── /clock + use_sim_time
    │     ├── TF 树可视化
    │     ├── 点云 / 图像叠加
    │     └── 机器人模型动画
    │
    └── mcap CLI ────────────────── 命令行分析
          ├── mcap info
          ├── mcap filter
          └── mcap convert
```

---

## 二、Foxglove Studio

### 2.1 概述

Foxglove Studio 是目前机器人领域**最强大的可视化调试工具**，由 Foxglove Technologies 开发（Webviz 的继承者）。

- **原生 MCAP 支持**：直接打开 `.mcap` 文件，无需 ROS 2 环境
- **多 Panel 布局**：3D 视图、图像、时序图、原始消息、主题图
- **WebSocket 实时连接**：通过 `foxglove_bridge` 连接到运行中的 ROS 2 系统

### 2.2 foxglove_bridge 配置

```bash
# 安装 foxglove_bridge
sudo apt install ros-humble-foxglove-bridge

# 启动 bridge（默认端口 8765）
ros2 launch foxglove_bridge foxglove_bridge_launch.xml

# 或直接运行节点
ros2 run foxglove_bridge foxglove_bridge \
  --ros-args \
  -p port:=8765 \
  -p address:=0.0.0.0 \
  -p tls:=false \
  -p certfile:="" \
  -p keyfile:="" \
  -p topic_whitelist:='[".*"]' \
  -p service_whitelist:='[".*"]' \
  -p param_whitelist:='[".*"]' \
  -p max_qos_depth:=10 \
  -p use_sim_time:=false \
  -p num_threads:=4
```

完整 launch 文件集成：
```python
# foxglove_replay.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # 启动 foxglove bridge
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{
                'port': 8765,
                'address': '0.0.0.0',
                'use_sim_time': True,    # 配合 bag 回放的 /clock
                'max_qos_depth': 100,    # 图像等高频 topic 需要更大深度
            }]
        ),
        # 可选：rosbag2 回放（也可手动启动）
        # ExecuteProcess(
        #     cmd=['ros2', 'bag', 'play', '/data/episode_001', '--clock'],
        # )
    ])
```

### 2.3 Layout JSON 配置

Foxglove Layout 可以保存为 JSON，方便团队共享具身智能调试面板：

```json
{
  "configById": {
    "Plot!left": {
      "type": "Plot",
      "title": "关节位置",
      "paths": [
        {
          "value": "/joint_states.position[0]",
          "label": "joint1",
          "color": "#1f77b4",
          "enabled": true
        },
        {
          "value": "/joint_states.position[1]",
          "label": "joint2",
          "color": "#ff7f0e",
          "enabled": true
        }
      ],
      "xAxisVal": "timestamp",
      "showLegend": true,
      "isSynced": true
    },
    "ImageView!top": {
      "type": "ImageView",
      "cameraTopic": "/camera/color/image_raw",
      "calibrationTopic": "/camera/color/camera_info",
      "flipHorizontal": false,
      "flipVertical": false,
      "rotation": 0,
      "minValue": 0,
      "maxValue": 10000
    },
    "3D!main": {
      "type": "3D",
      "followTf": "ee_link",
      "scene": {
        "backgroundColor": "#1a1a2e",
        "enableStats": false
      },
      "topics": {
        "/joint_states": {
          "visible": true
        },
        "/camera/color/image_raw": {
          "visible": true,
          "cameraInfoTopic": "/camera/color/camera_info"
        }
      },
      "transforms": {
        "editable": false,
        "showLabel": true
      }
    },
    "RawMessages!episode": {
      "type": "RawMessages",
      "topicPath": "/episode_marker"
    }
  },
  "layout": {
    "first": "3D!main",
    "second": {
      "first": {
        "first": "ImageView!top",
        "second": "Plot!left",
        "direction": "column",
        "splitPercentage": 50
      },
      "second": "RawMessages!episode",
      "direction": "column",
      "splitPercentage": 70
    },
    "direction": "row",
    "splitPercentage": 60
  }
}
```

### 2.4 自定义 Panel（扩展点）

Foxglove Studio 支持 **Extension（TypeScript）** 开发自定义 Panel：

```typescript
// VLA 动作可视化 Panel（伪代码）
import { PanelExtensionContext, RenderState } from "@foxglove/studio";

function initVlaPanel(context: PanelExtensionContext): void {
  context.subscribe([
    { topic: "/vla_actions" },      // 订阅 VLA 输出的动作
    { topic: "/joint_states" },     // 订阅实际关节状态
  ]);

  context.onRender = (renderState: RenderState, done: () => void) => {
    const vla_actions = renderState.currentFrame?.filter(
      msg => msg.topic === "/vla_actions"
    );
    const joint_states = renderState.currentFrame?.filter(
      msg => msg.topic === "/joint_states"
    );

    // 绘制 VLA 预测 vs 实际轨迹对比
    // ...

    done();
  };
}
```

---

## 三、PlotJuggler

### 3.1 ROS 2 Plugin 配置

```bash
# 安装 PlotJuggler ROS 2 plugin
sudo apt install ros-humble-plotjuggler-ros

# 启动（自动加载 ROS 2 plugin）
ros2 run plotjuggler plotjuggler
```

### 3.2 实时 topic 订阅

在 PlotJuggler 中连接实时 ROS 2 数据：

1. **File → Start**（选择 ROS2 Topic Subscriber）
2. 选择要订阅的 topic（支持多选）：
   - `/joint_states`
   - `/wrench`
   - `/imu/data`
3. 拖拽字段到绘图区：
   - `joint_states/position[0]` → 关节1位置
   - `joint_states/velocity[0]` → 关节1速度

### 3.3 离线 bag 文件分析

PlotJuggler 支持直接打开 **ROS 2 bag 文件**（通过 DataLoadROS2Bag 插件）：

1. **File → Load Data File** → 选择 `.db3` 或包含 `metadata.yaml` 的目录
2. 选择要加载的 topic
3. 支持时间轴联动（与 rviz2 同步）

### 3.4 时序对齐面板（具身智能专用布局）

```
PlotJuggler 具身智能调试推荐布局：

┌─────────────────────────────────────────────────────────────┐
│  Row 1（主视图）：关节位置轨迹（6 轴叠加）                    │
│  x 轴：时间  y 轴：角度 [rad]                                │
│  Series: joint1.pos ... joint6.pos                          │
├─────────────────────────────────────────────────────────────┤
│  Row 2（对比）：关节命令 vs 关节状态                         │
│  Series: joint1.cmd（虚线）vs joint1.pos（实线）             │
│  → 评估跟踪误差                                             │
├─────────────────────────────────────────────────────────────┤
│  Row 3（力矩）：wrench.force.z + wrench.torque 向量         │
├─────────────────────────────────────────────────────────────┤
│  Row 4（IMU）：imu.linear_acceleration.x/y/z               │
└─────────────────────────────────────────────────────────────┘

保存为 PlotJuggler Layout.xml 文件，方便复用。
```

---

## 四、rviz2 回放

### 4.1 与 /clock + use_sim_time 配合

```bash
# 终端1：启动 bag 回放（发布 /clock）
ros2 bag play /data/episode_001 \
  --clock \
  --clock-publish-frequency 100 \
  --rate 0.5   # 半速回放，便于观察

# 终端2：启动 rviz2（配置 use_sim_time）
ros2 run rviz2 rviz2 \
  --ros-args -p use_sim_time:=true \
  -d /config/robot_replay.rviz
```

### 4.2 TF 历史长度配置

回放时 TF 数据历史较长，需要增加 tf2 Buffer 的历史深度：

```python
# robot_replay.launch.py
import tf2_ros
# Buffer 默认历史 10s，回放长 episode 时需增大
buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=300))
```

或在 rviz2 的 TF Display 配置中：
- **TF** → Display Properties → **TF Cache Duration**：设为 300s（5分钟 episode）

### 4.3 推荐 rviz2 配置文件（机械臂回放）

```yaml
# robot_replay.rviz（关键部分）
Visualization Manager:
  Global Options:
    Fixed Frame: world
    Frame Rate: 30

  Displays:
    - Class: rviz_default_plugins/RobotModel
      Name: Robot Model
      Description Source: /robot_description
      Use_Sim_Time: true

    - Class: rviz_default_plugins/TF
      Name: TF
      Frame Timeout: 300   # 秒

    - Class: rviz_default_plugins/Image
      Name: RGB Camera
      Topic: /camera/color/image_raw

    - Class: rviz_default_plugins/PointCloud2
      Name: Depth Point Cloud
      Topic: /camera/depth/points
      Style: Points
      Size(m): 0.01

    - Class: rviz_default_plugins/Path
      Name: EE Trajectory
      Topic: /ee_path          # 需要额外节点发布
```

---

## 五、mcap CLI 工具

mcap CLI 是 Foxglove 提供的命令行工具，直接操作 `.mcap` 文件：

### 5.1 安装

```bash
# Linux (amd64)
curl -L https://github.com/foxglove/mcap/releases/latest/download/mcap-linux-amd64 \
     -o /usr/local/bin/mcap
chmod +x /usr/local/bin/mcap

# 或通过 npm
npm install -g @mcap/cli
```

### 5.2 mcap info — 查看 bag 元数据

```bash
mcap info my_bag_0.mcap

# 输出示例：
library:   rosbag2_storage_mcap 0.7.0
profile:   ros2
messages:  450123
duration:  30.123s (1699920000.000 to 1699920030.123)
compression: zstd
channels:
  (1) /joint_states             sensor_msgs/msg/JointState     30000 msgs
  (2) /camera/color/image_raw   sensor_msgs/msg/Image          900 msgs
  (3) /imu/data                 sensor_msgs/msg/Imu            6000 msgs
  (4) /wrench                   geometry_msgs/msg/WrenchStamped 15000 msgs
```

### 5.3 mcap filter — 按 topic 或时间过滤

```bash
# 只保留关节状态 topic
mcap filter my_bag_0.mcap \
  --output joint_only.mcap \
  --include-topic /joint_states

# 只保留前 10 秒
mcap filter my_bag_0.mcap \
  --output first_10s.mcap \
  --start-secs 0 \
  --end-secs 10

# 组合过滤：特定 topic + 时间范围
mcap filter my_bag_0.mcap \
  --output filtered.mcap \
  --include-topic /joint_states \
  --include-topic /camera/color/image_raw \
  --start-secs 5 --end-secs 25
```

### 5.4 mcap convert — 格式转换

```bash
# 读取 JSON 编码的 mcap，转换为 ROS 2 CDR 格式
# （主要用于跨语言数据交换）
mcap convert input.mcap output_ros2.mcap \
  --output-compression zstd

# mcap 内部转 protobuf → ros2
mcap convert proto.mcap ros2.mcap
```

### 5.5 mcap doctor — 文件完整性检查

```bash
mcap doctor my_bag_0.mcap

# 输出：
✓ Magic bytes valid
✓ Header valid
✓ All chunks CRC valid
✓ Summary section present
✓ All channel IDs referenced
✓ 450123 messages indexed
No issues found.
```

### 5.6 mcap cat — 打印消息内容

```bash
# 打印 /episode_marker 的所有消息
mcap cat my_bag_0.mcap \
  --topics /episode_marker \
  --output-format json | head -20

# 输出示例：
{"topic":"/episode_marker","sequence":0,"logTime":1699920000000000000,"publishTime":1699920000000000000,"data":{"data":"START"}}
{"topic":"/episode_marker","sequence":1,"logTime":1699920028500000000,"publishTime":1699920028500000000,"data":{"data":"END"}}
```

---

## 六、具身智能调试场景：VLA Rollout 检查

### 6.1 VLA 推理回放调试流程

**目标**：回放一条 VLA rollout，检查关节轨迹与环境观测是否一致

```bash
# 步骤1：录制 VLA rollout
ros2 bag record -o /data/vla_rollout_001 \
  /joint_states \
  /joint_commands \
  /camera/color/image_raw \
  /vla_output \           # VLA 输出的动作 token
  /vla_latency \          # VLA 推理延迟
  /episode_marker \
  --storage mcap

# 步骤2：检查 bag 基本信息
ros2 bag info /data/vla_rollout_001
mcap info /data/vla_rollout_001/vla_rollout_001_0.mcap

# 步骤3：回放并在 Foxglove 中可视化
ros2 bag play /data/vla_rollout_001 --clock --rate 0.2
# 在 Foxglove Studio 中：
#   - 3D 面板：robot model 动画
#   - Image 面板：VLA 输入视觉观测
#   - Plot 面板：joint_commands vs joint_states 对比
```

### 6.2 Foxglove 专用 VLA 调试 Layout

```json
{
  "_comment": "VLA Rollout 调试 Layout",
  "layout": {
    "direction": "row",
    "splitPercentage": 55,
    "first": {
      "direction": "column",
      "splitPercentage": 60,
      "first": "3D!robot",
      "second": "Image!camera"
    },
    "second": {
      "direction": "column",
      "splitPercentage": 50,
      "first": "Plot!joint_tracking",
      "second": "Plot!vla_latency"
    }
  },
  "configById": {
    "3D!robot": {
      "type": "3D",
      "followTf": "ee_link",
      "topics": {
        "/joint_states": {"visible": true},
        "/camera/color/image_raw": {"visible": true}
      }
    },
    "Image!camera": {
      "type": "Image",
      "topic": "/camera/color/image_raw"
    },
    "Plot!joint_tracking": {
      "type": "Plot",
      "title": "Joint Tracking Error",
      "paths": [
        {"value": "/joint_states.position[0]", "label": "j1_actual"},
        {"value": "/joint_commands.data[0]", "label": "j1_cmd"}
      ]
    },
    "Plot!vla_latency": {
      "type": "Plot",
      "title": "VLA Inference Latency (ms)",
      "paths": [
        {"value": "/vla_latency.data", "label": "latency_ms"}
      ]
    }
  }
}
```

### 6.3 关节轨迹质量分析脚本

```python
#!/usr/bin/env python3
"""
VLA rollout 关节轨迹质量分析
- 跟踪误差（command vs actual）
- 轨迹平滑度
- 速度限制违规
"""

import rosbag2_py
import numpy as np
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def analyze_vla_rollout(bag_path: str, output_dir: str = '.'):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr')
    )

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    # 加载关节状态和命令
    joint_states = []  # [(ts, positions)]
    joint_commands = []  # [(ts, targets)]

    reader.set_filter(rosbag2_py.StorageFilter(
        topics=['/joint_states', '/joint_commands']))

    while reader.has_next():
        (topic, data, ts) = reader.read_next()
        if topic == '/joint_states':
            msg_type = get_message(topic_types[topic])
            msg = deserialize_message(data, msg_type)
            joint_states.append((ts, np.array(msg.position)))
        elif topic == '/joint_commands':
            # 根据实际消息类型调整
            joint_commands.append((ts, np.frombuffer(data[8:], dtype=np.float64)))

    if not joint_states:
        print("No joint states found")
        return

    # 时间轴（秒）
    t0 = joint_states[0][0]
    state_times = np.array([(ts - t0) / 1e9 for ts, _ in joint_states])
    state_positions = np.array([pos for _, pos in joint_states])

    n_joints = state_positions.shape[1]

    # 绘制关节轨迹
    fig, axes = plt.subplots(n_joints, 1, figsize=(12, 3 * n_joints))
    for j in range(n_joints):
        axes[j].plot(state_times, np.degrees(state_positions[:, j]),
                     label=f'joint{j+1} actual', linewidth=0.8)
        axes[j].set_ylabel('degrees')
        axes[j].set_xlabel('time [s]')
        axes[j].legend()
        axes[j].grid(True, alpha=0.3)

    plt.suptitle('VLA Rollout: Joint Trajectories')
    plt.tight_layout()
    plt.savefig(f'{output_dir}/joint_trajectories.png', dpi=150)
    print(f"Saved: {output_dir}/joint_trajectories.png")

    # 计算速度
    dt = np.diff(state_times)
    velocities = np.diff(state_positions, axis=0) / dt[:, None]
    max_velocity = np.max(np.abs(velocities), axis=0)

    print("\n关节最大速度（rad/s）：")
    for j in range(n_joints):
        print(f"  joint{j+1}: {max_velocity[j]:.4f} rad/s")

    return {
        'duration_s': state_times[-1],
        'n_frames': len(joint_states),
        'max_velocities': max_velocity.tolist(),
    }

if __name__ == '__main__':
    import sys
    bag_path = sys.argv[1] if len(sys.argv) > 1 else '/data/vla_rollout_001'
    result = analyze_vla_rollout(bag_path)
    print(f"\nAnalysis: {result}")
```
