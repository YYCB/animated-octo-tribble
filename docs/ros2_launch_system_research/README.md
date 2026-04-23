# ROS2 Launch 系统深度调研 - 目录

## 文件列表

| 文件 | 内容 |
|------|------|
| [01_overview.md](./01_overview.md) | Launch 系统架构、LaunchDescription、执行流程 |
| [02_actions.md](./02_actions.md) | 核心 Action：Node/ExecuteProcess/IncludeLaunchDescription/GroupAction |
| [03_events.md](./03_events.md) | 事件系统：OnProcessStart/OnShutdown/RegisterEventHandler |
| [04_substitutions.md](./04_substitutions.md) | 替换系统：LaunchConfiguration/PathJoinSubstitution/PythonExpression |
| [05_summary.md](./05_summary.md) | 速查手册、常用模式、调试方法 |

## 核心架构

```
.launch.py 文件
    └── generate_launch_description() → LaunchDescription
            └── List[LaunchDescriptionEntity]
                    ├── Node（launch_ros）
                    ├── ExecuteProcess
                    ├── IncludeLaunchDescription
                    ├── GroupAction
                    ├── DeclareLaunchArgument
                    ├── SetEnvironmentVariable
                    └── RegisterEventHandler

LaunchService（核心执行引擎）：
    ├── LaunchContext（全局状态、substitution 解析上下文）
    ├── EventQueue（事件队列，asyncio）
    └── ProcessManager（子进程管理）

ros2 launch → LaunchService.run() → asyncio event loop
```
