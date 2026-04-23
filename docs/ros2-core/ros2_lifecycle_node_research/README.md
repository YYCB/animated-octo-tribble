# ROS2 Lifecycle Node 深度调研 - 目录

## 文件列表

| 文件 | 内容 |
|------|------|
| [01_overview.md](./01_overview.md) | Lifecycle 状态机、节点类型、15个状态转换 |
| [02_state_machine.md](./02_state_machine.md) | rcl_lifecycle 状态机源码、callback 注册 |
| [03_rclcpp_lifecycle.md](./03_rclcpp_lifecycle.md) | rclcpp::LifecycleNode 完整 API |
| [04_lifecycle_manager.md](./04_lifecycle_manager.md) | 外部管理：LifecycleManager、Service 接口 |
| [05_summary.md](./05_summary.md) | 速查手册、状态表、调试命令 |

## 核心状态机

```
         configure()         activate()
UNCONFIGURED ──────► INACTIVE ──────► ACTIVE
     ▲                 │  ▲             │
     │   cleanup()     │  │ deactivate()│
     └─────────────────┘  └────────────┘
     
     ACTIVE/INACTIVE → shutdown() → FINALIZED
     任意状态 → 回调返回失败 → ERRORPROCESSING
     ERRORPROCESSING → cleanup() → UNCONFIGURED（恢复）
                     → error() → FINALIZED（不可恢复）

关键 Service（每个 Lifecycle Node 自动创建）：
  ~/change_state           (lifecycle_msgs/srv/ChangeState)
  ~/get_state             (lifecycle_msgs/srv/GetState)
  ~/get_available_states  (lifecycle_msgs/srv/GetAvailableStates)
  ~/get_available_transitions
  ~/get_transition_graph
```
