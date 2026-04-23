# ROS2 Component 机制深度调研 - 目录

本目录包含对 ROS2 Component（组件）机制的源码级深度调研，覆盖架构设计、关键实现、调度模型和工程实践。

## 文件列表

| 文件 | 内容 | 适合读者 |
|------|------|---------|
| [01_overview.md](./01_overview.md) | Component 概述、架构层次、包列表、生命周期 | 入门 |
| [02_component_manager_source.md](./02_component_manager_source.md) | ComponentManager 源码、NodeFactory、class_loader、进程内通信 | 进阶 |
| [03_executor_and_scheduling.md](./03_executor_and_scheduling.md) | Executor 调度模型、CallbackGroup、SingleThread/MultiThread | 进阶 |
| [04_lifecycle_and_launch.md](./04_lifecycle_and_launch.md) | LifecycleNode 状态机、Launch 文件集成、静态/动态组合对比 | 工程 |
| [05_summary_and_cheatsheet.md](./05_summary_and_cheatsheet.md) | 速查手册、常见陷阱、源码索引 | 速查 |

## 核心知识点

1. **Component = Node + 动态加载** - 通过 `RCLCPP_COMPONENTS_REGISTER_NODE` 宏注册节点工厂
2. **ComponentManager** - 负责加载/卸载组件的特殊节点，提供 3 个 ROS 服务接口
3. **IntraProcessManager** - 进程内 `unique_ptr` 传递，实现零拷贝通信
4. **class_loader** - 基于 `dlopen` 的动态库加载，支持运行时热加载
5. **Executor** - 单线程/多线程事件循环，通过 `CallbackGroup` 控制并发

## 快速导航

- 想了解 Component 是什么 → [01_overview.md](./01_overview.md)
- 想看 ComponentManager 如何动态加载 .so → [02_component_manager_source.md](./02_component_manager_source.md)
- 想了解多组件并发调度 → [03_executor_and_scheduling.md](./03_executor_and_scheduling.md)
- 想用 LifecycleNode → [04_lifecycle_and_launch.md](./04_lifecycle_and_launch.md)
- 开发中遇到问题 → [05_summary_and_cheatsheet.md](./05_summary_and_cheatsheet.md)
