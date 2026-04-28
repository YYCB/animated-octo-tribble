# ROS2 rcl（C 客户端库）深度调研 - 目录

`rcl`（ROS Client Library）是 ROS2 中间层的 **C 语言接口**，位于 rmw 抽象层之上、rclcpp/rclpy 之下。它是所有语言绑定的共同基础。

## 文件列表

| 文件 | 内容 | 适合读者 |
|------|------|---------|
| [01_overview.md](./01_overview.md) | rcl 架构、与 rmw/rclcpp 的关系、核心数据结构 | 入门 |
| [02_init_and_context.md](./02_init_and_context.md) | rcl_init/shutdown、Context 生命周期、信号处理 | 进阶 |
| [03_node_and_entities.md](./03_node_and_entities.md) | rcl_node、publisher、subscription、service、client 完整源码 | 源码级 |
| [04_wait_and_executor.md](./04_wait_and_executor.md) | rcl_wait_set、rcl_wait、Timer、Guard Condition | 核心机制 |
| [05_summary.md](./05_summary.md) | 速查手册、内存模型、错误处理规范 | 速查 |

## 核心定位

```
rclcpp / rclpy / rclnodejs（各语言绑定）
           ↓
          rcl（本层，C 语言，语言无关）
           ↓
          rmw（中间件抽象接口）
           ↓
     FastDDS / CycloneDDS（具体实现）
```

## 快速导航

- 了解 rcl 在 ROS2 中的位置 → [01_overview.md](./01_overview.md)
- 理解 `rclcpp::init()` 背后发生了什么 → [02_init_and_context.md](./02_init_and_context.md)
- 追踪 `rcl_publisher_publish()` 调用链 → [03_node_and_entities.md](./03_node_and_entities.md)
- 理解 `rcl_wait()` 如何阻塞等待消息 → [04_wait_and_executor.md](./04_wait_and_executor.md)
