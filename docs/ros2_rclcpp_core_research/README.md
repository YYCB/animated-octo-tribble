# ROS2 rclcpp 核心机制深度调研 - 目录

`rclcpp` 是 ROS2 的 **C++ 客户端库**，在 `rcl` C 接口之上提供了现代 C++ 的编程模型，包括模板、智能指针、lambda 回调、RAII 等特性。

## 文件列表

| 文件 | 内容 | 重要性 |
|------|------|-------|
| [01_overview.md](./01_overview.md) | rclcpp 架构、类层次、关键设计模式 | ★★★★★ |
| [02_node_lifecycle.md](./02_node_lifecycle.md) | Node 构造/析构、NodeBase/NodeTopics 等接口分层 | ★★★★★ |
| [03_publisher_subscription.md](./03_publisher_subscription.md) | Publisher/Subscription 模板实现、TypeAdapter | ★★★★★ |
| [04_callback_and_executor.md](./04_callback_and_executor.md) | 回调注册、AnyExecutable、Executor 完整实现 | ★★★★★ |
| [05_summary.md](./05_summary.md) | API 速查、常见模式、源码路径索引 | ★★★★ |

## 关键设计思想

1. **接口分离**：Node 通过多个 `NodeXxxInterface` 接口组合（发布、订阅、参数、时钟等）
2. **模板类型安全**：`Publisher<T>` 和 `Subscription<T>` 在编译期绑定消息类型
3. **RAII**：所有资源通过 `shared_ptr` 管理，析构时自动释放
4. **回调组合**：lambda、`std::function`、成员函数均可作为回调
5. **进程内通信透明**：编程模型不变，零拷贝由框架自动选择
