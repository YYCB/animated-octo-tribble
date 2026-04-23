# ROS2 Topic 通信调用链深度调研 - 目录

本文档追踪一条消息从 `pub->publish(msg)` 到用户回调 `callback(msg)` 的**完整代码路径**，跨越 rclcpp → rcl → rmw → FastDDS 四层。

## 文件列表

| 文件 | 内容 |
|------|------|
| [01_overview.md](./01_overview.md) | 全链路架构图、数据流概述、层间接口 |
| [02_publish_path.md](./02_publish_path.md) | 发布路径：rclcpp publish → rcl → rmw → DDS DataWriter |
| [03_receive_path.md](./03_receive_path.md) | 接收路径：DDS DataReader → rmw take → rcl take → 用户回调 |
| [04_serialization.md](./04_serialization.md) | CDR 序列化、rosidl 类型支持、IDL 编译生成代码 |
| [05_summary.md](./05_summary.md) | 完整时序图、性能瓶颈、零拷贝路径对比 |

## 最重要的结论

```
publish() 调用链（深度：6层）：
  rclcpp::Publisher<T>::publish()
    → [序列化 MessageT → CDR 字节]
    → rcl_publish()（委托）
    → rmw_publish()（委托）
    → DataWriter::write()（FastDDS）
    → [DDS 网络传输 / 进程内队列]

take() 调用链（深度：6层）：
  Executor::execute_subscription()
    → rcl_take()（委托）
    → rmw_take()（委托）
    → DataReader::read()（FastDDS）
    → [CDR 字节 → 反序列化 MessageT]
    → user_callback(msg)
```
