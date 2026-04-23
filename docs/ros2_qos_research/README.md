# ROS2 QoS 系统三层映射深度调研 - 目录

QoS（Quality of Service）是 ROS2 中控制通信行为的关键机制，贯穿 rclcpp → rcl → rmw → DDS 四层。

## 文件列表

| 文件 | 内容 |
|------|------|
| [01_qos_policies.md](./01_qos_policies.md) | 所有 QoS 策略详解与适用场景 |
| [02_layer_mapping.md](./02_layer_mapping.md) | rclcpp QoS → rcl → rmw → DDS QoS 三层映射源码 |
| [03_presets_and_events.md](./03_presets_and_events.md) | 预设 QoS、QoS 事件、兼容性规则 |
| [04_summary.md](./04_summary.md) | 速查手册、常见配置、调试方法 |

## 核心概念

```
QoS 策略影响：
  ├── 可靠性（消息是否必须到达）
  ├── 持久性（历史消息是否保存）
  ├── 历史（保存多少条历史消息）
  ├── 截止期（消息必须在多久内送达）
  ├── 活性（发布者必须多久证明自己还活着）
  ├── 资源限制（内存使用上限）
  └── 内容过滤（订阅者只接收满足条件的消息）
```
