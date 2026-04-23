# ROS2 tf2 坐标变换系统深度调研 - 目录

## 文件列表

| 文件 | 内容 |
|------|------|
| [01_overview.md](./01_overview.md) | tf2 架构、数学模型、坐标树概念 |
| [02_buffer_core.md](./02_buffer_core.md) | BufferCore 源码：时空插值、LookupTransform、坐标树管理 |
| [03_broadcaster.md](./03_broadcaster.md) | TransformBroadcaster/StaticBroadcaster 源码 |
| [04_listener.md](./04_listener.md) | TransformListener/Buffer 源码、tf2_ros 集成 |
| [05_summary.md](./05_summary.md) | 速查手册、常见坐标变换、调试命令 |

## 核心架构

```
发布端：
  TransformBroadcaster   → /tf  (dynamic transforms)
  StaticTransformBroadcaster → /tf_static (QoS: TRANSIENT_LOCAL)

接收端：
  TransformListener ─→ Buffer（rclcpp 层）
                           ↓
                       BufferCore（纯 C++ 数学库，无 ROS 依赖）
                           ↓
                       TimeCacheInterface（每条边的时间缓存）
                           ↓
                       CompactFrameID → TransformStorage

查询：
  Buffer::lookupTransform(target, source, time)
    → BufferCore::lookupTransformImpl()
       → walkToTopParent() 构建变换链
       → 时间插值（SLERP四元数 + 线性位置）
       → 合并变换矩阵
```
