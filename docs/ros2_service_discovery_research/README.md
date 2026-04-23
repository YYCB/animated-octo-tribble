# ROS2 服务发现机制深度调研 - 目录

本目录包含对 ROS2 服务发现（Service Discovery）机制的源码级深度调研，覆盖 DDS 协议原理、RTPS 实现、rmw/rcl 层封装和工程配置实践。

## 文件列表

| 文件 | 内容 | 适合读者 |
|------|------|---------|
| [01_overview_and_dds.md](./01_overview_and_dds.md) | 总览、RTPS 协议、SPDP/SEDP、FastDDS 源码分析、rmw 接口 | 入门+进阶 |
| [02_rcl_rmw_source.md](./02_rcl_rmw_source.md) | rcl/rmw 源码、双层发现机制、GraphCache、ros_discovery_info | 源码级 |
| [03_fastdds_vs_cyclonedds.md](./03_fastdds_vs_cyclonedds.md) | FastDDS vs CycloneDDS 对比、各自发现实现细节 | 对比分析 |
| [04_advanced_and_engineering.md](./04_advanced_and_engineering.md) | Action/参数发现、SROS2 安全、大规模优化、Docker/K8s | 工程实践 |
| [05_summary_and_cheatsheet.md](./05_summary_and_cheatsheet.md) | 完整流程图、速查表、排查命令、源码索引 | 速查 |

## 核心知识点

1. **去中心化** - ROS2 用 DDS 完全取代了 ROS1 的 rosmaster，无单点故障
2. **双层发现** - DDS 原生（SPDP/SEDP）+ ROS2 扩展（ros_discovery_info Topic）
3. **SPDP** - UDP 多播广播参与者信息（端口 7400 + 250*domain_id）
4. **SEDP** - 单播交换端点信息（DataWriter/Reader），含 QoS 兼容性检查
5. **GraphCache** - `rmw_dds_common` 维护的本地缓存，`ros2 node list` 等命令直接读缓存
6. **Discovery Server** - FastDDS 专有扩展，解决大规模（200+节点）的 O(n²) 握手问题

## 快速导航

- 想了解 ROS2 发现机制原理 → [01_overview_and_dds.md](./01_overview_and_dds.md)
- 想看 rmw/rcl 层如何实现 `ros2 node list` → [02_rcl_rmw_source.md](./02_rcl_rmw_source.md)
- 想对比 FastDDS 和 CycloneDDS → [03_fastdds_vs_cyclonedds.md](./03_fastdds_vs_cyclonedds.md)
- 想配置 Docker/K8s 多机发现 → [04_advanced_and_engineering.md](./04_advanced_and_engineering.md)
- 发现遇到问题要排查 → [05_summary_and_cheatsheet.md](./05_summary_and_cheatsheet.md)
