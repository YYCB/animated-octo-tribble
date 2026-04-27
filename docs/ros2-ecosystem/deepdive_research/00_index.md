# 二期深化专题 — 总索引

> **背景**：Phase 1–11 已建立具身智能 ROS 2 架构的完整知识地图。本 Phase 12 针对 Phase 1–11 中识别的高价值开放问题，开展二期深化研究，聚焦四个专题：ros2_control 高阶特性、Nav2/MoveIt 2 高阶配置、完整数据飞轮架构，以及全局总结与实施路线图。

---

## 一、各专题背景与开放问题

| 专题 | 来源 Phase | 开放问题 |
|------|-----------|---------|
| **ros2_control 高阶** | Phase 1 | ChainableController 实战？Ruckig 如何集成？GPIO 接口如何控制夹爪？ |
| **Nav2 高阶** | Phase 5 | MPPI 参数如何系统调优？SmacPlannerLattice 运动学文件如何生成？ |
| **MoveIt 2 高阶** | Phase 4 | bio_ik 相比 KDL 的实测优势？STOMP 何时优于 OMPL？ |
| **数据飞轮二期** | Phase 3/6/9 | 如何实现在线自动筛选？Continual Learning 如何防遗忘？ |

---

## 二、文档导航表

| 文件 | 内容摘要 |
|------|---------|
| [00_index.md](./00_index.md) | 本文档：背景、开放问题、导航表 |
| [01_ros2_control_advanced.md](./01_ros2_control_advanced.md) | ChainableController 三级链式、Ruckig 实时轨迹、GPIO Interface、Semantic Components、Controller 热重启 |
| [02_nav2_advanced.md](./02_nav2_advanced.md) | MPPI 深化调优、SmacPlannerLattice 运动学、collision_monitor 配置、BT 自定义节点开发 |
| [03_moveit2_advanced.md](./03_moveit2_advanced.md) | bio_ik PSO、STOMP 随机优化、碰撞矩阵优化、Grasp Pose Detection、力控装配 |
| [04_data_flywheel.md](./04_data_flywheel.md) | 完整数据飞轮闭环、在线数据筛选、Continual Learning、DVC+MLflow 版本管理 |
| [05_summary.md](./05_summary.md) | 12 Phase 核心发现汇总表、实施路线图、开放问题清单、参考资料索引 |
