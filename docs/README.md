# 调研文档库

本仓库用于沉淀技术调研文档。文档按主题/类型分类存放，便于持续累积与检索。

## 目录结构

```
docs/
├── README.md                  # 本文件：目录约定与新增文档指南
├── ROS2_RESEARCH_INDEX.md     # ROS 2 调研总索引
│
├── ros2-core/                 # ROS 2 框架源码深度调研（按模块组织）
│   ├── ros2_component_research/
│   ├── ros2_rcl_research/
│   ├── ros2_rclcpp_core_research/
│   ├── ros2_topic_communication_research/
│   ├── ros2_qos_research/
│   ├── ros2_service_action_research/
│   ├── ros2_tf2_research/
│   ├── ros2_parameter_system_research/
│   ├── ros2_launch_system_research/
│   ├── ros2_clock_time_research/
│   ├── ros2_lifecycle_node_research/
│   └── ros2_service_discovery_research/
│
├── ros2-ecosystem/            # ROS 2 生态：厂商 SDK / 驱动 / 周边项目分析
│   ├── orbbec_sdk_ros2_analysis/
│   ├── realsense-ros-4.57.7-analysis/
│   └── hal_comparative_analysis/
│
└── ros2-roadmaps/             # 长篇路线图、专题与跨模块综述
    └── ros2_custom_fork_roadmap/
```

> 注：仓库中的代码项目（如根目录下的 `chassis_protocol/`）将各自的设计文档放在自己仓内的 `docs/` 目录中（例如 `chassis_protocol/docs/design/`），不集中收纳到 `docs/` 下，以便与代码同步演进。

## 分类约定

| 顶层目录 | 适用内容 |
|----------|----------|
| `ros2-core/` | 针对 ROS 2 上游某个模块（rclcpp、rcl、rmw、tf2、launch 等）的源码 / 机制 / 协议层调研 |
| `ros2-ecosystem/` | 第三方基于 ROS 2 的 SDK、驱动、工具链的源码分析与对比 |
| `ros2-roadmaps/` | 跨模块的路线图、性能优化专题、长期演进规划 |
| 代码项目自带 `docs/` | 仓库中各代码项目（如 `chassis_protocol/`）的设计文档放在项目自身的 `docs/` 目录下，与代码一同维护 |

未来若出现非 ROS 2 主题（例如 Linux 内核、AI 框架等），可在 `docs/` 下新增同级分类目录（如 `linux/`、`ai/`），保持一级目录即"领域"，二级目录即"具体调研主题"的结构。

## 新增调研文档的建议流程

1. **确定分类**：参考上表选择最贴切的顶层目录；若没有合适的分类，新建一个。
2. **创建主题目录**：在选定分类下新建目录，命名采用 `小写+下划线` 或 `小写+连字符`，能清晰表达调研主题。
3. **目录内组织**：建议每个主题目录至少包含：
   - `README.md` 或 `00_index.md`：本主题概览、阅读顺序、要点速查
   - 按编号命名的章节文件，如 `01_overview.md`、`02_xxx.md`，便于线性阅读
4. **更新索引**：
   - 若属于 ROS 2 调研，在 `docs/ROS2_RESEARCH_INDEX.md` 中追加条目
   - 若开辟新领域，可在 `docs/` 下另建索引文件（如 `LINUX_RESEARCH_INDEX.md`）
5. **跨主题引用**：使用相对路径链接，例如同分类内 `../ros2_qos_research/`，跨分类 `../../ros2-core/ros2_qos_research/`。

## 现有索引

- [ROS 2 源码深度调研 — 总索引](./ROS2_RESEARCH_INDEX.md)
