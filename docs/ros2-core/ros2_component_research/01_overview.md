# ROS2 Component（组件）机制深度调研

## 一、什么是 ROS2 Component

ROS2 Component 是一种**进程内组合（Intra-process Composition）**的节点加载机制，允许多个节点共享同一个操作系统进程，从而避免进程间通信（IPC）的序列化/反序列化开销，实现真正的零拷贝消息传递。

### 核心价值

| 特性 | 传统节点（独立进程） | Component（组合节点） |
|------|-------------------|-------------------|
| 消息传输 | 进程间，需要序列化 | 进程内，指针传递 |
| 延迟 | 高（IPC 序列化开销） | 低（直接内存访问） |
| 资源消耗 | 高（每进程独立内存） | 低（共享堆） |
| 部署灵活性 | 一次编译固定 | 运行时动态加载 |
| 调试 | 独立进程，隔离好 | 共享进程，需注意崩溃影响 |

---

## 二、架构层次总览

```
┌─────────────────────────────────────────────────────────────────┐
│                    User Application Layer                        │
│   ros2 component load / ComponentManager node / LaunchFile       │
└───────────────────────────┬─────────────────────────────────────┘
                            │
┌───────────────────────────▼─────────────────────────────────────┐
│                 rclcpp_components (ROS2 package)                  │
│  ComponentManager  │  NodeFactory  │  component_container        │
└───────────────────────────┬─────────────────────────────────────┘
                            │
┌───────────────────────────▼─────────────────────────────────────┐
│         pluginlib / class_loader (ROS2 plugin system)            │
│   ClassLoader<rclcpp::Node>  │  plugin.xml  │  ament_index       │
└───────────────────────────┬─────────────────────────────────────┘
                            │
┌───────────────────────────▼─────────────────────────────────────┐
│                   rclcpp  (C++ Client Library)                   │
│  Node  │  Executor  │  IntraProcessManager  │  Context           │
└───────────────────────────┬─────────────────────────────────────┘
                            │
┌───────────────────────────▼─────────────────────────────────────┐
│                     rcl  (C Client Library)                      │
│  rcl_node  │  rcl_subscription  │  rcl_publisher                 │
└───────────────────────────┬─────────────────────────────────────┘
                            │
┌───────────────────────────▼─────────────────────────────────────┐
│            rmw  (ROS Middleware Interface / DDS 抽象层)           │
│  rmw_fastrtps  │  rmw_cyclonedds  │  rmw_connextdds              │
└─────────────────────────────────────────────────────────────────┘
```

---

## 三、关键包列表

| 包名 | GitHub 仓库 | 功能 |
|------|------------|------|
| `rclcpp_components` | ros2/rclcpp | Component 框架核心 |
| `rclcpp` | ros2/rclcpp | C++ 节点基类，IntraProcessManager |
| `class_loader` | ros/class_loader | 动态库加载（dlopen 封装） |
| `pluginlib` | ros/pluginlib | ROS 插件系统（基于 class_loader） |
| `rcl` | ros2/rcl | C 语言 ROS 客户端库 |
| `rcl_components` | ros2/rcl | 组件容器 C 层接口 |
| `composition` | ros2/demos | 官方示例（Talker/Listener Component） |

---

## 四、Component 的生命周期

```
                 ┌─────────────────┐
                 │  .so 动态库文件  │
                 └────────┬────────┘
                          │ dlopen()
                 ┌────────▼────────┐
                 │  class_loader   │
                 │ ClassLoader<T>  │
                 └────────┬────────┘
                          │ createUniqueInstance()
                 ┌────────▼────────┐
                 │  NodeFactory    │  ← ComponentManager 调用
                 │ rclcpp_components│
                 └────────┬────────┘
                          │ 构造函数(NodeOptions)
                 ┌────────▼────────┐
                 │  Component Node │  ← 用户实现类
                 │  继承 rclcpp::Node│
                 └────────┬────────┘
                          │ add_to_executor()
                 ┌────────▼────────┐
                 │    Executor     │  ← SingleThreaded / MultiThreaded
                 └────────┬────────┘
                          │ spin()
                 ┌────────▼────────┐
                 │  消息/服务/定时器│  ← 事件循环
                 └─────────────────┘
```

---

## 五、源码目录速查

### rclcpp 仓库结构（关键路径）

```
ros2/rclcpp/
├── rclcpp/
│   ├── include/rclcpp/
│   │   ├── node.hpp                     # Node 基类声明
│   │   ├── executor.hpp                 # Executor 基类
│   │   ├── intra_process_manager.hpp    # 进程内通信管理
│   │   └── node_options.hpp             # NodeOptions（含 use_intra_process_comms）
│   └── src/
│       ├── node.cpp                     # Node 实现
│       ├── executor.cpp                 # Executor 实现
│       └── intra_process_manager.cpp    # IntraProcessManager 实现
└── rclcpp_components/
    ├── include/rclcpp_components/
    │   ├── component_manager.hpp        # ComponentManager 声明
    │   └── node_factory.hpp             # NodeFactory 接口
    └── src/
        ├── component_manager.cpp        # ComponentManager 实现 ★核心
        ├── component_container.cpp      # 容器 main 入口
        └── component_container_mt.cpp   # 多线程容器 main 入口
```
