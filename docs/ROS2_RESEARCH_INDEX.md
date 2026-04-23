# ROS2 源码深度调研 — 总索引

## 调研目录总览

| 目录 | 主题 | 文档数 | 核心内容 |
|------|------|--------|---------|
| [ros2_component_research](./ros2_component_research/) | Component 机制 | 5 | rclcpp_components、动态加载、ComponentContainer |
| [ros2_service_discovery_research](./ros2_service_discovery_research/) | 服务发现 | 5 | SPDP/SEDP、RTPS、参与者发现 |
| [ros2_rcl_research](./ros2_rcl_research/) | rcl C 客户端库 | 6 | 底层 C API、wait_set、rcl_init |
| [ros2_rclcpp_core_research](./ros2_rclcpp_core_research/) | rclcpp 核心 | 6 | Node/Publisher/Subscription/Executor |
| [ros2_topic_communication_research](./ros2_topic_communication_research/) | Topic 通信链路（含 IPC / Loaned / RTPS / SHM 深度剖析） | 10 | publish→DDS→subscribe 完整路径、CDR 序列化、IntraProcess、Loaned Message、RTPS 线协议、SHM 传输 |
| [ros2_qos_research](./ros2_qos_research/) | QoS 系统 | 5 | 策略详解、rclcpp→rmw→DDS 三层映射 |
| [ros2_service_action_research](./ros2_service_action_research/) | Service/Action | 6 | 请求-响应配对、Action 5-topic 状态机 |
| [ros2_tf2_research](./ros2_tf2_research/) | tf2 坐标变换 | 6 | BufferCore、LCA算法、时间插值 |
| [ros2_parameter_system_research](./ros2_parameter_system_research/) | 参数系统 | 6 | 6个Service、动态回调、AsyncClient |
| [ros2_launch_system_research](./ros2_launch_system_research/) | Launch 系统 | 6 | LaunchService、事件系统、Substitution |
| [ros2_clock_time_research](./ros2_clock_time_research/) | Clock/Time | 6 | 仿真时间、Timer源码、Duration类型 |
| [ros2_lifecycle_node_research](./ros2_lifecycle_node_research/) | Lifecycle Node | 4 | 状态机、5个回调、LifecyclePublisher |
| [ros2_custom_fork_roadmap](./ros2_custom_fork_roadmap/) | **专题**：独立维护 ROS 2 定制版本 — 性能优化路线图 | 12 | 面向中间件/系统架构师的 6 阶段路线图、12 个优化维度、ROI 决策矩阵、基准方法论、长期维护策略 |

---

## 架构层次图

```
应用层
  ┌─────────────────────────────────────────────────────────────┐
  │  rclcpp::Node / rclcpp_lifecycle::LifecycleNode             │
  │  ├── Publisher/Subscription (Topic)                         │
  │  ├── Client/Service                                         │
  │  ├── rclcpp_action::Client/Server (Action)                  │
  │  ├── Timer (ROS/Wall)                                       │
  │  ├── Parameters (NodeParameters → ParameterService)         │
  │  └── tf2_ros::Buffer/Listener/Broadcaster                  │
  └─────────────────────────────────────────────────────────────┘
           ↕ 调用
  ┌─────────────────────────────────────────────────────────────┐
  │  rcl / rcl_action / rcl_lifecycle（C 客户端库）              │
  │  ├── rcl_node_t, rcl_publisher_t, rcl_subscription_t        │
  │  ├── rcl_wait_set_t（事件多路复用）                          │
  │  ├── rcl_timer_t（时钟绑定）                                 │
  │  └── rcl_lifecycle_state_machine_t                          │
  └─────────────────────────────────────────────────────────────┘
           ↕ rmw 抽象层
  ┌─────────────────────────────────────────────────────────────┐
  │  rmw（ROS Middleware 接口）                                  │
  │  rmw_fastrtps_cpp / rmw_cyclonedds_cpp                      │
  └─────────────────────────────────────────────────────────────┘
           ↕ DDS
  ┌─────────────────────────────────────────────────────────────┐
  │  FastDDS / CycloneDDS                                        │
  │  ├── RTPS 协议（UDP/共享内存）                               │
  │  ├── SPDP/SEDP（参与者/端点发现）                           │
  │  └── CDR 序列化                                             │
  └─────────────────────────────────────────────────────────────┘
```

---

## 关键调用链索引

### Topic 发布完整链路
```
node->create_publisher<T>(name, qos)
  → NodeTopics::add_publisher()
  → rcl_publisher_init() → rmw_create_publisher() → DDS DataWriter

publisher->publish(msg)
  → 序列化（rosidl TypeSupport → CDR）
  → rcl_publish() → rmw_publish()
  → DDS DataWriter::write()
  → RTPS → UDP
```

### Service 请求完整链路
```
client->async_send_request(req) → rcl_send_request() → DDS DataWriter::write()
   ↓                                seq_num → pending_requests_[seq_num]
DDS DataReader（server side）
   ↓ Executor → execute_service()
rcl_take_request() → user_callback() → rcl_send_response()
   ↓
DDS DataReader（client side）
   ↓ Executor → execute_client()
rcl_take_response() → pending_requests_[seq_num].promise.set_value()
   ↓
future.get() 返回
```

### tf2 查询完整链路
```
buffer->lookupTransform("target", "source", time)
  → BufferCore::lookupTransform()
  → lookupTransformImpl()
  → walkToTopParent()：LCA 算法
  → 对每条边：TimeCache::getData()（SLERP 时间插值）
  → 累积变换矩阵
  → 返回 TransformStamped
```

### Lifecycle 状态转换链路
```
ros2 lifecycle set /node activate
  → ChangeState Service 请求
  → rcl_lifecycle_trigger_transition(TRANSITION_ACTIVATE)
  → 过渡到 ACTIVATING 状态
  → 调用 on_activate() 用户回调
  → SUCCESS → 转换到 ACTIVE
  → 发布 ~/transition_event
```

---

## 待研究主题（P4 — 后续计划）

| 主题 | 涉及包 | 优先级 |
|------|--------|--------|
| rosbag2 录制/回放架构 | rosbag2, rosbag2_transport | P4 |
| rosidl 消息生成系统 | rosidl_default_generators | P4 |
| rclcpp 零拷贝 API | rclcpp LoanedMessage | P4 |
| rmw 抽象层完整分析 | rmw, rmw_fastrtps | P4 |
| ros2_control 架构 | controller_manager, hardware_interface | P4 |
| MoveIt2 架构 | moveit_core, moveit_ros | P4 |
| Nav2 架构 | nav2_core, nav2_bt_navigator | P4 |
