# ROS2 QoS（一）：所有 QoS 策略详解

## 一、QoS 策略总览

```
ROS2 支持以下 QoS 策略（完整列表）：

基础策略（最常用）：
  1. Reliability      可靠性：RELIABLE / BEST_EFFORT
  2. Durability       持久性：TRANSIENT_LOCAL / VOLATILE
  3. History          历史：KEEP_LAST(N) / KEEP_ALL
  4. Depth            历史深度（History=KEEP_LAST 时）

实时性策略：
  5. Deadline         截止期：消息必须在此时间内到达
  6. Liveliness       活性：发布者证明自己活跃的方式
  7. Liveliness Lease Duration  活性租约周期

资源策略：
  8. ResourceLimits   资源限制：最大样本数、实例数

扩展策略（DDS 特有，rcl 层暴露）：
  9. ContentFilteredTopic  内容过滤（仅 Subscription）
  10. Lifespan         消息生命周期（超时则丢弃）

ROS2 特有策略（基于 DDS 但有语义封装）：
  11. avoid_ros_namespace_conventions  是否遵循 ROS 命名规范
```

---

## 二、Reliability（可靠性）—— 最重要的策略

### 2.1 策略值

| 值 | 含义 | DDS 对应 |
|----|------|---------|
| `RELIABLE` | 保证消息送达，允许重传 | `RELIABLE_RELIABILITY_QOS` |
| `BEST_EFFORT` | 尽力传递，不重传 | `BEST_EFFORT_RELIABILITY_QOS` |

### 2.2 源码定义

```cpp
// rclcpp/include/rclcpp/qos.hpp（实际来自 rmw/types.h）

typedef enum RMW_PUBLIC_TYPE rmw_qos_reliability_policy_e {
  RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT = 0,
  RMW_QOS_POLICY_RELIABILITY_RELIABLE = 1,
  RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT = 2,
  RMW_QOS_POLICY_RELIABILITY_UNKNOWN = 3
} rmw_qos_reliability_policy_t;
```

### 2.3 RELIABLE 的实现机制

```
发布者（RELIABLE）：
  1. 发送 DATA 子消息
  2. 周期发送 HEARTBEAT（"我有 seq=1~N 的消息"）
  3. 收到 ACKNACK 后，重传缺失消息

订阅者（RELIABLE）：
  1. 收到 DATA，存入 ReaderHistory
  2. 检测到序列号空洞时，发送 NACK
  3. 发送 ACKNACK（确认已收到的消息）

FastDDS 配置：
  heartbeatPeriod: 3 秒（默认，建议生产环境改为 200ms）
  nackResponseDelay: 0（立即响应 NACK）
  nackSupressionDuration: 0
```

### 2.4 BEST_EFFORT 的实现

```
发布者（BEST_EFFORT）：
  1. 发送 DATA，不等确认
  2. 不发 HEARTBEAT，不处理 NACK
  
订阅者（BEST_EFFORT）：
  1. 收到即用，不追踪序列号
  2. 不发 NACK，不发 ACKNACK
  
适用场景：
  - 传感器数据（位置、速度）：最新值最重要，旧值无意义
  - 视频流：允许丢帧
  - 高频状态更新（100Hz+）：RELIABLE 的重传会增加延迟
```

---

## 三、Durability（持久性）

### 3.1 策略值

| 值 | 含义 | 典型使用场景 |
|----|------|------------|
| `VOLATILE` | 不保存历史，晚订阅者收不到已发消息 | 实时传感器数据 |
| `TRANSIENT_LOCAL` | 保存历史（depth 条），晚订阅者能收到已发消息 | 静态地图、初始配置 |

### 3.2 TRANSIENT_LOCAL 的实现

```
发布者（TRANSIENT_LOCAL）：
  DataWriter 的 WriterHistory 保存最近 depth 条消息
  
  当新订阅者加入时（SEDP 匹配）：
  1. DataWriter 检测到新的匹配 DataReader
  2. 将 WriterHistory 中所有保存的消息发送给新订阅者
  3. 然后才开始正常通信
  
  关键：即使发布者先于订阅者退出，其保存的消息也消失了
  （与 DDS Persistence Service 不同，后者可持久化到磁盘）

DDS 实现约束：
  Durability=TRANSIENT_LOCAL 必须与 Reliability=RELIABLE 配合使用
  （DDS 规范规定）
```

---

## 四、History（历史策略）

```cpp
typedef enum rmw_qos_history_policy_e {
  RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT = 0,
  RMW_QOS_POLICY_HISTORY_KEEP_LAST = 1,  // 保留最近 N 条
  RMW_QOS_POLICY_HISTORY_KEEP_ALL = 2,   // 保留所有未被订阅者取走的消息
  RMW_QOS_POLICY_HISTORY_UNKNOWN = 3
} rmw_qos_history_policy_t;
```

### 组合示例

```
History=KEEP_LAST, Depth=1, Reliability=RELIABLE：
  - 只保留最新一条（覆盖旧消息）
  - 但保证最新一条到达
  - 适合：命令速度（/cmd_vel），最新命令最重要

History=KEEP_ALL, Reliability=RELIABLE：
  - 保留所有消息直到订阅者取走
  - 可能导致内存增长
  - 适合：事件日志、里程计序列

History=KEEP_LAST, Depth=10, Reliability=BEST_EFFORT：
  - 最多缓存 10 条，允许丢失
  - 适合：状态估计（每帧独立，可以丢失）
```

---

## 五、Deadline（截止期）

```cpp
// rmw/include/rmw/types.h

typedef struct rmw_qos_profile_s {
  // ...
  rmw_duration_t deadline;  // {sec, nsec}
  // ...
} rmw_qos_profile_t;

// 含义：发布者必须每隔 deadline 时间至少发布一次
//       订阅者必须每隔 deadline 时间至少收到一次

// 违反后的事件（可通过 QoS 事件回调处理）：
// 发布者：deadline_missed（我没能按时发布）
// 订阅者：deadline_missed（我没能按时收到）
```

### 使用示例

```cpp
// 要求 /cmd_vel 至少每 100ms 发布一次
auto sub_qos = rclcpp::QoS(10)
  .reliability(rclcpp::ReliabilityPolicy::Reliable)
  .deadline(std::chrono::milliseconds(100));

auto sub = node->create_subscription<geometry_msgs::msg::Twist>(
  "/cmd_vel", sub_qos,
  callback,
  rclcpp::SubscriptionOptions()
    .event_callbacks(rclcpp::SubscriptionEventCallbacks()
      .deadline_callback([](rclcpp::QOSDeadlineRequestedInfo & event) {
        RCLCPP_WARN(logger, "Deadline missed! Total count: %d", event.total_count);
      }))
);
```

---

## 六、Liveliness（活性）

```cpp
typedef enum rmw_qos_liveliness_policy_e {
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT = 0,
  RMW_QOS_POLICY_LIVELINESS_AUTOMATIC = 1,
    // 只要 DataWriter 存在就认为"活着"（默认）
    // DDS 底层通过 heartbeat 自动维持
  RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC = 3,
    // 必须主动调用 assert_liveliness() 才认为"活着"
    // 适合：需要检测节点是否真的在处理数据（而不只是进程还在）
} rmw_qos_liveliness_policy_t;

// lease_duration：多久没收到活性证明则认为死亡
rmw_duration_t liveliness_lease_duration;
```

### Liveliness 与 Deadline 的区别

```
Deadline：关注消息发布的频率
  → "我期望每 100ms 收到一条命令"

Liveliness：关注发布者是否存活
  → "我需要知道发布者进程是否还在运行"

典型用法：
  机器人控制器订阅 /cmd_vel，设置 Liveliness=MANUAL_BY_TOPIC
  操控端发布 /cmd_vel，定期调用 assert_liveliness()
  如果操控端卡死（不再 assert），控制器收到 liveliness_lost 事件
  → 控制器立即停车（安全行为）
```

---

## 七、Lifespan（消息生命周期）

```cpp
rmw_duration_t lifespan;  // 消息从发布到被取走的最大时间

// 如果消息在队列中等待超过 lifespan，则被自动丢弃
// 适合：时效性强的消息（如导航路径规划结果）
// 防止：订阅者积压了大量过期消息后一次性处理

// 示例：
auto pub_qos = rclcpp::QoS(10)
  .reliability(rclcpp::ReliabilityPolicy::Reliable)
  .lifespan(std::chrono::seconds(1));  // 消息1秒后过期

// 注意：DDS 中 lifespan 由发布者设置，影响其 DataWriter 队列中的消息
```

---

## 八、ResourceLimits（资源限制）

```cpp
// 防止 KEEP_ALL 导致的内存无限增长
typedef struct rmw_qos_resource_limits_s {
  size_t max_samples;           // 最大样本数（-1 = 无限）
  size_t max_instances;         // 最大实例数（-1 = 无限，Topic 类型使用实例时）
  size_t max_samples_per_instance; // 每个实例的最大样本数
} rmw_qos_resource_limits_t;

// 使用示例（KEEP_ALL + 限制内存）：
auto qos = rclcpp::QoS(rclcpp::KeepAll())
  .resource_limits(rclcpp::QoSResourceLimits()
    .max_samples(1000));
```

---

## 九、内容过滤（ContentFilteredTopic）

ROS2 Humble+，允许订阅者在 DDS 层过滤消息（减少不需要的反序列化）：

```cpp
// 只接收 x > 0 的消息（DDS 在传输层过滤）
auto sub_options = rclcpp::SubscriptionOptions();
sub_options.content_filter_options.filter_expression = "x > %0";
sub_options.content_filter_options.expression_parameters = {"0"};

auto sub = node->create_subscription<geometry_msgs::msg::Point>(
  "position", 10, callback, sub_options);

// SQL92 子集语法：
// "x > 0 AND y < 10"
// "data LIKE 'prefix%'"（字符串类型）
// "id = %0"（参数化，运行时可修改）

// 修改过滤条件（运行时）：
sub->set_content_filter("x > %0 AND y > %1", {"0", "0"});
```
