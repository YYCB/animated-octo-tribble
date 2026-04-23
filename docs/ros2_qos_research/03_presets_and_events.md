# ROS2 QoS（三）：预设配置与 QoS 事件

## 一、ROS2 内置预设 QoS（rmw_qos_profile_*）

```c
// rmw/include/rmw/qos_profiles.h

// ── 默认（最常用，平衡型）────────────────────────────────
// Reliability: RELIABLE
// Durability:  VOLATILE
// History:     KEEP_LAST(10)
const rmw_qos_profile_t rmw_qos_profile_default = {
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  10,
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_DURATION_UNSPECIFIED,  // deadline
  RMW_DURATION_UNSPECIFIED,  // lifespan
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_DURATION_UNSPECIFIED,  // liveliness_lease_duration
  false                       // avoid_ros_namespace_conventions
};

// ── 传感器数据（高频，允许丢失）──────────────────────────
// Reliability: BEST_EFFORT
// Durability:  VOLATILE
// History:     KEEP_LAST(5)
const rmw_qos_profile_t rmw_qos_profile_sensor_data = {
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  5,
  RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  ...
};

// ── 参数（服务类，用于 set/get parameter）─────────────────
// Reliability: RELIABLE
// Durability:  VOLATILE
// History:     KEEP_LAST(1000)
const rmw_qos_profile_t rmw_qos_profile_parameters = {
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  1000,
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  ...
};

// ── 服务（service client/server）─────────────────────────
// Reliability: RELIABLE
// Durability:  VOLATILE
// History:     KEEP_LAST(10)
const rmw_qos_profile_t rmw_qos_profile_services_default = {
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  10,
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  ...
};

// ── 参数事件（/parameter_events topic）──────────────────
// Reliability: RELIABLE
// Durability:  VOLATILE
// History:     KEEP_ALL
const rmw_qos_profile_t rmw_qos_profile_parameter_events = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  0,
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  ...
};

// ── 系统默认（由 DDS 实现决定）───────────────────────────
const rmw_qos_profile_t rmw_qos_profile_system_default = {
  RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT,
  RMW_QOS_POLICY_DEPTH_SYSTEM_DEFAULT,
  RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
  ...
};
```

---

## 二、rclcpp 预设 QoS 类

```cpp
// rclcpp/include/rclcpp/qos.hpp

// ── 默认（最推荐的起点）──────────────────────────────────
auto qos = rclcpp::QoS(10);
// 等价于：rclcpp::QoS(rclcpp::KeepLast(10)) = rmw_qos_profile_default with depth=10

// ── 传感器数据预设 ────────────────────────────────────────
auto qos = rclcpp::SensorDataQoS();
// 相当于：QoS(5).best_effort().volatile_()

// ── 服务预设 ──────────────────────────────────────────────
auto qos = rclcpp::ServicesQoS();
// 相当于：QoS(10).reliable().volatile_()

// ── 参数预设 ──────────────────────────────────────────────
auto qos = rclcpp::ParametersQoS();
// 相当于：QoS(1000).reliable().volatile_()

// ── Clock 预设（/clock topic）────────────────────────────
auto qos = rclcpp::ClockQoS();
// 相当于：SensorDataQoS()（best_effort, volatile, depth=1）

// ── SystemDefaults ────────────────────────────────────────
auto qos = rclcpp::SystemDefaultsQoS();
```

---

## 三、QoS 事件回调系统

### 3.1 什么是 QoS 事件？

当 QoS 策略被违反时，DDS 层会生成事件通知：

| 事件 | 触发方 | 含义 |
|------|-------|------|
| `deadline_missed` | Publisher/Subscriber | 截止期未满足 |
| `liveliness_lost` | Publisher | 发布者未能在租约内证明活性 |
| `liveliness_changed` | Subscriber | 发布者活性状态改变 |
| `incompatible_qos` | Publisher/Subscriber | 发现不兼容的 QoS 对端 |
| `offered_deadline_missed` | Publisher | Publisher 的截止期 |
| `requested_deadline_missed` | Subscriber | Subscriber 的截止期 |
| `message_lost` | Subscriber | 消息被丢弃（BEST_EFFORT 时序列号跳变） |

### 3.2 订阅 QoS 事件

```cpp
// 方式1：PublisherEventCallbacks
auto pub_event_callbacks = rclcpp::PublisherEventCallbacks();
pub_event_callbacks.deadline_callback = 
  [](rclcpp::QOSDeadlineOfferedInfo & event) {
    RCLCPP_WARN(logger, "Publisher deadline missed. Count: %d", 
      event.total_count);
  };
pub_event_callbacks.liveliness_callback =
  [](rclcpp::QOSLivelinessLostInfo & event) {
    RCLCPP_ERROR(logger, "Liveliness lost! Total count: %d",
      event.total_count);
  };
pub_event_callbacks.incompatible_qos_callback =
  [](rclcpp::QOSOfferedIncompatibleQoSInfo & event) {
    RCLCPP_WARN(logger, "Incompatible QoS! Policy: %d",
      event.last_policy_kind);
  };

auto pub_options = rclcpp::PublisherOptions();
pub_options.event_callbacks = pub_event_callbacks;
auto pub = node->create_publisher<Msg>("topic", 10, pub_options);

// 方式2：SubscriptionEventCallbacks
auto sub_event_callbacks = rclcpp::SubscriptionEventCallbacks();
sub_event_callbacks.deadline_callback =
  [](rclcpp::QOSDeadlineRequestedInfo & event) {
    RCLCPP_WARN(logger, "Subscriber deadline missed. Count: %d",
      event.total_count_change);
  };
sub_event_callbacks.incompatible_qos_callback =
  [](rclcpp::QOSRequestedIncompatibleQoSInfo & event) {
    RCLCPP_WARN(logger, "Incompatible QoS! Policy: %d",
      event.last_policy_kind);
  };
sub_event_callbacks.message_lost_callback =
  [](rclcpp::QOSMessageLostInfo & event) {
    RCLCPP_WARN(logger, "Message lost! Count: %d",
      event.total_count_change);
  };
```

### 3.3 QoS 事件的内部实现

```cpp
// rclcpp/include/rclcpp/qos_event.hpp

// QoS 事件本质上是一个特殊的"Waitable"，注册到 Executor 的 wait_set 中
class QOSEventHandlerBase : public Waitable
{
public:
  // 向 wait_set 添加事件句柄
  void add_to_wait_set(rcl_wait_set_t * wait_set) override {
    rcl_wait_set_add_event(wait_set, &event_handle_, nullptr);
  }
  
  bool is_ready(rcl_wait_set_t * wait_set) override {
    return wait_set->events[index_in_wait_set_] != nullptr;
  }
  
  void execute(std::shared_ptr<void> & data) override {
    // 调用事件回调
    callback_(event_info);
  }
};

// rcl 层：rcl_event_t 对应 DDS StatusCondition
```

---

## 四、"best_available" QoS（Humble+）

```cpp
// 让 ROS2 自动选择与现有发布者/订阅者最兼容的 QoS
auto qos = rclcpp::QoS(10).best_available();

// 内部实现：
// 1. 在 create_subscription 时查询已存在的 publishers 的 QoS
// 2. 选择与所有 publishers 都兼容的"最严格"的 QoS
// 3. 如果没有 publishers，使用系统默认

// 适用场景：
// - 不确定发布者使用什么 QoS 时
// - 希望自动适配不同配置的发布者
// - 录制工具（rosbag2）需要适配各种 topic
```

---

## 五、QoS 覆盖（Override）机制

ROS2 允许在启动时通过参数覆盖 QoS 设置：

```yaml
# params.yaml
my_node:
  ros__parameters:
    # QoS 覆盖（格式：qos_overrides./<topic_name>.<property>）
    qos_overrides./chatter.reliability: reliable
    qos_overrides./chatter.durability: volatile
    qos_overrides./chatter.history: keep_last
    qos_overrides./chatter.depth: 20
```

```python
# launch 文件
from launch_ros.actions import Node
Node(
  package='my_pkg',
  executable='my_node',
  parameters=[{'qos_overrides./chatter.reliability': 'best_effort'}]
)
```

```cpp
// 节点内部，NodeOptions 可以禁用 QoS 覆盖
rclcpp::NodeOptions options;
options.allow_undeclared_parameters(false);
// 订阅者创建时会自动检查并应用 QoS 覆盖参数
```

---

## 六、QoS 兼容性矩阵（完整版）

```
Reliability 兼容性：
  Publisher\Subscriber  RELIABLE  BEST_EFFORT
  RELIABLE              ✅        ✅
  BEST_EFFORT           ❌        ✅

Durability 兼容性：
  Publisher\Subscriber  TRANSIENT_LOCAL  VOLATILE
  TRANSIENT_LOCAL       ✅               ✅
  VOLATILE              ❌               ✅

Deadline 兼容性：
  Publisher.deadline >= Subscriber.deadline → ✅
  Publisher.deadline <  Subscriber.deadline → ❌
  UNSPECIFIED = 无要求（与任何值兼容）

Liveliness 兼容性：
  AUTOMATIC ≥ MANUAL_BY_TOPIC（数值越大，要求越严格）
  Publisher.liveliness >= Subscriber.liveliness → ✅
```

---

## 七、常见 QoS 配置场景

```cpp
// 1. 导航地图（静态，晚订阅者也需要收到）
auto map_qos = rclcpp::QoS(1)
  .reliability(rclcpp::ReliabilityPolicy::Reliable)
  .durability(rclcpp::DurabilityPolicy::TransientLocal);

// 2. 激光雷达点云（高频，允许丢失，减少延迟）
auto lidar_qos = rclcpp::SensorDataQoS();
// Reliability=BEST_EFFORT, Durability=VOLATILE, depth=5

// 3. 控制命令（低频，必须可靠，需要截止期监控）
auto cmd_qos = rclcpp::QoS(10)
  .reliability(rclcpp::ReliabilityPolicy::Reliable)
  .deadline(std::chrono::milliseconds(100));

// 4. 图像（大消息，高频，允许丢失）
auto image_qos = rclcpp::QoS(4)
  .reliability(rclcpp::ReliabilityPolicy::BestEffort)
  .durability(rclcpp::DurabilityPolicy::Volatile);

// 5. 状态机状态（不频繁但重要，晚订阅者需要当前状态）
auto state_qos = rclcpp::QoS(1)
  .reliability(rclcpp::ReliabilityPolicy::Reliable)
  .durability(rclcpp::DurabilityPolicy::TransientLocal);

// 6. 日志消息（可靠，大量消息）
auto log_qos = rclcpp::QoS(1000)
  .reliability(rclcpp::ReliabilityPolicy::Reliable)
  .durability(rclcpp::DurabilityPolicy::Volatile)
  .history(rclcpp::HistoryPolicy::KeepAll);
```
