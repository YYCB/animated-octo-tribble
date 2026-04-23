# ROS2 QoS（四）：速查手册与调试方法

## 一、QoS 策略一句话速查

| 策略 | 选项 | 记忆口诀 |
|------|------|---------|
| Reliability | RELIABLE / BEST_EFFORT | "保证到达" vs "尽力就好" |
| Durability | TRANSIENT_LOCAL / VOLATILE | "保存历史" vs "即时即忘" |
| History | KEEP_LAST(N) / KEEP_ALL | "最新N条" vs "全部保留" |
| Deadline | duration | "多久必须来一次" |
| Liveliness | AUTOMATIC / MANUAL_BY_TOPIC | "心跳" vs "主动报活" |
| Lifespan | duration | "消息有效期" |

---

## 二、各层 QoS 类型对照表

```
rclcpp 层（C++）             rmw 层（C）                    DDS 层（FastDDS）
─────────────────────────────────────────────────────────────────────────────

ReliabilityPolicy::Reliable  RMW_QOS_POLICY_RELIABILITY_RELIABLE    RELIABLE_RELIABILITY_QOS
ReliabilityPolicy::BestEffort  RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT  BEST_EFFORT_RELIABILITY_QOS

DurabilityPolicy::TransientLocal  RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL  TRANSIENT_LOCAL_DURABILITY_QOS
DurabilityPolicy::Volatile  RMW_QOS_POLICY_DURABILITY_VOLATILE      VOLATILE_DURABILITY_QOS

HistoryPolicy::KeepLast    RMW_QOS_POLICY_HISTORY_KEEP_LAST       KEEP_LAST_HISTORY_QOS
HistoryPolicy::KeepAll     RMW_QOS_POLICY_HISTORY_KEEP_ALL        KEEP_ALL_HISTORY_QOS

LivelinessPolicy::Automatic  RMW_QOS_POLICY_LIVELINESS_AUTOMATIC   AUTOMATIC_LIVELINESS_QOS
LivelinessPolicy::ManualByTopic  RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC  MANUAL_BY_TOPIC_LIVELINESS_QOS
```

---

## 三、rclcpp QoS 链式调用速查

```cpp
// 完整的链式调用示例
auto qos = rclcpp::QoS(rclcpp::KeepLast(10))  // 或 QoS(10)
  .reliable()                   // 等价于 .reliability(ReliabilityPolicy::Reliable)
  .durability_volatile()        // 等价于 .durability(DurabilityPolicy::Volatile)
  .deadline(std::chrono::milliseconds(100))
  .liveliness(LivelinessPolicy::Automatic)
  .liveliness_lease_duration(std::chrono::seconds(5))
  .lifespan(std::chrono::seconds(1));

// 常用快捷方法
.reliable()                 // 等价于 .reliability(ReliabilityPolicy::Reliable)
.best_effort()              // 等价于 .reliability(ReliabilityPolicy::BestEffort)
.durability_volatile()      // 等价于 .durability(DurabilityPolicy::Volatile)
.transient_local()          // 等价于 .durability(DurabilityPolicy::TransientLocal)
.keep_last(N)               // History=KeepLast, Depth=N
.keep_all()                 // History=KeepAll
```

---

## 四、预设 QoS 对应的完整参数

```
名称                  reliability    durability      history        depth
─────────────────────────────────────────────────────────────────────────
QoS(10)              RELIABLE       VOLATILE        KEEP_LAST       10
SensorDataQoS()       BEST_EFFORT    VOLATILE        KEEP_LAST        5
ServicesQoS()         RELIABLE       VOLATILE        KEEP_LAST       10
ParametersQoS()       RELIABLE       VOLATILE        KEEP_LAST     1000
ClockQoS()            BEST_EFFORT    VOLATILE        KEEP_LAST        1
SystemDefaultsQoS()   SYSTEM_DEFAULT SYSTEM_DEFAULT  SYSTEM_DEFAULT   -

特殊预设：
  rclcpp::QoS(rclcpp::KeepAll())  RELIABLE VOLATILE KEEP_ALL  0

ROS 内置 topic 使用：
  /tf, /tf_static     RELIABLE  VOLATILE(tf) / TRANSIENT_LOCAL(tf_static)  KEEP_LAST/KEEP_ALL
  /clock              BEST_EFFORT  VOLATILE  KEEP_LAST(1)
  /rosout             RELIABLE  VOLATILE  KEEP_LAST(1000)
  /parameter_events   RELIABLE  VOLATILE  KEEP_ALL
  /robot_description  RELIABLE  TRANSIENT_LOCAL  KEEP_LAST(1)
```

---

## 五、QoS 调试命令

```bash
# 查看 topic 的 QoS 信息
ros2 topic info /my_topic --verbose
# 输出示例：
# Publisher count: 1
# Subscription count: 1
#
# Node name: talker
# Node namespace: /
# Topic type: std_msgs/msg/String
# Endpoint type: PUBLISHER
# GID: 01.0f.2c.38.6a.73.24.06.00.00.00.00.00.00.11.03.00.00.00.00.00.00.00.00
# QoS profile:
#   Reliability: Reliable
#   Durability: Volatile
#   Lifespan: Infinite
#   Deadline: Infinite
#   Liveliness: Automatic
#   Liveliness lease duration: Infinite

# 使用 ros2 doctor 检查 QoS 不兼容
ros2 doctor

# Python 脚本检查 QoS
python3 << 'EOF'
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

rclpy.init()
node = Node('qos_inspector')
pubs = node.get_publishers_info_by_topic('/my_topic')
for p in pubs:
    print(f"Node: {p.node_name}")
    print(f"QoS: reliability={p.qos_profile.reliability}")
node.destroy_node()
rclpy.shutdown()
EOF
```

---

## 六、QoS 不兼容的常见案例

### Case 1：BEST_EFFORT Publisher + RELIABLE Subscriber

```bash
# 现象：subscriber 收不到任何消息
# 日志中出现：
# [WARN] [rmw_fastrtps_cpp]: Incompatible QoS - offered_incompatible_qos

# 原因：DDS 规范规定，Subscriber 要求的质量 > Publisher 提供的质量时不建立连接
# 解决：将 Subscriber 改为 BEST_EFFORT
auto sub_qos = rclcpp::QoS(10).best_effort();
```

### Case 2：VOLATILE Publisher + TRANSIENT_LOCAL Subscriber

```bash
# 现象：Subscriber 启动前发布的消息收不到
# 解决：Publisher 也改为 TRANSIENT_LOCAL
auto pub_qos = rclcpp::QoS(1).reliable().transient_local();
```

### Case 3：Depth 不够导致消息丢失

```bash
# 现象：RELIABLE 模式但仍有消息丢失
# 原因：History depth 太小，发布速率 > 消费速率，导致队列溢出
# 解决：增大 depth 或优化消费速率
auto qos = rclcpp::QoS(100);  // 深度从 10 增加到 100
```

---

## 七、跨 DDS 实现的 QoS 差异

```
策略                FastDDS 默认        CycloneDDS 默认
───────────────────────────────────────────────────────
Reliability         RELIABLE            RELIABLE
Durability          VOLATILE            VOLATILE
History             KEEP_LAST(1)        KEEP_LAST(1)  [注意：DDS默认是1，rclcpp覆盖为10]
Heartbeat 周期      3秒                 100ms（更积极）
NACK 延迟           0                   5ms
内存模式            PREALLOCATED_WITH_REALLOC  动态

行为差异：
  - CycloneDDS 的 HEARTBEAT 更频繁，可靠消息的确认更快
  - FastDDS 支持 DataSharing（同机器零拷贝），CycloneDDS 支持 iceoryx
  - FastDDS 支持 ContentFilteredTopic，CycloneDDS 部分支持
```

---

## 八、源码路径速查

```
QoS 定义：
  rmw/include/rmw/types.h              ← rmw_qos_profile_t 定义
  rmw/include/rmw/qos_profiles.h       ← 预设 QoS 常量
  rclcpp/include/rclcpp/qos.hpp        ← rclcpp::QoS 类

QoS 转换：
  rmw_fastrtps_shared_cpp/src/qos.cpp  ← rmw → FastDDS QoS 转换
  rclcpp/src/rclcpp/publisher_base.cpp ← rclcpp → rcl QoS 传递

QoS 事件：
  rclcpp/include/rclcpp/qos_event.hpp  ← QoS 事件回调定义
  rclcpp/src/rclcpp/qos_event.cpp      ← 事件注册实现

QoS 覆盖（参数化）：
  rclcpp/src/rclcpp/node_interfaces/node_topics.cpp  ← QoS override 应用逻辑

GitHub 链接：
  https://github.com/ros2/rmw/blob/rolling/rmw/include/rmw/qos_profiles.h
  https://github.com/ros2/rclcpp/blob/rolling/rclcpp/include/rclcpp/qos.hpp
  https://github.com/ros2/rmw_fastrtps/blob/rolling/rmw_fastrtps_shared_cpp/src/qos.cpp
```
