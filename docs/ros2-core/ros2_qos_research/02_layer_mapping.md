# ROS2 QoS（二）：三层映射源码详解

## 一、rclcpp::QoS 类实现

```cpp
// rclcpp/include/rclcpp/qos.hpp

class QoS
{
public:
  // 从深度构造（最简单方式，History=KEEP_LAST, Reliability=RELIABLE）
  explicit QoS(
    QoSInitialization qos_initialization,
    const rmw_qos_profile_t & initial_profile = rmw_qos_profile_default)
  : rmw_qos_profile_(initial_profile)
  {
    // qos_initialization 可以是 KeepLast(N) 或 KeepAll()
    if (qos_initialization.history_policy == HistoryPolicy::KeepLast) {
      rmw_qos_profile_.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
      rmw_qos_profile_.depth = qos_initialization.depth;
    } else {
      rmw_qos_profile_.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
      rmw_qos_profile_.depth = 0;
    }
  }
  
  // Builder 方法（链式调用）
  QoS & reliability(rmw_qos_reliability_policy_t reliability) {
    rmw_qos_profile_.reliability = reliability;
    return *this;
  }
  QoS & reliability(ReliabilityPolicy reliability) {
    return this->reliability(static_cast<rmw_qos_reliability_policy_t>(reliability));
  }
  
  QoS & durability(DurabilityPolicy durability) {
    rmw_qos_profile_.durability = static_cast<rmw_qos_durability_policy_t>(durability);
    return *this;
  }
  
  QoS & deadline(rmw_duration_t deadline) {
    rmw_qos_profile_.deadline = deadline;
    return *this;
  }
  QoS & deadline(const std::chrono::duration<double> & deadline) {
    rmw_qos_profile_.deadline = {
      static_cast<uint64_t>(deadline.count()),
      static_cast<uint64_t>((deadline.count() - static_cast<uint64_t>(deadline.count())) * 1e9)
    };
    return *this;
  }
  
  QoS & liveliness(LivelinessPolicy liveliness) {
    rmw_qos_profile_.liveliness = static_cast<rmw_qos_liveliness_policy_t>(liveliness);
    return *this;
  }
  
  QoS & liveliness_lease_duration(rmw_duration_t liveliness_lease_duration) {
    rmw_qos_profile_.liveliness_lease_duration = liveliness_lease_duration;
    return *this;
  }
  
  QoS & lifespan(rmw_duration_t lifespan) {
    rmw_qos_profile_.lifespan = lifespan;
    return *this;
  }
  
  QoS & best_available() {
    rmw_qos_profile_.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_AVAILABLE;
    rmw_qos_profile_.durability = RMW_QOS_POLICY_DURABILITY_BEST_AVAILABLE;
    rmw_qos_profile_.deadline.nsec = RMW_QOS_DEADLINE_BEST_AVAILABLE;
    rmw_qos_profile_.liveliness = RMW_QOS_POLICY_LIVELINESS_BEST_AVAILABLE;
    rmw_qos_profile_.liveliness_lease_duration.nsec = RMW_QOS_LIVELINESS_LEASE_DURATION_BEST_AVAILABLE;
    return *this;
  }
  
  // 获取底层 rmw 结构
  const rmw_qos_profile_t & get_rmw_qos_profile() const { return rmw_qos_profile_; }
  rmw_qos_profile_t & get_rmw_qos_profile() { return rmw_qos_profile_; }

private:
  rmw_qos_profile_t rmw_qos_profile_;
};
```

---

## 二、rmw_qos_profile_t 结构

```c
// rmw/include/rmw/types.h

typedef struct rmw_qos_profile_s
{
  // ── 历史 ──────────────────────────────────────────────
  enum rmw_qos_history_policy_e history;
  size_t depth;
  
  // ── 可靠性 ────────────────────────────────────────────
  enum rmw_qos_reliability_policy_e reliability;
  
  // ── 持久性 ────────────────────────────────────────────
  enum rmw_qos_durability_policy_e durability;
  
  // ── 截止期 ────────────────────────────────────────────
  rmw_duration_t deadline;  // {sec, nsec}，RMW_DURATION_UNSPECIFIED = 不设置
  
  // ── 消息生命周期 ──────────────────────────────────────
  rmw_duration_t lifespan;
  
  // ── 活性 ──────────────────────────────────────────────
  enum rmw_qos_liveliness_policy_e liveliness;
  rmw_duration_t liveliness_lease_duration;
  
  // ── 兼容性 ────────────────────────────────────────────
  bool avoid_ros_namespace_conventions;  // 避免 ROS 命名规范（如 rt/ 前缀）
  
} rmw_qos_profile_t;

// 时间常量
#define RMW_DURATION_UNSPECIFIED {0LL, 0LL}
#define RMW_DURATION_INFINITE {9223372036LL, 854775807LL}  // INT64_MAX ns
```

---

## 三、rcl → rmw QoS 转换

```c
// rcl/src/rcl/publisher.c（publisher 初始化时）

// rcl 层不做 QoS 转换！直接将 rmw_qos_profile_t 传给 rmw 层
// rclcpp QoS → rmw_qos_profile_t 的转换在 rclcpp 层完成

// rclcpp/src/rclcpp/publisher_base.cpp：
rcl_publisher_options_t pub_options = rcl_publisher_get_default_options();
pub_options.qos = publisher_options.qos.get_rmw_qos_profile();  // 直接赋值

rcl_publisher_init(
  publisher_handle,
  node_handle,
  type_support_handle,
  topic_name,
  &pub_options  // 含 rmw_qos_profile_t
);
```

---

## 四、rmw → DDS QoS 映射（FastDDS 实现）

```cpp
// rmw_fastrtps_shared_cpp/src/qos.cpp

// 将 rmw_qos_profile_t 转换为 FastDDS DataWriterQos
bool fill_datawriterqos_from_rmw(
  eprosima::fastdds::dds::DataWriterQos & data_writer_qos,
  const rmw_qos_profile_t * qos_policies)
{
  // ── Reliability ──────────────────────────────────────
  switch (qos_policies->reliability) {
    case RMW_QOS_POLICY_RELIABILITY_RELIABLE:
      data_writer_qos.reliability().kind =
        eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS;
      break;
    case RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT:
      data_writer_qos.reliability().kind =
        eprosima::fastdds::dds::BEST_EFFORT_RELIABILITY_QOS;
      break;
    default:
      // SYSTEM_DEFAULT → FastDDS 默认（RELIABLE）
      break;
  }
  
  // ── Durability ────────────────────────────────────────
  switch (qos_policies->durability) {
    case RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:
      data_writer_qos.durability().kind =
        eprosima::fastdds::dds::TRANSIENT_LOCAL_DURABILITY_QOS;
      break;
    case RMW_QOS_POLICY_DURABILITY_VOLATILE:
      data_writer_qos.durability().kind =
        eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
      break;
  }
  
  // ── History ───────────────────────────────────────────
  switch (qos_policies->history) {
    case RMW_QOS_POLICY_HISTORY_KEEP_LAST:
      data_writer_qos.history().kind =
        eprosima::fastdds::dds::KEEP_LAST_HISTORY_QOS;
      data_writer_qos.history().depth = static_cast<int32_t>(qos_policies->depth);
      break;
    case RMW_QOS_POLICY_HISTORY_KEEP_ALL:
      data_writer_qos.history().kind =
        eprosima::fastdds::dds::KEEP_ALL_HISTORY_QOS;
      break;
  }
  
  // ── Deadline ──────────────────────────────────────────
  if (!is_time_default(qos_policies->deadline)) {
    data_writer_qos.deadline().period = convert_rmw_to_fastdds_duration(qos_policies->deadline);
  }
  
  // ── Lifespan ──────────────────────────────────────────
  if (!is_time_default(qos_policies->lifespan)) {
    data_writer_qos.lifespan().duration = convert_rmw_to_fastdds_duration(qos_policies->lifespan);
  }
  
  // ── Liveliness ────────────────────────────────────────
  switch (qos_policies->liveliness) {
    case RMW_QOS_POLICY_LIVELINESS_AUTOMATIC:
      data_writer_qos.liveliness().kind =
        eprosima::fastdds::dds::AUTOMATIC_LIVELINESS_QOS;
      break;
    case RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC:
      data_writer_qos.liveliness().kind =
        eprosima::fastdds::dds::MANUAL_BY_TOPIC_LIVELINESS_QOS;
      break;
  }
  if (!is_time_default(qos_policies->liveliness_lease_duration)) {
    data_writer_qos.liveliness().lease_duration =
      convert_rmw_to_fastdds_duration(qos_policies->liveliness_lease_duration);
  }
  
  return true;
}
```

---

## 五、DataReaderQos 转换（对称的订阅者侧）

```cpp
bool fill_datareaderqos_from_rmw(
  eprosima::fastdds::dds::DataReaderQos & data_reader_qos,
  const rmw_qos_profile_t * qos_policies)
{
  // 与 DataWriterQos 转换相同的逻辑
  // DDS 规范规定：Publisher 和 Subscriber 的 QoS 必须兼容
  // FastDDS 在 SEDP 匹配时检查兼容性，不兼容则不建立连接
  // ...
}
```

---

## 六、QoS 兼容性检查（DDS 内部）

```cpp
// FastDDS: src/cpp/fastdds/subscriber/SubscriberImpl.cpp

// 当 DataWriter 和 DataReader 发现彼此（通过 SEDP）时，
// DDS 检查 QoS 兼容性

bool qos_match(
  const DataWriterQos & writer_qos,
  const DataReaderQos & reader_qos)
{
  // 规则：Subscriber 的"期望"不能高于 Publisher 的"提供"
  
  // 1. Reliability：RELIABLE > BEST_EFFORT
  if (reader_qos.reliability().kind == RELIABLE_RELIABILITY_QOS &&
      writer_qos.reliability().kind == BEST_EFFORT_RELIABILITY_QOS) {
    return false;  // 订阅者要可靠，但发布者只是 best_effort
  }
  
  // 2. Durability：TRANSIENT_LOCAL > VOLATILE
  if (reader_qos.durability().kind == TRANSIENT_LOCAL_DURABILITY_QOS &&
      writer_qos.durability().kind == VOLATILE_DURABILITY_QOS) {
    return false;  // 订阅者要历史，但发布者不保存
  }
  
  // 3. Deadline：订阅者期望的截止期 >= 发布者承诺的截止期
  if (!deadline_compatible(writer_qos.deadline(), reader_qos.deadline())) {
    return false;
  }
  
  // 4. Liveliness：类似规则
  
  return true;
}
```

---

## 七、run_time QoS 查询（rclcpp）

```cpp
// 查询已建立连接的 Publisher/Subscriber 的实际 QoS
auto publishers = node->get_publishers_info_by_topic("/chatter");
for (auto & pub_info : publishers) {
  auto & qos = pub_info.qos_profile();
  RCLCPP_INFO(logger, "Publisher QoS: reliability=%d, durability=%d",
    qos.reliability(), qos.durability());
}

// 查询自己的 QoS
auto pub_qos = pub->get_actual_qos();
// 注意：actual_qos 可能与设置的 qos 不同（当设置了 SYSTEM_DEFAULT 时）
```

---

## 八、FastDDS XML QoS 配置（覆盖代码设置）

```xml
<!-- ~/fastdds_config.xml -->
<dds>
  <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <data_writer profile_name="default publisher profile" is_default_profile="true">
      <qos>
        <reliability>
          <kind>RELIABLE</kind>
        </reliability>
        <durability>
          <kind>VOLATILE</kind>
        </durability>
        <publish_mode>
          <kind>ASYNCHRONOUS_PUBLISH_MODE</kind>
          <flow_controller_name>default_flow_controller</flow_controller_name>
        </publish_mode>
        <history>
          <kind>KEEP_LAST</kind>
          <depth>10</depth>
        </history>
        <deadline>
          <period>
            <sec>0</sec>
            <nanosec>100000000</nanosec>  <!-- 100ms -->
          </period>
        </deadline>
      </qos>
    </data_writer>
    
    <data_reader profile_name="default subscriber profile" is_default_profile="true">
      <qos>
        <reliability>
          <kind>RELIABLE</kind>
        </reliability>
        <history>
          <kind>KEEP_LAST</kind>
          <depth>10</depth>
        </history>
      </qos>
    </data_reader>
  </profiles>
</dds>
```

```bash
# 指定配置文件
export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds_config.xml
# 或通过代码
export RMW_FASTRTPS_USE_QOS_FROM_XML=1  # 使用 XML 覆盖代码设置的 QoS
```
