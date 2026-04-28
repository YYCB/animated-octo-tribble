# Clock/Time（一）：时钟架构与类型系统

## 一、ROS2 时间类型体系

```
时钟来源（3种）：
┌─────────────────────────────────────────────────────────────┐
│  RCL_SYSTEM_CLOCK                                           │
│    → clock_gettime(CLOCK_REALTIME)                         │
│    → 日历时间，受 NTP 调整影响                               │
│    → 对应 rclcpp::Clock(RCL_SYSTEM_CLOCK)                  │
│                                                             │
│  RCL_STEADY_CLOCK                                           │
│    → clock_gettime(CLOCK_MONOTONIC)                        │
│    → 单调递增，不受系统时间调整影响                           │
│    → 适合：性能计时、定时器（不需要与 ROS 时间同步）           │
│    → 对应 rclcpp::Clock(RCL_STEADY_CLOCK)                  │
│                                                             │
│  RCL_ROS_CLOCK（默认）                                       │
│    → use_sim_time=false：等同于 SYSTEM_CLOCK                │
│    → use_sim_time=true：从 /clock topic 读取仿真时间         │
│    → Node::now() 返回此时钟的时间                            │
│    → 对应 rclcpp::Clock(RCL_ROS_CLOCK)                     │
└─────────────────────────────────────────────────────────────┘

时间单位：纳秒（int64_t）
  rcl_time_point_value_t = int64_t（纳秒）
  
  rclcpp::Time {
    int32_t sec;   // 秒
    uint32_t nanosec;  // 纳秒余数
    rcl_clock_type_t clock_type;  // 标记来源（防止不同时钟混用）
  }
  
  rclcpp::Duration {
    int64_t nanoseconds;  // 有符号（可为负）
  }
```

---

## 二、rcl_clock_t 数据结构

```c
// rcl/include/rcl/time.h

typedef struct rcl_clock_s {
  rcl_clock_type_t type;           // RCL_SYSTEM_CLOCK / STEADY / ROS
  rcl_clock_jump_callbacks_t jump_callbacks;  // 时间跳跃回调（sim time 用）
  rcl_allocator_t allocator;
  
  // ★ 时间数据（仅 ROS_CLOCK 有此字段）
  rcl_ros_clock_storage_t * ros_time_override_storage;
  // 包含：is_enabled（是否启用 sim time）+ current_time（当前时钟值）
  
} rcl_clock_t;

// 获取当前时间：
rcl_ret_t
rcl_clock_get_now(rcl_clock_t * clock, rcl_time_point_value_t * time_point_value)
{
  switch (clock->type) {
    case RCL_SYSTEM_CLOCK:
      return rcl_system_clock_get_now(time_point_value);  // gettimeofday
    case RCL_STEADY_CLOCK:
      return rcl_steady_clock_get_now(time_point_value);  // CLOCK_MONOTONIC
    case RCL_ROS_CLOCK:
      if (clock->ros_time_override_storage->is_enabled) {
        // ★ 返回仿真时间
        *time_point_value = clock->ros_time_override_storage->current_time;
      } else {
        // 回退到系统时间
        return rcl_system_clock_get_now(time_point_value);
      }
      return RCL_RET_OK;
  }
}
```

---

## 三、rclcpp::Clock 封装

```cpp
// rclcpp/include/rclcpp/clock.hpp

class Clock
{
public:
  explicit Clock(rcl_clock_type_t clock_type = RCL_ROS_CLOCK)
  {
    // 初始化底层 rcl_clock_t
    rcl_ret_t ret = rcl_clock_init(clock_type, &rcl_clock_, &allocator_);
  }
  
  // ★ 获取当前时间
  Time now()
  {
    rcl_time_point_value_t now;
    rcl_clock_get_now(&rcl_clock_, &now);
    return Time(now, rcl_clock_.type);
  }
  
  // ★ 睡眠直到指定时间（支持 sim time）
  bool sleep_until(Time until, Context::SharedPtr context = nullptr)
  {
    if (rcl_clock_.type == RCL_STEADY_CLOCK || rcl_clock_.type == RCL_SYSTEM_CLOCK) {
      // 普通睡眠
      auto duration = until - now();
      std::this_thread::sleep_for(
        std::chrono::nanoseconds(duration.nanoseconds()));
    } else {
      // ★ ROS 时钟：等待 /clock 推进
      // 使用 condition_variable，由 TimeSource 在时间更新时通知
      std::unique_lock<std::mutex> lock(wait_mutex_);
      wait_cond_var_.wait(lock, [this, &until]() {
        return now() >= until || shutdown_requested_;
      });
    }
    return now() >= until;
  }
  
  rcl_clock_type_t get_clock_type() const;
  bool ros_time_is_active();  // 检查 sim time 是否已启用
  
  rcl_clock_t rcl_clock_;
};
```

---

## 四、rclcpp::Time 完整类

```cpp
// rclcpp/include/rclcpp/time.hpp

class Time
{
public:
  // ★ 构造
  Time(int32_t seconds, uint32_t nanoseconds, rcl_clock_type_t clock_type = RCL_ROS_CLOCK)
  : nanoseconds_(RCL_S_TO_NS((int64_t)seconds) + (int64_t)nanoseconds),
    clock_type_(clock_type)
  {
    if (nanoseconds < 0) throw std::runtime_error("nanoseconds must be non-negative");
  }
  
  Time(int64_t nanoseconds = 0, rcl_clock_type_t clock_type = RCL_ROS_CLOCK)
  : nanoseconds_(nanoseconds), clock_type_(clock_type) {}
  
  // ★ 从 builtin_interfaces/msg/Time 转换
  explicit Time(const builtin_interfaces::msg::Time & time_msg,
                rcl_clock_type_t clock_type = RCL_ROS_CLOCK)
  : Time(time_msg.sec, time_msg.nanosec, clock_type) {}
  
  // ★ 算术运算
  Time operator+(const Duration & rhs) const {
    return Time(nanoseconds_ + rhs.nanoseconds(), clock_type_);
  }
  Duration operator-(const Time & rhs) const {
    if (clock_type_ != rhs.clock_type_) {
      // ★ 不同时钟类型不能相减！
      throw std::runtime_error("Cannot subtract times of different clock types");
    }
    return Duration(nanoseconds_ - rhs.nanoseconds_);
  }
  
  // ★ 比较
  bool operator>(const Time & rhs) const { return nanoseconds_ > rhs.nanoseconds_; }
  bool operator<(const Time & rhs) const { return nanoseconds_ < rhs.nanoseconds_; }
  bool operator==(const Time & rhs) const {
    if (clock_type_ != rhs.clock_type_) {
      throw std::runtime_error("Cannot compare times of different clock types");
    }
    return nanoseconds_ == rhs.nanoseconds_;
  }
  
  // ★ 访问器
  int32_t seconds() const { return (int32_t)RCL_NS_TO_S(nanoseconds_); }
  uint32_t nanoseconds() const {
    return (uint32_t)(nanoseconds_ % RCL_S_TO_NS(1LL));
  }
  int64_t nanoseconds_as_int64() const { return nanoseconds_; }
  double seconds_as_double() const { return (double)nanoseconds_ * 1e-9; }
  
  // ★ 转换为 msg
  builtin_interfaces::msg::Time to_msg() const {
    builtin_interfaces::msg::Time msg;
    msg.sec = seconds();
    msg.nanosec = nanoseconds();
    return msg;
  }
  
private:
  int64_t nanoseconds_;
  rcl_clock_type_t clock_type_;
};
```

---

## 五、时间戳在消息中的使用

```cpp
// 所有带 Header 的消息都包含时间戳：
// std_msgs/msg/Header.msg
// builtin_interfaces/Time stamp  → {int32 sec, uint32 nanosec}
// string frame_id

// ★ 正确方式：用节点的时钟生成时间戳
auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
msg->header.stamp = node->now();  // ★ 使用节点时钟（可能是 sim time）
msg->header.frame_id = "lidar_link";
publisher->publish(std::move(msg));

// ★ 不要用系统时钟直接生成（sim time 下不一致）：
// msg->header.stamp = rclcpp::Clock().now();  // ❌ 绕过了 sim time
// msg->header.stamp = rclcpp::Time(0);         // ❌ 时间戳为 0

// 从 msg Header 重建 rclcpp::Time：
rclcpp::Time stamp = rclcpp::Time(msg->header.stamp);
// 或者明确时钟类型：
rclcpp::Time stamp(msg->header.stamp, rclcpp::RCL_ROS_CLOCK);
```

---

## 六、特殊时间值

```cpp
// ★ rclcpp::Time(0)：在 tf2 中代表"最新可用时间"
// 在 rclcpp 中，0 仍然是有效的时间值（epoch）

// ★ 最大/最小时间：
auto max_time = rclcpp::Time(std::numeric_limits<int64_t>::max());
auto min_time = rclcpp::Time(std::numeric_limits<int64_t>::min());

// ★ Duration 转换：
using namespace std::chrono_literals;
rclcpp::Duration dur = rclcpp::Duration(500ms);  // 500 毫秒
rclcpp::Duration dur2 = rclcpp::Duration::from_seconds(1.5);  // 1.5 秒

// ★ 从 std::chrono 转换：
auto chrono_dur = std::chrono::milliseconds(100);
rclcpp::Duration ros_dur(chrono_dur);  // 直接从 std::chrono 构造
```

---

## 七、Node::now() 内部实现

```cpp
// rclcpp/src/rclcpp/node.cpp

rclcpp::Time Node::now() const
{
  // ★ 委托给 NodeClock 接口
  return node_clock_->get_clock()->now();
}

// NodeClock 持有 rclcpp::Clock（RCL_ROS_CLOCK 类型）
// rclcpp::Clock::now() → rcl_clock_get_now() → 根据 sim time 状态返回
```
