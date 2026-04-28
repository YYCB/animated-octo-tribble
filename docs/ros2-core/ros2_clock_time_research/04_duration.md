# Clock/Time（四）：Duration 类型系统与转换

## 一、Duration 类完整实现

```cpp
// rclcpp/include/rclcpp/duration.hpp

class Duration
{
public:
  // ★ 构造（纳秒整数）
  explicit Duration(int64_t nanoseconds) : nanoseconds_(nanoseconds) {}
  
  // ★ 从 std::chrono::duration 构造
  template<typename DurationT>
  Duration(DurationT duration)
  : nanoseconds_(
      std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count())
  {}
  
  // ★ 工厂方法（以秒为单位）
  static Duration from_seconds(double seconds) {
    return Duration(static_cast<int64_t>(seconds * 1e9));
  }
  static Duration from_nanoseconds(int64_t nanoseconds) {
    return Duration(nanoseconds);
  }
  static Duration from_rmw_time(rmw_time_t duration) {
    return Duration(
      RCL_S_TO_NS((int64_t)duration.sec) + (int64_t)duration.nsec);
  }
  
  // ★ 算术
  Duration operator+(const Duration & rhs) const {
    return Duration(nanoseconds_ + rhs.nanoseconds_);
  }
  Duration operator-(const Duration & rhs) const {
    return Duration(nanoseconds_ - rhs.nanoseconds_);
  }
  Duration operator*(double scale) const {
    return Duration(static_cast<int64_t>((double)nanoseconds_ * scale));
  }
  Duration operator/(double divisor) const {
    return Duration(static_cast<int64_t>((double)nanoseconds_ / divisor));
  }
  bool operator>(const Duration & rhs) const { return nanoseconds_ > rhs.nanoseconds_; }
  bool operator<(const Duration & rhs) const { return nanoseconds_ < rhs.nanoseconds_; }
  bool operator==(const Duration & rhs) const { return nanoseconds_ == rhs.nanoseconds_; }
  
  // ★ 访问
  int64_t nanoseconds() const { return nanoseconds_; }
  double seconds() const { return (double)nanoseconds_ * 1e-9; }
  
  // ★ 转换为 std::chrono
  template<typename DurationT>
  operator DurationT() const {
    return std::chrono::duration_cast<DurationT>(
      std::chrono::nanoseconds(nanoseconds_));
  }
  
  // ★ 转换为 builtin_interfaces/msg/Duration
  builtin_interfaces::msg::Duration to_msg() const {
    builtin_interfaces::msg::Duration msg;
    msg.sec = (int32_t)RCL_NS_TO_S(nanoseconds_);
    msg.nanosec = (uint32_t)(nanoseconds_ % RCL_S_TO_NS(1LL));
    return msg;
  }

private:
  int64_t nanoseconds_;
};
```

---

## 二、常用时间常量与字面量

```cpp
// ★ C++14 时间字面量（推荐）
using namespace std::chrono_literals;

rclcpp::Duration d1(100ms);     // 100毫秒
rclcpp::Duration d2(1s);        // 1秒
rclcpp::Duration d3(500us);     // 500微秒
rclcpp::Duration d4(1ns);       // 1纳秒

// ★ 从 double 秒
rclcpp::Duration d5 = rclcpp::Duration::from_seconds(0.1);  // 100ms

// ★ 定时器周期
node->create_timer(10ms, callback);    // 100Hz
node->create_timer(50ms, callback);    // 20Hz
node->create_timer(100ms, callback);   // 10Hz
node->create_timer(1s, callback);      // 1Hz

// ★ 时间比较
auto now = node->now();
auto deadline = now + rclcpp::Duration(5s);

if (node->now() > deadline) {
  RCLCPP_WARN(logger, "Deadline exceeded!");
}
```

---

## 三、时间转换速查

```cpp
// Time ↔ builtin_interfaces/msg/Time
rclcpp::Time t = rclcpp::Time(msg_time);           // msg → Time
builtin_interfaces::msg::Time msg = t.to_msg();    // Time → msg

// Duration ↔ builtin_interfaces/msg/Duration  
rclcpp::Duration d = rclcpp::Duration(msg_duration); // msg → Duration
builtin_interfaces::msg::Duration dmsg = d.to_msg(); // Duration → msg

// Time ↔ nanoseconds
rclcpp::Time t(nanoseconds);               // nanoseconds → Time
int64_t ns = t.nanoseconds();              // Time → nanoseconds

// Time ↔ seconds (double)
rclcpp::Time t = rclcpp::Time(static_cast<int64_t>(seconds * 1e9));
double sec = t.seconds();

// Duration ↔ std::chrono
rclcpp::Duration d(std::chrono::milliseconds(100));       // chrono → Duration
std::chrono::nanoseconds ns = std::chrono::nanoseconds(d.nanoseconds()); // Duration → chrono
auto ns2 = (std::chrono::nanoseconds)d;                   // 隐式转换

// tf2::TimePoint ↔ rclcpp::Time
tf2::TimePoint tp = tf2_ros::fromMsg(time_msg);
rclcpp::Time rt = rclcpp::Time(tp.time_since_epoch().count());
```

---

## 四、时间系统常见错误

```cpp
// ❌ 错误1：不同时钟类型相减
rclcpp::Time ros_t(0, 0, RCL_ROS_CLOCK);
rclcpp::Time steady_t(0, 0, RCL_STEADY_CLOCK);
auto diff = ros_t - steady_t;  // ★ 抛出异常！

// ❌ 错误2：仿真中使用 WallTimer 做控制
node->create_wall_timer(10ms, callback);  // bag 加速播放时仍然 10ms 真实时间

// ❌ 错误3：用 std::this_thread::sleep_for 代替 rate.sleep()
std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 不受 sim time 控制

// ❌ 错误4：消息时间戳用 std::chrono::system_clock
msg.header.stamp.sec = std::chrono::system_clock::now().time_since_epoch().count() / 1e9;
// ★ 应该用：
msg.header.stamp = node->now().to_msg();

// ❌ 错误5：忽略 use_sim_time 未收到第一个 /clock 时的 Time(0)
auto now = node->now();
if (now.nanoseconds() == 0) {
  // ★ 仿真时间未初始化，跳过或等待
  return;
}
```
