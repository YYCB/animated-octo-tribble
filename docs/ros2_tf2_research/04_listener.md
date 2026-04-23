# tf2（四）：高级特性与集成

## 一、time travel lookupTransform

```cpp
// 问题场景：
// 机器人在 t=1s 捕获 source_frame 数据
// 在 t=2s 想将这些数据变换到 target_frame（t=2s 时刻的位置）
// 机器人在 1~2s 间移动了，直接用 t=1s 的 target_frame 不准确

// 解决：两步变换（time travel）
//   source(t=1s) → fixed_frame(t=1s) → fixed_frame(t=2s) → target(t=2s)
//   fixed_frame 通常是 world 或 odom（不随时间漂移的参考帧）

geometry_msgs::msg::TransformStamped
Buffer::lookupTransform(
  const std::string & target_frame,
  const tf2::TimePoint & target_time,    // t=2s
  const std::string & source_frame,
  const tf2::TimePoint & source_time,    // t=1s
  const std::string & fixed_frame)       // "world"
{
  // Step1: source(source_time) → fixed_frame(source_time)
  tf2::Transform T1 = lookupTransformImpl(fixed_frame, source_frame, source_time);
  
  // Step2: fixed_frame(source_time) → fixed_frame(target_time)
  // （对于固定帧，这个变换应该是单位变换；但对于 odom 等可能有漂移的帧，有意义）
  tf2::Transform T2 = lookupTransformImpl(target_frame, fixed_frame, target_time);
  
  // 合并
  return toMsg(T2 * T1);
}
```

---

## 二、tf2 与 doTransform — 类型安全变换

```cpp
// tf2_geometry_msgs/include/tf2_geometry_msgs/tf2_geometry_msgs.hpp

// ★ doTransform：将任意 geometry_msgs 类型从 source 坐标系变换到 target 坐标系

// 1. 变换 PointStamped
void doTransform(
  const geometry_msgs::msg::PointStamped & t_in,
  geometry_msgs::msg::PointStamped & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  tf2::Transform tf2_transform;
  fromMsg(transform.transform, tf2_transform);  // 转为 tf2::Transform
  
  tf2::Vector3 point;
  fromMsg(t_in.point, point);
  
  tf2::Vector3 result = tf2_transform * point;  // ★ 变换应用
  
  toMsg(result, t_out.point);
  t_out.header.frame_id = transform.header.frame_id;
  t_out.header.stamp = transform.header.stamp;
}

// 2. 变换 PoseStamped（旋转也会变换）
void doTransform(
  const geometry_msgs::msg::PoseStamped & t_in,
  geometry_msgs::msg::PoseStamped & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  tf2::Transform tf2_transform;
  fromMsg(transform.transform, tf2_transform);
  
  tf2::Transform pose;
  fromMsg(t_in.pose, pose);
  
  tf2::Transform result = tf2_transform * pose;  // ★ 完整位姿变换
  
  toMsg(result, t_out.pose);
  t_out.header.frame_id = transform.header.frame_id;
}

// 3. 变换 Vector3Stamped（只旋转，不平移！）
void doTransform(
  const geometry_msgs::msg::Vector3Stamped & t_in,
  geometry_msgs::msg::Vector3Stamped & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  tf2::Transform tf2_transform;
  fromMsg(transform.transform, tf2_transform);
  
  tf2::Vector3 vec;
  fromMsg(t_in.vector, vec);
  
  // ★ 向量只应用旋转，不应用平移（向量没有位置）
  tf2::Vector3 result = tf2_transform.getBasis() * vec;
  
  toMsg(result, t_out.vector);
}
```

---

## 三、tf2_ros::Buffer 与 rclcpp 的时间集成

```cpp
// tf2_ros/src/buffer.cpp

// Buffer 构造时绑定 rclcpp::Clock（支持 sim time）
Buffer::Buffer(
  rclcpp::Clock::SharedPtr clock,
  tf2::Duration cache_time,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface)
: tf2::BufferCore(cache_time),
  clock_(clock)
{
  // 如果使用 sim time，tf2 的时间查询也使用仿真时间
  // rclcpp::Clock 可以是 ROSTime（使用 /clock topic）或 SystemTime
}

// lookupTransform 内部使用 clock_ 获取当前时间（用于 timeout 计算）
geometry_msgs::msg::TransformStamped
Buffer::lookupTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  const tf2::TimePoint & time,
  const tf2::Duration & timeout) const
{
  auto clock_type = clock_->get_clock_type();
  
  if (timeout != tf2::durationFromSec(0.0)) {
    // 等待变换可用（使用 rclcpp::Clock 计算超时）
    auto start = tf2_ros::toMsg(clock_->now());
    
    while (!canTransform(target_frame, source_frame, time)) {
      auto elapsed = tf2_ros::toMsg(clock_->now()) - start;
      if (elapsed > timeout) {
        throw LookupException("timeout waiting for transform");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  
  return BufferCore::lookupTransform(target_frame, source_frame, time);
}
```

---

## 四、异步 waitForTransform（Humble+）

```cpp
// tf2_ros/include/tf2_ros/buffer.h（Humble 新增）

// 返回 future，不阻塞当前线程
tf2_ros::TransformStampedFuture
Buffer::waitForTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  const tf2::TimePoint & time,
  const tf2::Duration & timeout,
  TransformReadyCallback callback)
{
  // 内部创建一个 timer，定期检查 canTransform
  // 一旦可用，触发 callback 并完成 promise
  
  auto promise = std::make_shared<std::promise<geometry_msgs::msg::TransformStamped>>();
  auto future = promise->get_future().share();
  
  auto timer = node_->create_timer(
    std::chrono::milliseconds(10),
    [this, target_frame, source_frame, time, promise, callback, timer_ref]() mutable {
      if (canTransform(target_frame, source_frame, time)) {
        timer_ref->cancel();
        auto transform = lookupTransform(target_frame, source_frame, time);
        promise->set_value(transform);
        if (callback) callback(future);
      }
    }
  );
  
  return future;
}

// 使用方式（不阻塞 Executor）：
auto tf_future = tf_buffer_->waitForTransform(
  "odom", "base_link", tf2::TimePointZero,
  tf2::durationFromSec(1.0),
  [this](tf2_ros::TransformStampedFuture future) {
    auto transform = future.get();
    // 处理变换...
  }
);
```

---

## 五、tf2 的线程安全性

```
BufferCore 内部有 frame_mutex_（std::mutex）保护 frames_ 容器
→ setTransform（写）和 lookupTransform（读）是线程安全的

TransformListener 通常在独立线程（spin_thread=true）
→ 主节点 Executor 和 tf2 订阅回调并发执行，完全安全

注意事项：
1. lookupTransform 是同步阻塞的（带 timeout 时）
   → 不应在 Executor 回调中使用带 timeout 的 lookupTransform
   → 会阻塞 Executor，影响其他回调的处理

2. 推荐模式：
   - 高频回调（相机/雷达）：用 canTransform 检查，失败则跳过
   - 需要精确同步：用 MessageFilter

3. Time(0) 的陷阱：
   → Time(0) = 最新可用变换 ≠ 消息的时间戳
   → 如果需要与消息同步的变换，必须用消息的 header.stamp
```

---

## 六、常见使用模式

```cpp
// ✅ 模式1：最简单用法（Time(0) = 最新）
auto tf_buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

// 在回调中：
try {
  auto t = tf_buffer->lookupTransform("odom", "base_link", tf2::TimePointZero);
  // 使用 t...
} catch (tf2::TransformException & ex) {
  RCLCPP_WARN(logger, "%s", ex.what());
}

// ✅ 模式2：与消息同步（精确时间戳）
void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  try {
    auto stamp = tf2_ros::fromMsg(msg->header.stamp);
    auto t = tf_buffer->lookupTransform(
      "odom", msg->header.frame_id, stamp,
      tf2::durationFromSec(0.1));  // 等待最多 0.1s
    // 使用 t...
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_WARN(logger, "TF extrapolation: %s", ex.what());
  }
}

// ✅ 模式3：MessageFilter（推荐用于传感器数据）
// （见上节）

// ❌ 错误：在单线程 Executor 中用大 timeout
void timer_callback() {
  // ❌ 会阻塞 Executor 1 秒！
  auto t = tf_buffer->lookupTransform("odom", "base_link", 
            tf2::TimePointZero, tf2::durationFromSec(1.0));
}
```
