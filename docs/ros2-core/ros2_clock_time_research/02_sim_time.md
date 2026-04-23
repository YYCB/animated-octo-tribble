# Clock/Time（二）：仿真时间机制

## 一、仿真时间架构

```
┌───────────────────────────────────────────────────────────────┐
│  Gazebo / rosbag2 播放器                                       │
│    publish /clock (rosgraph_msgs/msg/Clock)                   │
│    → {sec: 123, nanosec: 456789000}                           │
└─────────────────────┬─────────────────────────────────────────┘
                      │ DDS
                      ▼
┌───────────────────────────────────────────────────────────────┐
│  TimeSource（每个 Node 内部）                                   │
│    订阅 /clock topic                                           │
│    → 收到消息 → 调用 rcl_enable_ros_time_override()            │
│    → 更新 rcl_clock_t.ros_time_override_storage               │
│    → 触发 time_jump 回调（通知 ROSTimer）                      │
└─────────────────────┬─────────────────────────────────────────┘
                      │
                      ▼
┌───────────────────────────────────────────────────────────────┐
│  rcl_clock_t（RCL_ROS_CLOCK）                                  │
│    is_enabled = true                                           │
│    current_time = 仿真时间（纳秒）                              │
└─────────────────────┬─────────────────────────────────────────┘
                      │
                      ▼
              node->now() 返回仿真时间
              ros::Timer 按仿真时间触发
```

---

## 二、TimeSource 源码

```cpp
// rclcpp/src/rclcpp/time_source.cpp

class TimeSource
{
public:
  TimeSource(rclcpp::Node::SharedPtr node)
  {
    node_ = node;
    // ★ 立即检查 use_sim_time 参数
    auto use_sim_time_param = node->get_parameter("use_sim_time");
    if (use_sim_time_param.as_bool()) {
      enable_ros_time();  // 启用 sim time 模式
    }
    
    // ★ 监听 use_sim_time 参数变化（可运行时切换）
    node->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        for (auto & p : params) {
          if (p.get_name() == "use_sim_time" && p.get_type() == PARAMETER_BOOL) {
            if (p.as_bool()) {
              enable_ros_time();
            } else {
              disable_ros_time();
            }
          }
        }
        return rcl_interfaces::msg::SetParametersResult{}.successful = true;
      }
    );
  }
  
  void enable_ros_time()
  {
    // ★ Step 1: 订阅 /clock topic
    clock_subscription_ = node_->create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock",
      rclcpp::QoS(1).best_effort(),  // 仿真时钟高频，BEST_EFFORT 够用
      [this](const rosgraph_msgs::msg::Clock::SharedPtr msg) {
        clock_callback(msg);
      }
    );
    
    // ★ Step 2: 在 rcl 层启用 ROSTime 覆盖
    for (auto & clock : associated_clocks_) {
      rcl_enable_ros_time_override(clock->get_clock_handle());
    }
  }
  
  void clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
  {
    // ★ 更新 rcl_clock_t 的仿真时间
    rcl_time_point_value_t new_time = 
      RCL_S_TO_NS((int64_t)msg->clock.sec) + msg->clock.nanosec;
    
    for (auto & clock : associated_clocks_) {
      // ★ 这会触发注册到 clock 上的 time_jump 回调
      rcl_set_ros_time_override(clock->get_clock_handle(), new_time);
    }
    
    // ★ 通知所有等待 sleep_until 的线程
    {
      std::lock_guard<std::mutex> lock(wait_mutex_);
    }
    wait_cond_var_.notify_all();
  }
};
```

---

## 三、rcl_set_ros_time_override() — 更新仿真时间

```c
// rcl/src/rcl/time.c

rcl_ret_t
rcl_set_ros_time_override(
  rcl_clock_t * clock,
  rcl_time_point_value_t time_value)
{
  // ★ 计算时间跳跃（time jump）信息
  rcl_time_point_value_t old_time = clock->ros_time_override_storage->current_time;
  
  rcl_time_jump_t time_jump;
  time_jump.delta.nanoseconds = time_value - old_time;
  time_jump.clock_change = RCL_ROS_TIME_NO_CHANGE;
  
  // ★ 通知所有注册的回调（Timer 会在此重新调度）
  for each callback in clock->jump_callbacks {
    callback.callback(&time_jump, true /* before */,
                      callback.user_data);
  }
  
  // 更新时间
  clock->ros_time_override_storage->current_time = time_value;
  
  // ★ 通知 after 回调
  for each callback in clock->jump_callbacks {
    callback.callback(&time_jump, false /* after */,
                      callback.user_data);
  }
  
  return RCL_RET_OK;
}
```

---

## 四、Timer 与 sim time 的协同

```cpp
// rclcpp/src/rclcpp/executors/executor_notify_waitable.cpp
// rclcpp/src/rclcpp/timer.cpp

// ★ 创建定时器：
auto timer = node->create_wall_timer(
  std::chrono::milliseconds(100),  // 100ms 周期
  callback
);
// → WallTimer：使用 STEADY_CLOCK，不受 sim time 影响
// → 真实时间 100ms 触发一次（适合 I/O 操作）

auto timer = node->create_timer(
  std::chrono::milliseconds(100),
  callback
);
// → GenericTimer<ROS_CLOCK>：使用 RCL_ROS_CLOCK
// → use_sim_time=false 时：真实时间 100ms
// → use_sim_time=true 时：仿真时间 100ms（可能更快/更慢）

// ★ Timer 注册 time_jump 回调：
// 当 rcl_set_ros_time_override() 被调用时：
//   1. time_jump 回调触发
//   2. Timer 重新计算下次触发时间（基于新的仿真时间）
//   3. 如果仿真时间跳过了下次触发点 → 立即触发
//   4. 如果仿真时间倒退（bag 重播）→ 重置 Timer

// rcl_timer.c 中的 time_jump 回调：
void
_rcl_timer_time_jump(
  const struct rcl_clock_t * clock,
  const rcl_time_jump_t * time_jump,
  bool before_jump,
  void * user_data)
{
  rcl_timer_t * timer = (rcl_timer_t *)user_data;
  
  if (before_jump) return;  // only act after jump
  
  // ★ 重置 Timer（从新的当前时间重新开始计数）
  rcl_ret_t ret = rcl_timer_reset(timer);
}
```

---

## 五、rosbag2 播放中的时间管理

```
rosbag2_transport::Player 实现：

1. 读取 bag 文件中消息的时间戳（原始录制时间）
2. 计算与 bag 开始时间的相对偏移
3. 以 rate 倍速推进仿真时间：
   next_sim_time = bag_start + (real_elapsed * playback_rate)
4. 发布 /clock 消息（5ms 间隔或每条消息前）
5. 在正确的仿真时刻发布对应消息

时间调整参数（ros2 bag play）：
  --rate 2.0        : 2倍速播放（sim time 推进是真实时间 2 倍）
  --start-offset 10 : 从 bag 第 10 秒开始
  --clock 10        : 每隔 10 个仿真时间单位发一次 /clock（默认是 ros_hz=10Hz）

注意：
  - /clock 发布频率决定了 sim time 的分辨率
  - 高频控制器（1000Hz）在仿真中可能无法准确触发
  - 建议仿真中 /clock 发布频率 ≥ 控制器频率
```

---

## 六、use_sim_time 与 Node 初始化

```cpp
// Node 初始化时自动声明 use_sim_time 参数：
// rclcpp/src/rclcpp/node_interfaces/node_parameters.cpp

// 在 NodeParameters 构造时：
declare_parameter(
  "use_sim_time",
  false,  // 默认值 false
  rcl_interfaces::msg::ParameterDescriptor{}.set__description(
    "Use simulation clock if true")
);

// ★ 如果命令行传入 --ros-args -p use_sim_time:=true
//   → 覆盖默认值为 true
//   → TimeSource::enable_ros_time() 被调用
//   → 订阅 /clock
//   → 节点等待第一个 /clock 消息

// 等待第一个 /clock 消息的陷阱：
// node->now() 在 /clock 首条消息到达前返回 0（仿真时间未初始化）
// → 可能导致 tf 时间戳为 0 或 Timer 立即触发
// 解决：在关键操作前等待 node->now() > rclcpp::Time(0)
```

---

## 七、ROS_CLOCK vs STEADY_CLOCK 选择指南

```
选 RCL_STEADY_CLOCK（create_wall_timer）的场景：
  ✅ 周期性 I/O 操作（读取硬件、发布心跳）
  ✅ 不需要与仿真同步的操作
  ✅ 性能计时/测量

选 RCL_ROS_CLOCK（create_timer）的场景：
  ✅ 控制循环（需要在仿真中正确触发）
  ✅ 任何需要与 bag 回放同步的定时器
  ✅ 使用 node->now() 生成时间戳

混用注意：
  ❌ 不要将 STEADY_CLOCK 的时间与 ROS_CLOCK 的时间比较
  ❌ 不要用 WallTimer 驱动需要与 sim time 同步的算法
  ✅ TF 总是使用 ROS_CLOCK（支持 sim time）
  ✅ rclcpp::Rate 使用 ROS_CLOCK（支持 sim time）
```
