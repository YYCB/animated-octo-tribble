# Clock/Time 速查总结

## API 速查

| 操作 | 代码 |
|------|------|
| 获取当前时间 | `node->now()` |
| 创建 ROS 定时器 | `create_timer(100ms, cb)` |
| 创建真实时间定时器 | `create_wall_timer(100ms, cb)` |
| 控制循环频率 | `rclcpp::Rate rate(100.0); rate.sleep();` |
| 消息时间戳 | `msg->header.stamp = node->now().to_msg()` |
| 仿真时间检查 | `clock->ros_time_is_active()` |

## 时钟类型选择

| 场景 | 推荐时钟 | API |
|------|---------|-----|
| 控制循环（需仿真） | RCL_ROS_CLOCK | `create_timer` |
| I/O 心跳 | RCL_STEADY_CLOCK | `create_wall_timer` |
| 性能计时 | RCL_STEADY_CLOCK | `rclcpp::Clock(RCL_STEADY_CLOCK).now()` |
| TF 时间戳 | RCL_ROS_CLOCK | `node->now()` |

## 时间转换速查

```cpp
// nanoseconds → Time
rclcpp::Time t(nanoseconds_int64);

// seconds (double) → Time
rclcpp::Time t(static_cast<int64_t>(sec * 1e9));

// msg → Time
rclcpp::Time t(msg->header.stamp);

// Time → msg
msg->header.stamp = node->now().to_msg();

// Duration 字面量
using namespace std::chrono_literals;
rclcpp::Duration(100ms)   // 100毫秒
rclcpp::Duration(1s)      // 1秒
rclcpp::Duration::from_seconds(0.1)  // 0.1秒
```

## use_sim_time 启用流程

```
1. 命令行：--ros-args -p use_sim_time:=true
   或 Launch 文件：parameters=[{'use_sim_time': True}]
2. Node 构造时声明 use_sim_time 参数
3. TimeSource 检测到参数 → subscribe /clock
4. Gazebo/rosbag 发布 /clock
5. TimeSource 回调 → rcl_set_ros_time_override()
6. 所有 ROSTimer 按仿真时间触发
7. node->now() 返回仿真时间
```

## 调试命令

```bash
# 检查当前 ROS 时间
ros2 topic echo /clock

# 检查节点是否使用 sim time
ros2 param get /my_node use_sim_time

# 手动发布一个 clock 消息（测试用）
ros2 topic pub /clock rosgraph_msgs/msg/Clock "{clock: {sec: 100, nanosec: 0}}"

# 检查 Timer 是否正常触发（统计频率）
ros2 topic hz /my_periodic_topic
```
