# ROS2 Clock 与 Time 系统深度调研 - 目录

## 文件列表

| 文件 | 内容 |
|------|------|
| [01_overview.md](./01_overview.md) | 时钟类型、rcl_clock、rclcpp::Clock 架构 |
| [02_sim_time.md](./02_sim_time.md) | 仿真时间机制：/clock Topic、TimeSource、ROSTimer |
| [03_timer.md](./03_timer.md) | rclcpp::Timer 源码：create_timer → rcl_timer → WallTimer vs ROSTimer |
| [04_duration.md](./04_duration.md) | Duration/TimePoint 类型系统、时间比较、转换 |
| [05_summary.md](./05_summary.md) | 速查手册、常见错误 |

## 核心架构

```
时钟类型（3种）：
  RCL_SYSTEM_CLOCK   → 系统时钟（wall clock），不受 use_sim_time 影响
  RCL_STEADY_CLOCK   → 单调时钟（不可调），适合计时/性能测量
  RCL_ROS_CLOCK      → ROS 时钟（默认），可被 /clock 话题覆盖

rclcpp::Clock → rcl_clock_t → 底层时钟实现

仿真时间路径：
  Gazebo/rosbag → publish /clock （rosgraph_msgs/msg/Clock）
  TimeSource（订阅 /clock）
    → 更新 ROSTimeSource 内部时间
    → 调用 rcl_clock_t 的 time_jump 回调
    → 通知所有 ROSTimer 重新调度
```
