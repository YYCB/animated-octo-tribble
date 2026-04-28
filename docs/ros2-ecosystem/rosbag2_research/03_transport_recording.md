# rosbag2_transport 层：录制与回放

> **参考来源**：`ros2/rosbag2@humble`：`rosbag2_transport/`；`ros2/ros2cli@humble`：`ros2bag/`；`ros2/rcl@humble`：`rcl/time.h`

---

## 一、rosbag2_transport 层级定位

rosbag2_transport 是 rosbag2_cpp（存储层）和 ROS 2 中间件之间的桥梁层：

```
┌─────────────────────────────────────────────────────────────────┐
│               ros2bag CLI / 用户程序                             │
└──────────────────────┬──────────────────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────────────────┐
│               rosbag2_transport 层                               │
│                                                                 │
│  ┌─────────────────────┐    ┌──────────────────────────────┐   │
│  │   Recorder           │    │   Player                     │   │
│  │  ├─ RecordOptions    │    │  ├─ PlayOptions              │   │
│  │  ├─ QoS 兼容性处理   │    │  ├─ PlayerClock（时钟控制）  │   │
│  │  ├─ topic 发现      │    │  ├─ read-ahead buffer        │   │
│  │  └─ writer_thread_  │    │  ├─ /clock 发布              │   │
│  └─────────────────────┘    │  └─ 键盘控制（Space/S键）    │   │
│                              └──────────────────────────────┘   │
└──────────────────────┬──────────────────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────────────────┐
│               rosbag2_cpp 层（SequentialWriter/Reader）          │
└─────────────────────────────────────────────────────────────────┘
```

---

## 二、RecordOptions 完整参数说明

```cpp
// rosbag2_transport/include/rosbag2_transport/record_options.hpp

struct RecordOptions {
  // === topic 选择 ===
  bool all_topics = false;
  // true = 录制所有活跃 topic（动态发现）
  // false = 仅录制 topics 列表中的 topic

  std::vector<std::string> topics;
  // 显式指定录制的 topic 名称列表

  std::string regex = "";
  // 正则表达式过滤 topic 名（含）
  // 示例："/joint_.*" 匹配所有 joint 相关 topic

  std::string exclude_regex = "";
  // 正则表达式过滤 topic 名（排除）
  // 示例：".*/image_raw$" 排除所有 image_raw topic

  std::string exclude_topics = "";
  // 逗号分隔的 topic 名称（排除）

  // === 序列化 ===
  std::string rmw_serialization_format = "cdr";
  // 序列化格式，通常固定为 "cdr"

  // === topic 发现 ===
  std::chrono::milliseconds topic_polling_interval{100};
  // 轮询新 topic 的间隔（ms），0 = 禁用动态发现
  // 较小值：新 topic 更快被发现，但 CPU 开销稍增

  bool is_discovery_disabled = false;
  // true = 不做动态 topic 发现，仅录制 topics 列表中的已知 topic

  bool include_hidden_topics = false;
  // 是否包含隐藏 topic（名称以 ~ 开头）

  bool include_unpublished_topics = false;
  // 是否录制当前无发布者的 topic（预留槽位）

  // === 时钟 ===
  bool use_sim_time = false;
  // true = 使用 /clock 给消息打时间戳（仿真场景）
  // false = 使用 wall clock（真机录制默认）

  // === QoS 覆盖 ===
  std::string qos_profile_overrides_path;
  // QoS 覆盖文件路径（YAML 格式）
  // 解决录制节点与发布者 QoS 不兼容的问题

  std::unordered_map<std::string, rclcpp::QoS> topic_qos_profile_overrides;
  // 通过代码直接设置 per-topic QoS 覆盖

  // === 录制控制 ===
  bool start_paused = false;
  // true = 启动后立即暂停（等待手动 resume 或 Service 触发）
  // 用途：精确控制 episode 开始时间

  std::string node_prefix = "";
  // 录制节点名称前缀（多个录制节点共存时避免冲突）

  // === 压缩（传递给 StorageOptions）===
  std::string compression_format = "";      // "zstd" / "lz4"
  std::string compression_mode = "";        // "message" / "file"
  uint64_t compression_queue_size = 1;
  uint64_t compression_threads = 0;         // 0 = 自动（CPU 核数）
};
```

---

## 三、时钟模型详解

### 3.1 录制阶段的时钟

录制阶段默认使用 **wall clock**（`RCUTILS_TIME_POINT_VALUE_MAX` 不受 use_sim_time 影响）：

```
录制节点（Recorder）的时间戳来源：

默认（use_sim_time=false）：
  消息时间戳 = rcl_clock_get_now(RCL_STEADY_TIME)
             = std::chrono::steady_clock::now()（纳秒）

仿真录制（use_sim_time=true）：
  消息时间戳 = /clock topic 最新时间
             = rclcpp::Clock(RCL_ROS_TIME).now()
             （该 clock 在 use_sim_time=true 时订阅 /clock）

注意：消息的 header.stamp 由发布者填写（可能是 wall/sim）
      bag 文件中的 timestamp 是录制节点收到消息的时间
      → 两者可能略有差异（网络/中间件延迟）
```

### 3.2 回放阶段的时钟

回放时，Player 通过发布 `/clock` topic 来"驱动"整个 ROS 2 系统的时钟：

```
回放时钟架构：

┌──────────────────────────────────────────────────────────────────┐
│  rosbag2_transport::Player                                       │
│                                                                  │
│  PlayerClock（TimeControllerClock）                              │
│  ├─ reference_time_: bag 消息时间戳                              │
│  ├─ reference_wall_time_: 对应的 wall clock                     │
│  ├─ rate_: 回放速率（默认 1.0）                                   │
│  └─ now() → 计算当前仿真时间                                     │
│    = reference_time_ + (wall_now - reference_wall_time_) * rate │
│                                                                  │
│  /clock 发布器（100Hz 默认频率）：                                │
│    rosgraph_msgs::msg::Clock{clock: player_clock_.now()}         │
└────────────────────┬─────────────────────────────────────────────┘
                     │ /clock topic
┌────────────────────▼─────────────────────────────────────────────┐
│  所有设置了 use_sim_time: true 的节点                             │
│  ├─ rviz2（TF 回放）                                              │
│  ├─ Nav2 服务器（路径规划基于仿真时间）                             │
│  └─ 用户自定义分析节点                                            │
└──────────────────────────────────────────────────────────────────┘
```

### 3.3 use_sim_time 配置

```bash
# 方法1：命令行参数
ros2 bag play my_bag --clock

# 所有下游节点需要设置 use_sim_time=true：
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
ros2 run my_package my_node --ros-args -p use_sim_time:=true
```

```yaml
# 方法2：Launch 文件配置
# my_replay_analysis.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    common_params = {'use_sim_time': True}
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            parameters=[common_params]
        ),
        Node(
            package='my_analysis_pkg',
            executable='joint_analysis_node',
            parameters=[common_params]
        ),
        # rosbag2 player 通过 ros2 bag play 命令启动
        # 或通过 rosbag2_transport::Player API 启动
    ])
```

### 3.4 与 ros2_clock_time_research 的衔接

参见 `docs/ros2-core/ros2_clock_time_research/`，回放时的关键概念：

| 概念 | 真机录制 | bag 回放 |
|------|---------|---------|
| `RCL_SYSTEM_TIME` | 系统 POSIX 时间 | 系统 POSIX 时间（不变）|
| `RCL_STEADY_TIME` | 单调递增，无跳变 | 单调递增，无跳变 |
| `RCL_ROS_TIME` | = SYSTEM_TIME（默认）| = `/clock` 时间（use_sim_time=true）|
| Timer 触发 | 按 wall clock | 按 `/clock` 仿真时间 |
| TF 历史查询 | 当前 wall time | 基于 `/clock` 时间查询 |

**常见问题**：rviz2 回放时 TF 数据显示异常 → 通常是 `use_sim_time` 未正确设置，导致 TF buffer 中的时间与当前时间差超出 `tf2::Duration`（默认 10s 缓存）。

---

## 四、QoS 兼容性处理

### 4.1 QoS 不匹配的根本原因

录制节点作为**订阅者**，需要与各 topic 发布者的 QoS 兼容：

```
典型冲突场景：

发布者 QoS：
  reliability: RELIABLE
  durability:  TRANSIENT_LOCAL   ← 历史消息保留
  depth: 10

录制节点默认 QoS（sensor_data profile）：
  reliability: BEST_EFFORT       ← 不匹配！
  durability:  VOLATILE
  depth: 10

结果：QoS 不兼容 → 录制节点无法接收消息
```

### 4.2 QoS 兼容性规则（DDS 标准）

| 发布者 QoS | 订阅者需求 | 是否兼容 |
|-----------|-----------|---------|
| RELIABLE | RELIABLE | ✅ |
| RELIABLE | BEST_EFFORT | ✅（订阅者降级）|
| BEST_EFFORT | BEST_EFFORT | ✅ |
| BEST_EFFORT | RELIABLE | ❌ |
| TRANSIENT_LOCAL | VOLATILE | ✅ |
| TRANSIENT_LOCAL | TRANSIENT_LOCAL | ✅ |
| VOLATILE | TRANSIENT_LOCAL | ❌ |

### 4.3 QoS 覆盖配置

```yaml
# /config/rosbag2_qos_overrides.yaml
# 格式：per-topic QoS 覆盖（覆盖录制节点的订阅 QoS）

/joint_states:
  subscription:
    reliability: reliable        # 强制与发布者 RELIABLE 兼容
    durability: volatile
    history: keep_last
    depth: 10

/camera/color/image_raw:
  subscription:
    reliability: best_effort     # 相机通常用 BEST_EFFORT
    durability: volatile
    history: keep_last
    depth: 1

/tf:
  subscription:
    reliability: reliable
    durability: volatile
    history: keep_last
    depth: 100                   # TF 消息频率高，保留更多历史
```

```bash
# CLI 使用 QoS 覆盖
ros2 bag record -a \
  --qos-profile-overrides-path /config/rosbag2_qos_overrides.yaml \
  -o my_bag
```

### 4.4 自动 QoS 适配

Humble 中 rosbag2 引入了**自动 QoS 适配**：

```cpp
// rosbag2_transport/src/rosbag2_transport/recorder.cpp
// 自动探测发布者 QoS 并适配
auto offered_qos = get_offered_qos_for_topic(topic_name);
auto adapted_qos = adapt_request_to_offered_qos(offered_qos, requested_qos);
// → 以发布者 QoS 中更宽松的策略作为订阅 QoS
```

---

## 五、键盘控制与 Service 接口

### 5.1 录制的暂停/恢复

```
录制时的键盘控制：
  （需要终端聚焦到 ros2 bag record 进程）
  Space键 → 暂停录制（消息继续到达但不写入磁盘）
  Space键 → 恢复录制

Service 接口（程序化控制，无需键盘）：
  /rosbag2_recorder/pause    → std_srvs/srv/Trigger
  /rosbag2_recorder/resume   → std_srvs/srv/Trigger
  /rosbag2_recorder/split_bagfile → std_srvs/srv/Trigger（立即分片）
  /rosbag2_recorder/snapshot_bag  → std_srvs/srv/Trigger（仅录制最近 N 秒）
```

```bash
# 程序化暂停/恢复（用于精确 episode 控制）
ros2 service call /rosbag2_recorder/pause std_srvs/srv/Trigger
# ... 执行重置/换 episode 操作 ...
ros2 service call /rosbag2_recorder/resume std_srvs/srv/Trigger
ros2 service call /rosbag2_recorder/split_bagfile std_srvs/srv/Trigger
```

### 5.2 回放的键盘控制

```
回放时的键盘控制：
  Space 键 → 暂停/恢复回放
  S 键     → 向前跳一帧（暂停状态下）
  D 键     → 增加回放速率 × 1.1
  A 键     → 降低回放速率 × 0.9
  Ctrl+C   → 退出回放

Service/Action 接口：
  /rosbag2_player/pause     → 暂停
  /rosbag2_player/resume    → 恢复
  /rosbag2_player/stop      → 停止并退出
  /rosbag2_player/play_next → 逐帧（暂停状态）
  /rosbag2_player/set_rate  → 设置速率（Float64）
  /rosbag2_player/seek      → 跳转到指定时间戳（rosbag2_interfaces/action/Seek）
```

---

## 六、ros2 bag CLI 完整参数说明

### 6.1 ros2 bag record

```bash
ros2 bag record [OPTIONS] [-o OUTPUT] [TOPICS...]

必选参数：
  TOPICS              要录制的 topic 列表（不与 -a 同用）

选项：
  -o, --output DIR    输出目录（bag 名称）
  -a, --all           录制所有 topic（动态发现）
  --storage ID        存储插件 ID（默认: mcap）
  --serialization-format FMT  序列化格式（默认: cdr）
  --max-bag-size BYTES  按大小分片（0=不限）
  --max-bag-duration SEC  按时长分片（0=不限）
  --compression-mode MODE  压缩模式: message|file（默认: none）
  --compression-format FMT  压缩格式: zstd|lz4
  --compression-queue-size N  压缩队列深度（默认: 1）
  --compression-threads N  压缩线程数（0=自动）
  -e, --regex PATTERN  topic 正则过滤（含）
  --exclude-regex PATTERN  topic 正则过滤（排除）
  --qos-profile-overrides-path FILE  QoS 覆盖文件
  --include-hidden-topics  包含隐藏 topic
  --include-unpublished-topics  包含无发布者 topic
  --polling-interval MS  topic 发现轮询间隔（默认: 100）
  --start-paused      启动时暂停
  --use-sim-time      使用仿真时钟打时间戳
  --node-prefix PREFIX  节点名称前缀
  --storage-config-uri FILE  存储插件自定义配置文件

示例：
# 录制所有 topic，mcap 格式，1GB 分片
ros2 bag record -a -o /data/episode_001 \
  --storage mcap --max-bag-size 1073741824

# 只录制关节和图像
ros2 bag record -o /data/episode_001 \
  /joint_states /camera/color/image_raw /camera/depth/image_raw \
  --storage mcap

# 使用正则批量录制 camera 相关 topic
ros2 bag record -a -o /data/episode_001 \
  -e "/camera/.*" --exclude-regex ".*/camera_info$"
```

### 6.2 ros2 bag play

```bash
ros2 bag play [OPTIONS] BAG_PATH

选项：
  --storage ID        指定存储插件（可选，从 metadata 读取）
  --rate RATE         回放速率（默认: 1.0）
  --start-offset SEC  从第 N 秒开始回放
  --duration SEC      回放持续时长（-1=全部）
  --loop              循环回放
  --clock             发布 /clock topic（使下游节点用仿真时间）
  --clock-publish-frequency HZ  /clock 发布频率（默认: 100）
  --topics TOPIC...   只回放指定 topic
  -e, --regex PATTERN  topic 正则过滤
  --exclude-regex     topic 正则排除
  --qos-profile-overrides-path FILE
  --remap SRC:=DST    topic 重映射（可多次使用）
  --read-ahead-queue-size N  预读队列深度（默认: 1000）
  --disable-keyboard-controls  禁用键盘控制
  --node-prefix PREFIX

示例：
# 以半速回放，发布 /clock
ros2 bag play /data/episode_001 --rate 0.5 --clock

# 只回放关节状态，从第 10 秒开始
ros2 bag play /data/episode_001 \
  --topics /joint_states --start-offset 10.0

# 重映射 topic 名称（用于回放到不同机器人）
ros2 bag play /data/episode_001 \
  --remap /joint_states:=/robot2/joint_states
```

### 6.3 ros2 bag info

```bash
ros2 bag info BAG_PATH [--storage ID]

输出示例：
Files:             my_bag_0.mcap
Bag size:          2.3 GiB
Storage id:        mcap
Duration:          30.123s
Start:             Nov 14 2023 10:00:00.000 (1699920000.000)
End:               Nov 14 2023 10:00:30.123 (1699920030.123)
Messages:          450123
Topic information:
  Topic: /joint_states | Type: sensor_msgs/msg/JointState | Count: 30000 | QoS: ...
  Topic: /camera/color/image_raw | Type: sensor_msgs/msg/Image | Count: 900 | QoS: ...
  Topic: /imu/data | Type: sensor_msgs/msg/Imu | Count: 6000 | QoS: ...
```

### 6.4 ros2 bag convert

```bash
# 格式转换（sqlite3 → mcap，或添加压缩）
ros2 bag convert \
  -i /data/old_bag \          # 输入 bag（sqlite3 格式）
  -o /data/new_bag \          # 输出 bag
  --output-options convert_config.yaml

# convert_config.yaml 示例：
output_bags:
  - uri: /data/new_bag
    storage_id: mcap
    compression_format: zstd
    compression_mode: file
    all: true                  # 转换所有 topic
    # topics: [/joint_states]  # 或只转换部分 topic
```

---

## 七、Player 预加载缓存机制

### 7.1 read-ahead buffer 设计

```
Player 的预加载架构（双缓冲）：

主线程（回放控制）:
  └─ 从 play_messages_queue_ 取出消息 → 等待时间 → 发布

预读线程（prefetch_thread_）:
  └─ 循环调用 reader.read_next()
  └─ 推入 play_messages_queue_（容量 = read_ahead_queue_size）
  └─ 队列满时阻塞（等待主线程消费）

```

### 7.2 缓存深度配置建议

```
read_ahead_queue_size 的选取：

单位：消息条数（不是字节数！）

场景1：纯关节数据（1kHz，< 1KB/msg）
  → 默认 1000 条即可，内存占用 < 1MB

场景2：混合图像+关节
  → 1000 条 × (1.38MB 图像 + 200B 关节) ≈ 1.38GB
  → 调低到 100-200（--read-ahead-queue-size 100）
  → 或改用字节限制（Humble 中暂无 bytes 限制 API）

场景3：高速回放（rate > 5.0）
  → 增加到 2000-5000，防止主线程等待预读

内存估算公式：
  max_memory ≈ read_ahead_queue_size × avg_message_size_bytes
```

```bash
# 限制预读队列（避免 OOM）
ros2 bag play /data/episode_001 \
  --read-ahead-queue-size 100 \
  --topics /joint_states /imu/data  # 过滤大型 topic 也可减少内存
```

---

## 八、SIGINT 处理与优雅退出

```
录制进程的 SIGINT（Ctrl+C）处理流程：

1. OS 发送 SIGINT → rclcpp::SignalHandler 捕获
2. rclcpp::shutdown() 被调用
3. Recorder 的 rclcpp::Node 的 on_shutdown 回调触发
4. Recorder::stop() 被调用：
   4.1 writer_thread_ 入队一条 SENTINEL 消息（通知退出）
   4.2 writer_thread_.join() 等待写线程完成当前队列
   4.3 SequentialWriter::close() 被调用：
       → flush 最后 chunk（mcap）
       → 写入 Summary Section
       → 更新 metadata.yaml
5. 进程退出

超时保护（5 秒）：
  若 writer_thread_ 超时未退出（队列过深）
  → 强制 abort，可能丢失最后几条消息
  → 建议保持 max_cache_size 适中，避免写队列积压
```
