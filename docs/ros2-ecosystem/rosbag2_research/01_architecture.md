# rosbag2_cpp 核心架构

> **参考来源**：`ros2/rosbag2@humble`，主要包：`rosbag2_cpp/`、`rosbag2_transport/`、`rosbag2_storage/`、`rosbag2_py/`

---

## 一、包结构详解

### 1.1 rosbag2_cpp

rosbag2 的核心 C++ 库，提供与存储无关的读写 API：

```
rosbag2_cpp/
├── include/rosbag2_cpp/
│   ├── writer.hpp                  ← 公开 Writer 接口（委托给 SequentialWriter）
│   ├── reader.hpp                  ← 公开 Reader 接口（委托给 SequentialReader）
│   ├── writers/
│   │   └── sequential_writer.hpp   ← 核心写入实现
│   ├── readers/
│   │   └── sequential_reader.hpp   ← 核心读取实现
│   ├── converter.hpp               ← 序列化格式转换
│   ├── bag_events.hpp              ← 分片事件回调
│   ├── clocks/
│   │   ├── time_controller_clock.hpp   ← 回放时钟控制
│   │   └── player_clock.hpp
│   └── info.hpp                    ← bag 元数据查询
├── src/rosbag2_cpp/
│   ├── writers/sequential_writer.cpp
│   ├── readers/sequential_reader.cpp
│   └── converter.cpp
└── CMakeLists.txt
```

### 1.2 rosbag2_transport

将 rosbag2_cpp 的读写能力包装为 ROS 2 节点，处理 topic 订阅/发布、QoS、时钟：

```
rosbag2_transport/
├── include/rosbag2_transport/
│   ├── record_options.hpp     ← 录制选项结构体
│   ├── play_options.hpp       ← 回放选项结构体
│   ├── recorder.hpp           ← 录制节点封装
│   ├── player.hpp             ← 回放节点封装
│   └── qos.hpp                ← QoS 覆盖工具
└── src/rosbag2_transport/
    ├── recorder.cpp           ← RecorderImpl
    ├── player.cpp             ← PlayerImpl（read-ahead buffer）
    ├── generic_publisher.hpp  ← 序列化发布
    └── generic_subscription.hpp ← 序列化订阅
```

### 1.3 rosbag2_storage

存储抽象层，定义 `ReadOnlyInterface` 和 `ReadWriteInterface`，通过 `pluginlib` 动态加载：

```cpp
// rosbag2_storage/include/rosbag2_storage/storage_interfaces/read_write_interface.hpp
class ReadWriteInterface : public BaseReadInterface, public BaseWriteInterface {
public:
  virtual void open(const StorageOptions & storage_options,
                    IOFlag flag) = 0;
  virtual void write(std::shared_ptr<const SerializedBagMessage> message) = 0;
  virtual void write(const std::vector<std::shared_ptr<const SerializedBagMessage>> & messages) = 0;
  virtual void create_topic(const TopicMetadata & topic) = 0;
  virtual void remove_topic(const TopicMetadata & topic) = 0;
};
```

### 1.4 rosbag2_compression

提供 per-message 和 per-file 两种粒度的压缩，压缩器通过 `pluginlib` 注册：

```
rosbag2_compression/
├── include/rosbag2_compression/
│   ├── sequential_compression_writer.hpp  ← 包装 SequentialWriter
│   ├── sequential_compression_reader.hpp  ← 包装 SequentialReader
│   └── base_compressor_interface.hpp      ← 压缩器抽象
```

### 1.5 rosbag2_py

使用 pybind11 暴露 C++ 核心 API 到 Python：

```python
import rosbag2_py
# 主要暴露的类：
# rosbag2_py.SequentialReader
# rosbag2_py.SequentialWriter
# rosbag2_py.StorageOptions
# rosbag2_py.ConverterOptions
# rosbag2_py.TopicMetadata
# rosbag2_py.get_registered_readers()
# rosbag2_py.get_registered_writers()
```

### 1.6 ros2bag CLI

```
ros2bag/
└── ros2bag/verb/
    ├── record.py     ← ros2 bag record
    ├── play.py       ← ros2 bag play
    ├── info.py       ← ros2 bag info
    └── convert.py    ← ros2 bag convert
```

---

## 二、SequentialWriter 内部流程

### 2.1 open() 流程

```cpp
// rosbag2_cpp/src/rosbag2_cpp/writers/sequential_writer.cpp

void SequentialWriter::open(
  const StorageOptions & storage_options,
  const ConverterOptions & converter_options)
{
  // 1. 解析 base URI（如 "/data/bags/my_bag"）
  base_folder_ = storage_options.uri;

  // 2. 动态加载存储插件
  storage_ = storage_factory_->open_read_write(
    {uri_with_index(base_folder_, 0), storage_options.storage_id}, /* 其他选项 */);

  // 3. 若配置了压缩，包装为 SequentialCompressionWriter
  if (!storage_options.compression_format.empty()) {
    // 创建 compressor（pluginlib 加载 zstd/lz4）
  }

  // 4. 初始化 converter（CDR 转换器）
  if (converter_options.output_serialization_format != storage_options.output_serialization_format) {
    converter_ = std::make_unique<Converter>(converter_options);
  }

  // 5. 写入初始 metadata（bag_metadata.yaml）
  metadata_.storage_identifier = storage_options.storage_id;
  metadata_.starting_time = ...;
}
```

### 2.2 write() 流程

```cpp
void SequentialWriter::write(std::shared_ptr<const SerializedBagMessage> message)
{
  // 1. 如果 topic 是第一次写入，先注册 topic metadata
  if (topics_names_to_info_.find(message->topic_name) == topics_names_to_info_.end()) {
    // 不应该发生（应先调用 create_topic）
    throw std::runtime_error("...");
  }

  // 2. （可选）Converter：CDR → 目标序列化格式
  auto converted_message = converter_ ? converter_->convert(message) : message;

  // 3. 写入存储层
  storage_->write(converted_message);

  // 4. 更新统计
  metadata_.message_count++;
  topics_names_to_info_[message->topic_name].message_count++;

  // 5. 检查是否需要分片
  split_bag_if_needed();
}
```

### 2.3 split_bag_if_needed() 流程

```cpp
void SequentialWriter::split_bag_if_needed()
{
  // 检查条件：文件大小 or 录制时长
  bool split_by_size = (max_bagfile_size_ > 0 &&
    storage_->get_bagfile_size() > max_bagfile_size_);
  bool split_by_duration = (max_bagfile_duration_ > rclcpp::Duration(0) &&
    current_bag_duration() > max_bagfile_duration_);

  if (split_by_size || split_by_duration) {
    // 1. 关闭当前 storage
    storage_->close();

    // 2. 记录分片信息到 metadata_
    metadata_.relative_file_paths.push_back(current_uri_);
    split_count_++;

    // 3. 打开新的 storage（文件名递增：my_bag_0.mcap → my_bag_1.mcap）
    std::string new_uri = uri_with_index(base_folder_, split_count_);
    storage_ = storage_factory_->open_read_write({new_uri, storage_id_}, ...);

    // 4. 重新注册所有 topic metadata 到新 storage
    for (auto & topic_info : topics_names_to_info_) {
      storage_->create_topic(topic_info.second.topic_metadata);
    }

    // 5. 触发分片事件回调（用户可通过 BagSplitCallback 响应）
    if (bag_split_callback_) {
      bag_split_callback_(BagSplitInfo{...});
    }
  }
}
```

### 2.4 close() 流程

```cpp
void SequentialWriter::close()
{
  // 1. flush 未写入的数据（mcap 中 flush 最后一个 chunk）
  if (storage_) {
    storage_->close();
    storage_.reset();
  }

  // 2. 记录最后一个文件路径
  metadata_.relative_file_paths.push_back(current_uri_);

  // 3. 更新 metadata 时长和结束时间
  metadata_.duration = /* 最后消息时间戳 - 第一条消息时间戳 */;

  // 4. 写入 bag_metadata.yaml
  metadata_io_->write_metadata(base_folder_, metadata_);
}
```

---

## 三、SequentialReader 内部流程

### 3.1 open() 流程

```cpp
void SequentialReader::open(
  const StorageOptions & storage_options,
  const ConverterOptions & converter_options)
{
  // 1. 读取 bag_metadata.yaml
  metadata_ = metadata_io_->read_metadata(storage_options.uri);

  // 2. 验证存储插件与 metadata 中记录的一致
  storage_id_ = metadata_.storage_identifier;

  // 3. 打开第一个文件
  file_iterator_ = metadata_.relative_file_paths.begin();
  load_next_file();

  // 4. 初始化 converter（如需格式转换）
}

void SequentialReader::load_next_file()
{
  std::string file_path = base_folder_ / *file_iterator_;
  storage_ = storage_factory_->open_read_only(
    {file_path, storage_id_}, ...);

  // 应用 topic filter（如已设置）
  if (!topics_filter_.empty()) {
    storage_->set_filter({topics_filter_});
  }
}
```

### 3.2 read_next() 流程

```cpp
std::shared_ptr<SerializedBagMessage> SequentialReader::read_next()
{
  // 1. 检查当前文件是否还有消息
  if (!storage_->has_next()) {
    // 尝试打开下一个分片文件
    ++file_iterator_;
    if (file_iterator_ == metadata_.relative_file_paths.end()) {
      throw std::runtime_error("No more messages");
    }
    load_next_file();
  }

  // 2. 从存储层读取下一条消息
  auto message = storage_->read_next();

  // 3. （可选）Converter：源格式 → CDR
  if (converter_) {
    message = converter_->convert(message);
  }

  return message;
}
```

### 3.3 seek() 流程

```cpp
void SequentialReader::seek(const rcutils_time_point_value_t & timestamp)
{
  // mcap 插件：利用 Summary Section 的 MessageIndex 做 O(1) seek
  // sqlite3 插件：执行 "SELECT * FROM messages WHERE timestamp >= ?" 查询

  // 1. 确定目标时间戳所在的分片文件（通过 metadata 的 starting_time + duration）
  // 2. 重新打开对应文件
  // 3. 调用 storage_->seek(timestamp)
  storage_->seek(timestamp);
}
```

### 3.4 set_filter() 流程

```cpp
void SequentialReader::set_filter(const StorageFilter & storage_filter)
{
  // topic_filter_ 记录要订阅的 topic 列表
  topics_filter_ = storage_filter.topics;

  // 传递给底层 storage（在 storage 层做过滤，避免不必要的反序列化）
  if (storage_) {
    storage_->set_filter(storage_filter);
  }
}
```

---

## 四、核心数据结构

### 4.1 BagMetadata

```cpp
// rosbag2_storage/include/rosbag2_storage/bag_metadata.hpp
struct BagMetadata {
  int version = 9;                                   // metadata 版本
  std::string storage_identifier;                    // "mcap" or "sqlite3"
  std::vector<std::string> relative_file_paths;      // 所有分片文件路径（相对于 bag 目录）
  std::chrono::nanoseconds duration;                 // 总时长
  rcutils_time_point_value_t starting_time;          // 第一条消息时间戳（ns since epoch）
  uint64_t message_count = 0;                        // 总消息数
  std::vector<TopicInformation> topics_with_message_count;  // per-topic 统计
  std::string compression_format;                    // "" / "zstd" / "lz4"
  std::string compression_mode;                      // "" / "file" / "message"
  std::unordered_map<std::string, std::string> custom_data;  // 扩展字段
  std::string ros_distro;                            // "humble"
};
```

### 4.2 TopicMetadata

```cpp
// rosbag2_storage/include/rosbag2_storage/topic_metadata.hpp
struct TopicMetadata {
  std::string name;                    // e.g., "/joint_states"
  std::string type;                    // e.g., "sensor_msgs/msg/JointState"
  std::string serialization_format;   // "cdr"
  std::string offered_qos_profiles;   // YAML string of offered QoS
  std::string type_description_hash;  // SHA-256 of message definition
};
```

### 4.3 SerializedBagMessage

```cpp
// rosbag2_storage/include/rosbag2_storage/serialized_bag_message.hpp
struct SerializedBagMessage {
  std::shared_ptr<rcutils_uint8_array_t> serialized_data;  // CDR 序列化字节流
  rcutils_time_point_value_t time_stamp;                   // 消息时间戳（ns since epoch）
  std::string topic_name;                                  // topic 名称
};
```

### 4.4 MessageDefinition

```cpp
// rosbag2_storage/include/rosbag2_storage/message_definition.hpp
// 存储消息定义，用于 schema 注册（尤其 mcap 格式需要）
struct MessageDefinition {
  std::string topic_type;          // "sensor_msgs/msg/JointState"
  std::string encoding;            // "ros2msg" or "ros2idl"
  std::string encoded_message_definition;  // 完整 .msg/.idl 文件内容
  std::string type_hash;           // SHA-256 hash
};
```

---

## 五、配置选项全览

### 5.1 StorageOptions

```cpp
// rosbag2_storage/include/rosbag2_storage/storage_options.hpp
struct StorageOptions {
  std::string uri;                    // bag 目录路径（不含扩展名）
  std::string storage_id = "mcap";    // 存储插件 ID
  uint64_t max_bagfile_size = 0;      // 0 = 不限大小（字节）
  uint64_t max_bagfile_duration = 0;  // 0 = 不限时长（纳秒）
  uint64_t max_cache_size = 100 * 1024 * 1024;  // 100MB 写缓存
  std::string storage_preset_profile = "";  // 预设配置（如 "resilient"）
  std::string compression_format = "";       // "zstd" or "lz4" or ""
  std::string compression_mode = "";         // "file" or "message" or ""
  int64_t compression_queue_size = 1;       // 压缩线程队列深度
  uint64_t compression_threads = 0;         // 0 = CPU核心数
  std::unordered_map<std::string, std::string> custom_data;  // 插件自定义参数
};
```

### 5.2 RecordOptions

```cpp
// rosbag2_transport/include/rosbag2_transport/record_options.hpp
struct RecordOptions {
  bool all_topics = false;                   // 录制所有 topic
  bool is_discovery_disabled = false;        // 禁用动态 topic 发现
  std::vector<std::string> topics;           // 显式指定 topic 列表
  std::string rmw_serialization_format = "cdr";  // 序列化格式
  std::chrono::milliseconds topic_polling_interval{100};  // topic 发现轮询间隔
  std::string regex = "";                    // 正则匹配 topic 名称（含）
  std::string exclude_regex = "";            // 正则匹配 topic 名称（排除）
  std::string exclude_topics = "";           // 逗号分隔的排除 topic 列表
  bool use_sim_time = false;                 // 使用仿真时钟打时间戳
  std::string qos_profile_overrides_path;   // QoS 覆盖文件路径
  bool include_hidden_topics = false;        // 是否录制隐藏 topic（~/xxx）
  bool include_unpublished_topics = false;   // 是否录制无发布者的 topic
  std::unordered_map<std::string, rclcpp::QoS> topic_qos_profile_overrides;  // QoS 覆盖表
  bool start_paused = false;                 // 启动时即暂停（等待手动恢复）
  std::string node_prefix = "";             // 录制节点名称前缀
  std::string compression_format = "";      // 见 StorageOptions
  std::string compression_mode = "";
  uint64_t compression_queue_size = 1;
  uint64_t compression_threads = 0;
};
```

### 5.3 PlayOptions

```cpp
// rosbag2_transport/include/rosbag2_transport/play_options.hpp
struct PlayOptions {
  size_t read_ahead_queue_size = 1000;       // 预读队列深度（消息数）
  std::string node_prefix = "";
  float rate = 1.0;                          // 回放速率（1.0 = 实时，0.5 = 半速）
  std::vector<std::string> topics_to_filter; // 只回放指定 topic
  std::string regex_to_filter = "";          // 正则过滤
  std::string exclude_regex = "";
  std::unordered_map<std::string, rclcpp::QoS> topic_qos_profile_overrides;
  bool loop = false;                         // 循环回放
  std::vector<std::string> topics_to_remap;  // topic 重映射
  rcutils_duration_value_t playback_duration = -1;  // -1 = 全部回放
  rcutils_time_point_value_t start_offset = 0;       // 从指定时间戳开始
  bool disable_keyboard_controls = false;    // 禁用键盘交互（space=暂停）
  bool wait_acked_before_publish = false;    // 等待订阅者 ACK
  bool disable_loan_message = false;         // 禁用 loaned message 优化
  bool clock_publish_on_topic_publish = false;  // 与消息发布同步 /clock 更新
  float clock_publish_frequency = 0.0;      // /clock 独立发布频率（Hz）
  rcutils_duration_value_t clock_trigger_topics_delay = 0;  // 延迟发布触发
  bool publish_clock_frequency_hz = 100.0;  // 默认 100Hz /clock
};
```

---

## 六、Converter Plugin 机制

Converter 解决**不同 RMW 或不同 ROS 版本**之间的序列化格式差异。主要场景：

1. **ROS 1 bag → ROS 2**：使用 `rosbag2_bag_v2_plugins` 读取 ROS 1 rosbag 格式
2. **CDR ↔ 自定义格式**：用于与非 ROS 系统交互的 bag 文件

```cpp
// rosbag2_cpp/include/rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp
class SerializationFormatConverterInterface {
public:
  // 反序列化：CDR bytes → ROS message
  virtual void deserialize(
    std::shared_ptr<const SerializedBagMessage> serialized_message,
    const rosidl_message_type_support_t * type_support,
    std::shared_ptr<rosbag2_cpp::rosbag2_introspection_message_t> ros_message) = 0;

  // 序列化：ROS message → target format bytes
  virtual void serialize(
    std::shared_ptr<const rosbag2_cpp::rosbag2_introspection_message_t> ros_message,
    const rosidl_message_type_support_t * type_support,
    std::shared_ptr<SerializedBagMessage> serialized_message) = 0;
};
```

**pluginlib 注册方式**（`plugin_description.xml`）：
```xml
<library path="rosbag2_cpp">
  <class name="rosbag2_cpp/CDRSerializationFormatConverter"
         type="rosbag2_cpp::converters::CDRSerializationFormatConverter"
         base_class_type="rosbag2_cpp::converter_interfaces::SerializationFormatConverterInterface">
    <description>CDR format converter</description>
  </class>
</library>
```

---

## 七、rosbag2_py Python 绑定

### 7.1 读取 bag 文件

```python
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import numpy as np

def read_joint_states(bag_path: str):
    """读取 bag 文件中的 /joint_states topic"""
    reader = rosbag2_py.SequentialReader()

    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id='mcap'
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader.open(storage_options, converter_options)

    # 设置 topic 过滤
    filter = rosbag2_py.StorageFilter(topics=['/joint_states'])
    reader.set_filter(filter)

    # 获取 topic 类型信息
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    joint_data = []
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        joint_data.append({
            'timestamp': timestamp,
            'positions': list(msg.position),
            'velocities': list(msg.velocity),
            'efforts': list(msg.effort),
        })

    return joint_data
```

### 7.2 写入 bag 文件

```python
import rosbag2_py
from rclpy.serialization import serialize_message
from sensor_msgs.msg import JointState
import time

def write_joint_states(bag_path: str, joint_data: list):
    """将关节状态数据写入 bag 文件"""
    writer = rosbag2_py.SequentialWriter()

    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id='mcap',
        max_bagfile_size=1 * 1024 * 1024 * 1024,  # 1GB 分片
    )
    converter_options = rosbag2_py.ConverterOptions('cdr', 'cdr')
    writer.open(storage_options, converter_options)

    # 注册 topic
    topic_info = rosbag2_py.TopicMetadata(
        name='/joint_states',
        type='sensor_msgs/msg/JointState',
        serialization_format='cdr'
    )
    writer.create_topic(topic_info)

    for data in joint_data:
        msg = JointState()
        msg.header.stamp.nanosec = data['timestamp'] % int(1e9)
        msg.header.stamp.sec = data['timestamp'] // int(1e9)
        msg.position = data['positions']

        serialized = serialize_message(msg)
        writer.write('/joint_states', serialized, data['timestamp'])

    # writer 析构时自动 close，也可显式调用 del writer
```

### 7.3 查询 bag 元数据

```python
import rosbag2_py

def inspect_bag(bag_path: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))

    metadata = reader.get_metadata()
    print(f"Duration: {metadata.duration.nanoseconds / 1e9:.2f}s")
    print(f"Message count: {metadata.message_count}")
    print(f"Storage ID: {metadata.storage_identifier}")

    topics = reader.get_all_topics_and_types()
    for topic in topics:
        print(f"  {topic.name} [{topic.type}]")
```

---

## 八、关键接口交叉引用

| 接口 | 定义位置 | 主要实现 |
|------|---------|---------|
| `ReadWriteInterface` | `rosbag2_storage/storage_interfaces/` | mcap plugin, sqlite3 plugin |
| `SerializationFormatConverterInterface` | `rosbag2_cpp/converter_interfaces/` | CDRConverter |
| `BaseCompressorInterface` | `rosbag2_compression/` | ZstdCompressor, Lz4Compressor |
| `PlayerClock` | `rosbag2_cpp/clocks/` | TimeControllerClock |
| `BagSplitCallback` | `rosbag2_cpp/bag_events.hpp` | 用户自定义 |
