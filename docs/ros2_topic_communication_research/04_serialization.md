# Topic 通信（四）：CDR 序列化与 rosidl 类型系统

## 一、ROS2 类型系统全景

```
.msg 文件（源码）
    │
    ▼ rosidl_generator_cpp（编译时生成）
std_msgs/msg/String.hpp（C++ 结构体）
std_msgs/msg/string.h（C 结构体）
    │
    ├─ rosidl_typesupport_cpp（编译时生成）
    │   └─ typesupport handle（get_message_type_support_handle<T>()）
    │       └─ 根据 RMW 实现动态加载正确的 typesupport 插件
    │
    ├─ rosidl_typesupport_fastrtps_cpp（编译时生成）
    │   └─ CDR 序列化/反序列化函数（使用 FastCDR）
    │
    ├─ rosidl_typesupport_introspection_cpp（编译时生成）
    │   └─ 字段名、类型、偏移量（运行时内省）
    │       └─ 用于 GenericPublisher/Subscriber、ros2 topic echo
    │
    └─ rosidl_generator_py（Python 类型）
        └─ std_msgs.msg.String（Python 类）
```

---

## 二、.msg 文件到 C++ 代码的生成

### 2.1 源文件（std_msgs/msg/String.msg）

```
string data
```

### 2.2 生成的 C++ 结构体

```cpp
// 自动生成：std_msgs/msg/detail/string__struct.hpp

namespace std_msgs
{
namespace msg
{
  struct String_
  {
    using _data_type = std::string;
    
    // ★ 消息字段
    _data_type data;
    
    // 类型别名（ROS2 惯例）
    using SharedPtr = std::shared_ptr<String_<ContainerAllocator>>;
    using ConstSharedPtr = std::shared_ptr<String_<ContainerAllocator> const>;
    using UniquePtr = std::unique_ptr<String_<ContainerAllocator>, ...>;
  };
  
  using String = String_<std::allocator<void>>;
  
} // namespace msg
} // namespace std_msgs
```

### 2.3 生成的 C 结构体

```c
// 自动生成：std_msgs/msg/detail/string__struct.h

typedef struct std_msgs__msg__String
{
  rosidl_runtime_c__String data;  // C 版本的 string
} std_msgs__msg__String;
```

### 2.4 类型支持信息（MessageMembers）

```cpp
// 自动生成：std_msgs/msg/detail/string__rosidl_typesupport_introspection_cpp.cpp

// 描述 String 消息的所有字段
static const ::rosidl_typesupport_introspection_cpp::MessageMember members[] = {
  {
    "data",                    // 字段名
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // 类型
    0,                         // 在 C++ 结构体中的偏移量（offsetof）
    false,                     // 是否数组
    nullptr,                   // 数组长度（nullptr = 动态）
    ... // 序列化/反序列化函数指针
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers message_members = {
  "std_msgs::msg",             // 包名
  "String",                    // 消息名
  1,                           // 字段数量
  members,                     // 字段描述数组
  sizeof(std_msgs::msg::String),  // 结构体大小
  init_function,               // 初始化函数
  fini_function                // 析构函数
};
```

---

## 三、CDR 序列化详解

### 3.1 CDR 数据格式规范

CDR（Common Data Representation）是 OMG 标准，RTPS/DDS 强制使用：

```
ROS2 使用的是 CDR Little-Endian（ENCAPSULATION_CDR_LE）：
  ├─ 基本类型（int32, float, etc.）：按本机字节序对齐
  ├─ 字符串：4字节长度（包含null terminator）+ 内容（UTF-8）
  ├─ 数组：元素按类型对齐，无额外长度头
  ├─ 序列（可变长数组）：4字节长度 + 元素
  └─ 结构体：各字段按类型对齐，无填充（顺序排列）

ROS2 CDR 数据包格式：
  [0x00][0x01][0x00][0x00]  ← 4字节 encapsulation header（CDR LE）
  [序列化数据...]
```

### 3.2 std_msgs::msg::String 的序列化示例

```
原始消息：String { data: "hello" }

序列化结果（十六进制）：
Offset  Value                       说明
0x00:   00 01 00 00                 encapsulation header (CDR LE)
0x04:   06 00 00 00                 string 长度 = 6（含 null terminator）
0x08:   68 65 6c 6c 6f 00          "hello\0"

总长度：10 字节
```

### 3.3 geometry_msgs::msg::Pose 的序列化示例

```
typedef struct geometry_msgs__msg__Pose {
  geometry_msgs__msg__Point position;    // x, y, z (double)
  geometry_msgs__msg__Quaternion orientation;  // x, y, z, w (double)
} geometry_msgs__msg__Pose;

序列化结果（64 字节）：
Offset  Value                       说明
0x00:   00 01 00 00                 encapsulation header
0x04:   [8 bytes]                   position.x (double)
0x0C:   [8 bytes]                   position.y (double)
0x14:   [8 bytes]                   position.z (double)
0x1C:   [8 bytes]                   orientation.x (double)
0x24:   [8 bytes]                   orientation.y (double)
0x2C:   [8 bytes]                   orientation.z (double)
0x34:   [8 bytes]                   orientation.w (double)

总长度：4 + 7×8 = 60 字节（无padding，double 本身 8 字节对齐）
```

---

## 四、TypeSupport 动态加载机制

### 4.1 为什么需要动态加载？

ROS2 支持多种 DDS 实现（FastDDS、CycloneDDS 等），每种实现有自己的序列化方式。`rosidl_typesupport` 使用**动态加载**让同一份 ROS 消息定义支持多种中间件：

```
std_msgs_rosidl_typesupport.so（加载器库）
    │
    ├─ 根据环境变量 RMW_IMPLEMENTATION 选择
    ├─ dlopen("std_msgs__rosidl_typesupport_fastrtps_cpp.so")
    │   └─ 提供 FastCDR 序列化实现
    └─ 或 dlopen("std_msgs__rosidl_typesupport_introspection_cpp.so")
        └─ 提供通用内省实现（适用于 CycloneDDS 等）
```

### 4.2 get_message_type_support_handle() 实现

```cpp
// rosidl_typesupport_cpp/include/rosidl_typesupport_cpp/message_type_support.hpp

template<typename T>
const rosidl_message_type_support_t *
get_message_type_support_handle()
{
  // 每种类型有一个静态的 TypeSupport 注册入口
  // 通过 rosidl 生成代码中定义的全局变量获取
  return rosidl_typesupport_cpp::get_message_type_support_handle<T>();
}
```

```cpp
// 自动生成代码（以 std_msgs::msg::String 为例）：

// rosidl_typesupport_cpp/std_msgs/msg/string__type_support.cpp
template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC_std_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<std_msgs::msg::String>()
{
  // 初始化（单次）：dlopen typesupport 库
  if (!type_support_.typesupport_identifier) {
    // 尝试加载 FastRTPS typesupport
    auto handle = dlopen(
      "libstd_msgs__rosidl_typesupport_fastrtps_cpp.so", RTLD_LAZY);
    if (handle) {
      // 找到 FastRTPS typesupport
      auto get_ts = reinterpret_cast<GetTsFunc>(
        dlsym(handle, "rosidl_typesupport_fastrtps_cpp__get_message_type_support_handle__std_msgs__msg__String"));
      type_support_ = *get_ts();
    } else {
      // 降级到内省 typesupport
      type_support_ = *rosidl_typesupport_introspection_cpp::...();
    }
  }
  return &type_support_;
}
```

---

## 五、rosidl 包层次与依赖

```
rosidl_core（基础定义）
├── rosidl_runtime_c（C 运行时：rosidl_runtime_c__String 等）
├── rosidl_runtime_cpp（C++ 运行时：MessageT 约束等）
└── rosidl_typesupport_interface（类型支持接口定义）

rosidl_generator（代码生成工具）
├── rosidl_generator_c（生成 C 结构体）
├── rosidl_generator_cpp（生成 C++ 结构体）
└── rosidl_generator_py（生成 Python 类）

rosidl_typesupport（序列化实现）
├── rosidl_typesupport_introspection_c（C 内省）
├── rosidl_typesupport_introspection_cpp（C++ 内省）
└── rosidl_typesupport_cpp（动态加载器）

rmw_fastrtps_cpp 提供：
├── rosidl_typesupport_fastrtps_c（FastCDR C 版序列化）
└── rosidl_typesupport_fastrtps_cpp（FastCDR C++ 版序列化）

rmw_cyclonedds_cpp 提供：
└── 使用内省 typesupport + CycloneDDS 的 CDR 实现
```

---

## 六、消息定义语法速查

```
# 基本类型
bool my_bool
byte my_byte            # uint8
char my_char            # uint8（Legacy）
float32 my_float32
float64 my_float64
int8/int16/int32/int64  # 有符号整数
uint8/uint16/uint32/uint64  # 无符号整数
string my_str
wstring my_wstr         # Unicode 字符串

# 数组（固定长度）
float32[3] xyz          # 长度3的float32数组

# 序列（可变长度）
float32[] data          # 动态长度
float32[<=10] bounded   # 有界序列（最大10）

# 嵌套消息
geometry_msgs/Point position
Header header           # std_msgs/Header（特殊：不需要包名）

# 常量
uint8 SPEED_MAX = 10

# 默认值（Galactic+）
float32 speed 0.5
```

---

## 七、action 类型系统

```
# my_msgs/action/Navigate.action

# ── Goal（发送给 server）──────────────────────────
geometry_msgs/Pose target_pose
---
# ── Result（最终返回）────────────────────────────
bool success
string error_message
---
# ── Feedback（进度更新）──────────────────────────
float32 distance_remaining

# 编译后生成：
# Navigate_Goal       → goal topic（rq/.../send_goal）
# Navigate_Result     → result topic（rr/.../send_goal）
# Navigate_Feedback   → feedback topic（rt/.../feedback）
# Navigate_GoalStatusArray → status topic（rt/.../status）
```

---

## 八、序列化性能优化

```cpp
// 1. 避免频繁 malloc（缓存序列化缓冲区）
// Publisher 内部会缓存 rcl_serialized_message_t：
auto serialized_msg = this->create_serialized_message();  // 从缓存取
// 用完后：
this->return_serialized_message(std::move(serialized_msg));  // 放回缓存

// 2. 零拷贝大型消息（iceoryx + loaned messages）
// 发布者：borrow_loaned_message() → 直接在共享内存写 → publish_loaned_message()
// 订阅者：take_loaned_message() → 直接读共享内存 → return_loaned_message()

// 3. 紧凑消息设计（减少序列化大小）
// 差：string 类型（有 4 字节长度头 + 内容）
// 好：固定长度类型（float32[N]）

// 4. 对齐优化
// CDR 按字段对齐，结构体尽量把大字段放前面：
// 推荐：float64 x; float64 y; float32 z;  → 24字节
// 差：  float32 z; float64 x; float64 y;  → 4+4padding+8+8 = 24字节（此例相同）
// 但在复杂结构体中对齐差异显著
```
