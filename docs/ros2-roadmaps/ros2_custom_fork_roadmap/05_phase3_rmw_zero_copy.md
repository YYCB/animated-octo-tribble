# 05 阶段 3：rmw 层零拷贝与 Loaned Message

> **目标**：消除 rclcpp → rcl → rmw → DDS 链路上的冗余拷贝，尤其针对 **大消息（> 64 KB）** 与 **跨进程同机场景**。
>
> **前置**：阶段 1、2 已完成。本阶段是"大消息场景的命门"——对小消息（< 1 KB）收益不大。

---

## 5.1 Loaned Message 模型

### 5.1.1 核心思想

**传统 `publish(msg)` 路径**：
```
user's MsgT  ──memcpy──▶  CDR 缓冲 (heap alloc + serialize)
                            │
                            ▼
                          DDS history cache (memcpy)
                            │
                            ▼
                          socket send (memcpy)
```

**Loaned Message 路径**：
```
rmw_borrow_loaned_message()
  └▶  从 DDS SHM pool 分配一块 MsgT 大小的内存（或 DDS 管理的堆块）
      ▲
      │ 用户**原地构造**消息
      ▼
rmw_publish_loaned_message(loaned)
  └▶  DDS 直接发送那块内存，不序列化、不拷贝
```

两端都用 Loaned + 同机 SHM 时：**从 publish 到 take 全程 0 拷贝**，只有 1 次内存屏障 + ringbuffer 指针更新。

### 5.1.2 API 全景

发布端：
```cpp
auto pub = node->create_publisher<MyMsg>("topic", qos);
if (pub->can_loan_messages()) {
  auto loaned = pub->borrow_loaned_message();    // 从 DDS 拿一块
  loaned.get() = /* 就地填充消息 */;
  pub->publish(std::move(loaned));               // 发送，DDS 接管所有权
}
```

订阅端（较少用，多数人仍用回调）：
```cpp
MyMsg msg;
rcl_ret_t rc = rcl_take_loaned_message(sub, &loaned_ptr, nullptr, nullptr);
// 处理 loaned_ptr 指向的消息
rcl_return_loaned_message_from_subscription(sub, loaned_ptr);
```

### 5.1.3 何时 `can_loan_messages()` 返回 true

三个条件**同时**满足：
1. **rmw 实现支持**：FastDDS 在消息固定大小且 SHM transport 启用时；CycloneDDS 近期版本也支持；
2. **消息类型是固定大小**（"fixed memory"）：rosidl 生成时确定（无 `std::string`、无动态 `std::vector`）；
3. **QoS 兼容**：KEEP_LAST、depth 不过大、reliability 兼容。

**实测中常见问题**：消息里有一个 `std::string header.frame_id` → 整个消息被判为变长 → 不能 Loan。**必须定义 bounded 类型**（`string<=32`）或使用不含 Header 的原始消息。

---

## 5.2 消息类型设计

### 5.2.1 IDL 定义约束

允许 Loaned 的消息：
```idl
# my_msgs/msg/JointCommand.msg
float64[7] positions          # fixed-size array: OK
float64[7] velocities         # OK
int32 seq                     # OK
builtin_interfaces/Time stamp # OK (固定大小)
```

**禁忌**：
```idl
string name                      # ❌ std::string
sensor_msgs/JointState state     # ❌ 包含 string[] name
float64[] positions              # ❌ dynamic array
```

**改造**：
```idl
string<=32 name                  # ✅ BoundedString
float64[] positions              # ❌  → 改为
float64[256] positions           # ✅ 固定上界数组（浪费内存换性能）
#  或
sequence<float64, 256> positions # ✅ BoundedSequence
```

### 5.2.2 标准消息的困境

许多标准消息（`std_msgs`, `sensor_msgs`, `geometry_msgs`）的 Header 都包含 `string frame_id`——不支持 Loan。

**选项**：
- **A. 定义并行消息类型**：`my_msgs/ImuRaw` 无 Header，用整数 frame_id（查表）；
- **B. 在 IDL 里用 `string<=16`**：但与标准 msg 不兼容，不能互通；
- **C. 等待上游**：社区有 PR 推动 ROS msg 的 bounded 化，进展缓慢。

**具身智能项目常见选择**：**A**——内部消息类型完全自定义，边界上通过桥接节点转成标准类型给 rviz / rosbag。

### 5.2.3 自定义 rosidl generator

生成 "纯 C 结构体" 的消息类型（无 `std::string`/`std::vector`），与默认 `rosidl_generator_cpp` 并存：
- 输入相同的 `.msg` 文件；
- 输出 `struct MyMsg { double positions[7]; ... };`；
- 对应的 `typesupport` 实现使用 `memcpy` 而非 CDR。

工作量：**约 2–4 人·周**，收益显著——从此所有内部消息都能 Loan。

---

## 5.3 rmw 层的改造

### 5.3.1 现状

- `rmw_fastrtps_cpp` / `rmw_cyclonedds_cpp` 都已实现 `rmw_borrow_loaned_message` 等 API；
- 但**rcl / rclcpp 层的包装不完全透传**：例如 `Publisher<MsgT>::publish(unique_ptr<MsgT>)` 内部若走不到 loan 路径，仍会序列化；
- 某些 `rmw_publish` 调用**即使支持 Loan 也会 memcpy 一次**进 DDS history cache（FastDDS 的 `WITH_SECURITY` 编译会强制拷贝）。

### 5.3.2 可定制改造点

1. **rmw 层消除冗余 cache copy**：
   - FastDDS 的 `DataWriter::write(loaned)` 若历史深度为 1 且 writer 可访问 SHM pool，可跳过 history cache 拷贝；
   - 需要 patch `rmw_fastrtps_cpp` + `fastrtps` 本身；
2. **rcl 层透传 loaned**：确认 `rcl_publisher_can_loan_messages` 的判定逻辑与 rmw 一致；
3. **rclcpp Publisher**：封装 `publish(MsgT&& msg)` 重载，自动尝试 `borrow_loaned_message + 构造 + publish_loaned`——对用户透明。

### 5.3.3 Serialized Message 路径（另一条优化线）

部分场景根本不需要解包消息（如 bridge / rosbag），可以直接用 `rclcpp::SerializedMessage`：
```cpp
rclcpp::SerializedMessage serialized;
rcl_take_serialized_message(sub, &serialized.get_rcl_serialized_message(), nullptr, nullptr);
// 直接转发或落盘，不反序列化
```
**收益**：rosbag2 的录制路径可省掉反序列化 + 重新序列化（2 次拷贝）。

---

## 5.4 IntraProcess Manager 的深化

IPC 路径理论上 0 拷贝，但官方实现在某些情况下会退化：

### 5.4.1 退化场景

1. **有多个订阅者，且订阅者用 `const shared_ptr<const MsgT>&`**：
   - 第一个订阅能拿到 unique_ptr 移交；
   - 其他订阅只能拿到 shared_ptr 副本——**但消息只有一份**，是 shared 访问；
   - 实际 0 拷贝，仅 shared_ptr atomic inc。
2. **有混合订阅者（shared + unique）**：
   - 使用 unique_ptr 的订阅端会强制消息被**复制一份**供 shared 订阅；
   - **一定要让所有订阅都用 `const shared_ptr<const MsgT>&`** 或都用 `unique_ptr`。
3. **跨 component container**：IPC 仅限同进程，跨容器退化到 DDS。

### 5.4.2 自研 IPC 改造

可以替换 `rclcpp::experimental::IntraProcessManager` 为**更激进的实现**：
- 环形缓冲代替当前的 ring-per-publisher；
- 消费者视角 lock-free；
- 允许 Publisher 知道自己有多少"只读共享"订阅者，一次性标记；
- 优化目标：μs 级同进程广播给 10+ 订阅者。

这是阶段 2 / 3 的交叉内容。

---

## 5.5 类型擦除的代价（TypeSupport）

ROS 2 的消息通过"type support"机制在 rcl / rmw / DDS 间传递类型信息：
```
rosidl_typesupport_introspection_cpp (反射式)
rosidl_typesupport_fastrtps_cpp       (FastDDS 原生)
rosidl_typesupport_introspection_c    (C 侧反射)
...
```

**性能影响**：
- FastDDS 的**内省式**类型支持每次 serialize 都要遍历成员元数据——慢；
- **原生** type support 在编译期生成 specialized serialize 函数——快 3–5×；
- 默认使用原生，但某些第三方工具（如 `ros2 topic echo` 的 `--field` 选项）会强制走内省路径。

**检查方式**：构建产物里应该有 `librosidl_typesupport_fastrtps_cpp__my_msgs.so`，缺失则退化到内省。

---

## 5.6 本阶段结束准则

典型收益（叠加 Phase1+2 之上）：

| 场景 | Phase2 后 | Phase3 后 | 收益 |
|------|----------|----------|------|
| 1 MB 消息同机 | 5 ms | 300 μs | **16×** |
| 1 KB 消息 | 30 μs | 25 μs | 1.2×（小消息提升小） |
| 内存分配/s（热路径） | 100k/s | ~0 | ∞ |
| 点云（5 MB） | 25 ms | 1.2 ms | 20× |

---

## 5.7 本阶段维护成本

- 改动面：**rclcpp + 可能的 rmw + 可能的 rosidl generator + 消息类型定义**；
- 与上游兼容性：**差**——Loaned API 在 Iron/Jazzy 之间有多次破坏性变化；
- 每次 ROS 2 版本升级：**2–4 周**；
- 典型团队规模：**2–3 人**；
- **决策关键**：消息类型定义的改造一旦全系统铺开，**不可逆**——所有上下游 / bag 工具 / UI 都要适配。启动前必须有完整的消息 schema review。

下一章：[06 阶段 4：SHM / Iceoryx](./06_phase4_dds_shm_iceoryx.md)。
