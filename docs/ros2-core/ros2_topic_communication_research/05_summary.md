# Topic 通信（五）：性能分析与零拷贝路径对比

## 一、完整发布-接收时序图

```
时间轴（从上到下）
│
│  [发布线程]
├── pub->publish(msg)
│     │
│     ├── [序列化] MessageT → CDR bytes（约 1-5μs，取决于消息大小）
│     │
│     └── rcl_publish_serialized_message()
│           │
│           └── rmw_publish_serialized_message()
│                 │
│                 └── DataWriter::write()
│                       │
│                       ├── [同步模式] 立即调用 UDP send()（约 5-20μs）
│                       │     └── ✓ 函数返回
│                       │
│                       └── [异步模式] 将数据放入队列（约 1μs）
│                             └── ✓ 函数返回（数据尚未发出）
│                                   │（异步线程）
│                                   └── UDP send()
│
│  [网络传输]（本机环回：~5μs，同主机 UDP：~10-50μs，跨主机：~100μs+）
│
│  [接收线程（DDS 内部）]
├── DDP RTPS DataReader 收到数据包
│     │
│     ├── 解析 RTPS 帧，提取序列化载荷
│     ├── 存入 ReaderHistory
│     └── conditionVariable.notify_all()（唤醒 rmw_wait()）
│
│  [spin 线程]
├── rcl_wait() 中的 rmw_wait() 返回
├── Executor::get_next_ready_executable() 找到就绪订阅
├── Executor::execute_subscription()
│     │
│     ├── subscription->create_message()（约 0.1μs，复用缓冲区）
│     ├── rcl_take() → rmw_take() → DataReader::read()
│     │     └── [反序列化] CDR bytes → MessageT（约 1-5μs）
│     └── AnySubscriptionCallback::dispatch()
│           │
│           └── user_callback(msg)（用户代码开始执行）
│
└── [用户代码执行]

端到端延迟（典型值，本机测试）：
  std_msgs::msg::String（10字节）：    ~20-50μs
  sensor_msgs::msg::PointCloud2（大）：~100-500μs（取决于消息大小）
  进程内通信（同进程）：               ~2-10μs
```

---

## 二、三种通信路径对比

### 2.1 进程外通信（Inter-Process）

```
                publish(msg)
                     │
                   拷贝（序列化 CDR）
                     │
                  UDP/TCP 发送
                     │
                   拷贝（接收 CDR）
                     │
                  反序列化（填充 C 结构体）
                     │
                   拷贝（C 结构体 → C++ 对象）
                     │
                  用户回调（msg）

总拷贝次数：3-4 次
延迟（本机）：20-100μs
适用场景：跨进程（必须），不同主机（必须）
```

### 2.2 进程内通信（Intra-Process）

```
                publish(unique_ptr)
                     │
             IntraProcessManager
                  存入 ring buffer（移动语义，0次拷贝）
                     │
                Guard Condition 触发
                     │
                Executor 唤醒
                     │
          取出 unique_ptr（0次拷贝）
                     │
             用户回调（shared_ptr<msg>）

总拷贝次数：0 次（unique_ptr → shared_ptr，引用计数增加）
延迟：2-10μs
适用场景：同一进程内的节点间通信
限制：发布者和订阅者在同一进程，且都启用了 use_intra_process_comms
```

### 2.3 借贷消息（Loaned Messages，iceoryx 路径）

```
                borrow_loaned_message()
                     │
             iceoryx 共享内存池
            返回共享内存区域的指针
                     │
                填写消息（直接写共享内存）
                     │
            publish_loaned_message()
                     │
        通知 iceoryx：某块共享内存有新数据
                     │
                订阅者收到通知
                     │
        take_loaned_message()（获取同一块共享内存）
                     │
             用户回调（void* → 类型转换）
                     │
         return_loaned_message()（归还内存）

总拷贝次数：0 次（发布者和订阅者共享同一块物理内存）
延迟：~5μs（仅进程间，iceoryx 内部使用信号量）
适用场景：同一主机的进程间大数据（如点云、图像）
限制：需要 iceoryx rmw（rmw_iceoryx_cpp），且 DDS 实现支持 loaned_messages
```

---

## 三、性能瓶颈分析

```
瓶颈位置（按重要性）：

1. ★★★★★ 序列化/反序列化
   - std_msgs::String（100字节）：约 1μs
   - sensor_msgs::PointCloud2（1MB 点云）：约 1ms
   - 优化：使用 loaned messages 或 intra-process 绕过序列化

2. ★★★★  UDP 内核调用
   - 每次 sendto/recvfrom：约 5-10μs（系统调用开销）
   - 优化：批量消息（RTPS 消息聚合）、ZeroCopy transport

3. ★★★   Executor wait_set 重建
   - 每次 spin_once 都要 clear + 重新 add 所有实体
   - 对于大量节点（100+）：约 10-50μs
   - 优化：StaticSingleThreadedExecutor

4. ★★★   C++ 对象构造/析构（内存分配器）
   - create_message() 分配 shared_ptr
   - 优化：使用自定义 Allocator（节点级内存池）

5. ★★    CallbackGroup 锁竞争
   - MultiThreadedExecutor 线程越多，锁竞争越明显
   - 优化：合理分配 ReentrantCallbackGroup（无状态回调）
```

---

## 四、常用性能诊断

```bash
# 1. 测量 topic 端到端延迟（需要消息中有 header.stamp）
ros2 topic delay /my_topic

# 2. 测量发布频率
ros2 topic hz /my_topic

# 3. 测量带宽
ros2 topic bw /my_topic

# 4. 查看 QoS（检查 reliable/best_effort 配置）
ros2 topic info /my_topic --verbose

# 5. Tracing（需要 ros2_tracing 包）
ros2 launch tracetools_launch example.launch.py
# 然后用 LTTng 分析 callback_start, callback_end, publish, rcl_take 等 tracepoint

# 6. 查看序列化大小
python3 -c "
from std_msgs.msg import String
from rclpy.serialization import serialize_message
msg = String(data='hello')
data = serialize_message(msg)
print(f'Size: {len(data)} bytes')
"
```

---

## 五、零拷贝实现对比

| 特性 | 进程内通信 | 借贷消息（iceoryx） | 共享内存（FastDDS SHM） |
|------|----------|-------------------|----------------------|
| 跨进程 | ❌ | ✅ | ✅ |
| 零拷贝 | ✅（unique_ptr） | ✅（共享内存） | ✅（共享内存） |
| 支持 rmw | 所有 | rmw_iceoryx_cpp | rmw_fastrtps_cpp |
| API 变化 | ✅ 透明 | 需要 borrow/return | ✅ 透明 |
| 多订阅者 | 需拷贝 N-1 份 | 引用计数 | 引用计数 |
| 大消息优势 | ✅ 显著 | ✅ 非常显著 | ✅ 显著 |

---

## 六、Publisher/Subscriber QoS 兼容性矩阵

```
Reliability（可靠性）：
  Publisher RELIABLE  ↔  Subscriber RELIABLE  ✅ 兼容
  Publisher RELIABLE  ↔  Subscriber BEST_EFFORT ✅ 兼容（订阅者"降级"）
  Publisher BEST_EFFORT ↔ Subscriber RELIABLE  ❌ 不兼容（不建立连接）
  Publisher BEST_EFFORT ↔ Subscriber BEST_EFFORT ✅ 兼容

Durability（持久性）：
  Publisher TRANSIENT_LOCAL ↔ Subscriber TRANSIENT_LOCAL ✅ 兼容（晚到订阅者收到历史数据）
  Publisher TRANSIENT_LOCAL ↔ Subscriber VOLATILE ✅ 兼容（订阅者不要求历史）
  Publisher VOLATILE ↔ Subscriber TRANSIENT_LOCAL ❌ 不兼容
  Publisher VOLATILE ↔ Subscriber VOLATILE ✅ 兼容

规则：Subscriber 的 QoS 要求不能"高于" Publisher 的 QoS 能力
```

---

## 七、调试发布-接收问题的系统方法

```
Problem: 订阅者收不到消息

Step 1: 检查 topic 名称
  ros2 topic list  →  确认 topic 名称一致

Step 2: 检查 QoS 兼容性
  ros2 topic info /topic --verbose
  比对 publisher 和 subscriber 的 reliability/durability

Step 3: 检查 domain ID
  echo $ROS_DOMAIN_ID  →  必须相同（默认都是 0）

Step 4: 检查节点是否启动
  ros2 node list  →  确认两个节点都在

Step 5: 检查是否真的发布了数据
  ros2 topic echo /topic  →  如果能收到，说明发布端正常
  →  如果不能收到，说明发布端未发布或 QoS 问题

Step 6: 检查进程内通信配置
  如果发布者和订阅者在同一进程，检查 NodeOptions::use_intra_process_comms()
  进程内通信的 QoS 规则与进程外相同

Step 7: 使用 ros2 doctor
  ros2 doctor --report  →  全面诊断报告
```

---

## 八、源码关键路径速查

```
发布路径：
  rclcpp/include/rclcpp/publisher.hpp             ← publish() 入口
  rclcpp/src/rclcpp/publisher_base.cpp            ← do_inter_process_publish()
  rcl/src/rcl/publisher.c                         ← rcl_publish_serialized_message()
  rmw_fastrtps_shared_cpp/src/rmw_publish.cpp     ← rmw_publish_serialized_message()
  FastDDS: DataWriterImpl.cpp                     ← DataWriter::write()

接收路径：
  rclcpp/src/rclcpp/executor.cpp                  ← execute_subscription()
  rclcpp/src/rclcpp/subscription_base.cpp         ← take_type_erased()
  rcl/src/rcl/subscription.c                      ← rcl_take()
  rmw_fastrtps_shared_cpp/src/rmw_take.cpp        ← rmw_take_with_info()

序列化：
  rmw_fastrtps_shared_cpp/src/TypeSupport.cpp     ← serialize/deserialize
  rclcpp/src/rclcpp/serialization.cpp             ← rclcpp serialize wrapper

进程内通信：
  rclcpp/src/rclcpp/intra_process_manager.cpp     ← 进程内消息分发

GitHub 链接：
  发布：https://github.com/ros2/rclcpp/blob/rolling/rclcpp/include/rclcpp/publisher.hpp
  接收：https://github.com/ros2/rclcpp/blob/rolling/rclcpp/src/rclcpp/executor.cpp
  序列化：https://github.com/ros2/rmw_fastrtps/blob/rolling/rmw_fastrtps_shared_cpp/src/TypeSupport.cpp
```
