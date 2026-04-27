# DDS 实时调优

> 本文档覆盖 ROS 2 支持的主流 RMW 实现的性能特性、CycloneDDS 和 FastDDS 的 RT 关键配置参数、DDS QoS 对延迟的影响，以及完整的网络栈调优方法。

---

## 一、RMW 四维对比表

| RMW 实现 | 本机延迟（P50/P99） | 跨网络延迟 | 吞吐量 | CPU Overhead | 开源协议 | RT 成熟度 |
|---------|-----------------|----------|-------|-------------|--------|---------|
| **iceoryx / rmw_iceoryx** | **< 1μs / < 5μs** | ❌ 不支持 | 极高 | 极低 | Apache 2.0 | ✅ 本机硬实时 |
| **CycloneDDS / rmw_cyclonedds** | 2μs / 20μs | 15μs / 50μs | 高 | 中 | Eclipse | ✅ 推荐 |
| **FastDDS / rmw_fastrtps** | 3μs / 30μs | 20μs / 80μs | 高 | 中高 | Apache 2.0 | ⚠️ 需要配置 |
| **Connext DDS / rmw_connextdds** | 2μs / 15μs | 10μs / 40μs | 极高 | 低 | 商业 | ✅ 工业级 |
| **Zenoh / rmw_zenoh** | 5μs / 30μs | 20μs / 60μs | 高 | 低 | Apache 2.0 | ⚠️ 新兴，测试中 |

> **选型建议**：
> - 本机控制环内通信 → **iceoryx2**（零拷贝，最低延迟）
> - 跨机器遥测/监控 → **CycloneDDS**（开源，配置简单，延迟低）
> - 商业工业项目 → **RTI Connext DDS**（最成熟的工业 RT DDS）
> - 实验性分布式架构 → **Zenoh**（下一代，原生支持云边协同）

---

## 二、CycloneDDS 实时配置

### 2.1 配置文件结构

```xml
<!-- /etc/cyclonedds/cyclonedds.xml -->
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config"
            xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
            xsi:schemaLocation="https://cdds.io/config
            https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain id="any">
    <General>
      <!-- 绑定到特定网卡（避免使用 loopback 以外的接口） -->
      <NetworkInterfaceAddress>lo;eth0</NetworkInterfaceAddress>
      <!-- 允许多播（同子网内 DDS 发现） -->
      <AllowMulticast>true</AllowMulticast>
      <!-- 最大消息大小（UDP payload，需小于 MTU） -->
      <MaxMessageSize>65500B</MaxMessageSize>
      <!-- 分片大小（大消息分片传输） -->
      <FragmentSize>4096B</FragmentSize>
    </General>

    <Internal>
      <!-- 线程优先级 -->
      <Watermarks>
        <WhcHigh>500kB</WhcHigh>
        <WhcLow>100kB</WhcLow>
      </Watermarks>

      <!-- 最小化丢包重传延迟 -->
      <AckNackDelay>0ms</AckNackDelay>
      <HeartbeatInterval>25ms 100ms 1s</HeartbeatInterval>

      <!-- 禁用 Nagle 算法（减少批处理延迟） -->
      <NackDelayedAck>false</NackDelayedAck>
    </Internal>

    <Compatibility>
      <!-- 与 FastDDS 互操作时启用 -->
      <StandardsConformance>lax</StandardsConformance>
    </Compatibility>

    <!-- UDP 套接字缓冲区大小 -->
    <Tracing>
      <OutputFile>stderr</OutputFile>
      <Verbosity>warning</Verbosity>
    </Tracing>
  </Domain>
</CycloneDDS>
```

### 2.2 Socket Buffer 大小调优

```xml
<!-- CycloneDDS 配置：增大收发缓冲区 -->
<CycloneDDS>
  <Domain>
    <Internal>
      <!-- 发送缓冲区：避免 DDS 发送线程阻塞 -->
      <SendSocketBufferSize>1MB</SendSocketBufferSize>
      <!-- 接收缓冲区：避免在高频率消息时丢包 -->
      <RecvSocketBufferSize>10MB</RecvSocketBufferSize>
    </Internal>
  </Domain>
</CycloneDDS>
```

### 2.3 CycloneDDS 共享内存传输（与 iceoryx 集成）

```xml
<!-- 启用 iceoryx 共享内存传输（本机通信走 SHM，跨机器走 UDP） -->
<CycloneDDS>
  <Domain>
    <SharedMemory>
      <Enable>true</Enable>
      <!-- RouDi 必须已启动 -->
      <SubQueueCapacity>256</SubQueueCapacity>
      <PubHistoryCapacity>16</PubHistoryCapacity>
      <LogLevel>info</LogLevel>
    </SharedMemory>
  </Domain>
</CycloneDDS>
```

### 2.4 环境变量快速配置

```bash
# 设置配置文件路径
export CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.xml

# 或直接通过环境变量内嵌 XML（开发/测试时方便）
export CYCLONEDDS_URI='<CycloneDDS><Domain><General>
  <NetworkInterfaceAddress>lo</NetworkInterfaceAddress>
  </General></Domain></CycloneDDS>'

# 禁用多播（单机或不支持多播的网络）
export CYCLONEDDS_URI='<CycloneDDS><Domain><General>
  <AllowMulticast>false</AllowMulticast>
  </General></Domain></CycloneDDS>'

# ROS 2 使用 CycloneDDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

---

## 三、FastDDS 实时配置

### 3.1 profiles XML 配置

```xml
<!-- /etc/fastdds/fastdds_rt_profile.xml -->
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">

  <!-- 默认 Participant 配置 -->
  <participant profile_name="rt_participant" is_default_profile="true">
    <rtps>
      <!-- 禁用 Multicast 发现（单机内使用 localhost） -->
      <builtinTransports>DEFAULT</builtinTransports>

      <!-- 网络接口 -->
      <listen_socket_buffer_size>10485760</listen_socket_buffer_size>

      <!-- 发现服务器（代替多播） -->
      <discovery_config>
        <discoveryProtocol>SIMPLE</discoveryProtocol>
        <use_CLIENT_UseLocatorList>false</use_CLIENT_UseLocatorList>
        <leaseDuration>
          <sec>10</sec>
          <nanosec>0</nanosec>
        </leaseDuration>
        <leaseAnnouncement>
          <sec>3</sec>
          <nanosec>0</nanosec>
        </leaseAnnouncement>
      </discovery_config>
    </rtps>
  </participant>

  <!-- RT 发布者配置模板 -->
  <publisher profile_name="rt_publisher">
    <historyMemoryPolicy>PREALLOCATED_WITH_REUSE</historyMemoryPolicy>
    <!-- 预分配避免运行时 malloc -->
    <payloadMaxSize>65536</payloadMaxSize>
    <!-- 最大历史缓存（KEEP_LAST 语义） -->
    <historyQos>
      <kind>KEEP_LAST</kind>
      <depth>1</depth>
    </historyQos>
    <resourceLimitsQos>
      <max_samples>1</max_samples>
      <max_instances>1</max_instances>
      <max_samples_per_instance>1</max_samples_per_instance>
      <!-- 关键：预分配，避免运行时扩容 -->
      <allocated_samples>1</allocated_samples>
    </resourceLimitsQos>
  </publisher>

  <!-- RT 订阅者配置模板 -->
  <subscriber profile_name="rt_subscriber">
    <historyMemoryPolicy>PREALLOCATED_WITH_REUSE</historyMemoryPolicy>
    <historyQos>
      <kind>KEEP_LAST</kind>
      <depth>1</depth>
    </historyQos>
    <resourceLimitsQos>
      <max_samples>10</max_samples>
      <allocated_samples>10</allocated_samples>
    </resourceLimitsQos>
  </subscriber>

  <!-- UDP Transport 配置（增大缓冲区） -->
  <transport_descriptor>
    <transport_id>udp_transport</transport_id>
    <type>UDPv4</type>
    <sendBufferSize>4194304</sendBufferSize>    <!-- 4MB -->
    <receiveBufferSize>10485760</receiveBufferSize>  <!-- 10MB -->
    <non_blocking_send>false</non_blocking_send>
  </transport_descriptor>

</profiles>
```

### 3.2 FastDDS 环境变量

```bash
# 设置 FastDDS 配置文件
export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds/fastdds_rt_profile.xml

# 使用 FastDDS
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 关闭 FastDDS 异步写入（减少延迟，但增加发布线程负担）
# 通过 XML profile 中 publisher 的 <publication_mode> 控制
```

### 3.3 historyMemoryPolicy 选项对比

| 策略 | 内存使用 | RT 适用性 | 说明 |
|------|---------|---------|------|
| `DYNAMIC` | 动态增长 | ❌ | 运行时 malloc，不适合 RT |
| `DYNAMIC_REUSABLE` | 动态但可复用 | ⚠️ | 减少 malloc 但不消除 |
| **`PREALLOCATED`** | 固定预分配 | ✅ | 启动时一次性分配所需内存 |
| **`PREALLOCATED_WITH_REUSE`** | **固定 + 复用** | **✅ 首选** | **预分配 + 不足时动态增长** |

---

## 四、DDS QoS 实时关键参数

### 4.1 RELIABILITY — 可靠性 vs 延迟的权衡

```yaml
# ROS 2 QoS 配置示例（rclcpp）

# 控制命令：最低延迟，允许丢包
qos_cmd:
  reliability: BEST_EFFORT     # 无重传，最低延迟
  durability: VOLATILE         # 不保留历史消息
  history: KEEP_LAST
  depth: 1                     # 只保留最新消息

# 关节状态（高频传感器数据）：允许偶尔丢包
qos_state:
  reliability: BEST_EFFORT
  durability: VOLATILE
  history: KEEP_LAST
  depth: 1

# 配置参数（低频但重要）：必须可靠
qos_config:
  reliability: RELIABLE
  durability: TRANSIENT_LOCAL  # 新订阅者能收到历史消息
  history: KEEP_LAST
  depth: 1
```

**延迟影响：**

| QoS 配置 | 额外延迟来源 | 典型开销 |
|---------|-----------|--------|
| `BEST_EFFORT` | 无 | ~0μs |
| `RELIABLE` | ACK/NACK 机制 | +50–200μs（本机）|
| `RELIABLE + TRANSIENT_LOCAL` | 历史缓存回放 | +100μs–1ms（新订阅者） |

### 4.2 DEADLINE — 发布频率监控

```cpp
// 设置 DEADLINE QoS：声明 Publisher 每 1ms 发布一次
rclcpp::QoS qos(rclcpp::KeepLast(1));
qos.deadline(rclcpp::Duration(0, 1000000));  // 1ms

auto publisher = node->create_publisher<JointState>("joint_states", qos);

// 如果超过 deadline 未发布，DDS 会触发 on_offered_deadline_missed 回调
// 可用于监控控制环是否超时
```

### 4.3 LIVELINESS — 节点存活检测

```cpp
// 对 controller_manager 订阅设置存活性检测
rclcpp::QoS qos(rclcpp::KeepLast(1));
qos.liveliness(rclcpp::LivelinessPolicy::Automatic);
qos.liveliness_lease_duration(rclcpp::Duration(1, 0));  // 1秒内必须有活动

// 若订阅者超过 1 秒没有收到任何消息，触发 on_liveliness_changed 回调
// 可用于检测 hardware_interface 进程崩溃
```

### 4.4 LATENCY_BUDGET — 延迟预算（提示）

```cpp
// LATENCY_BUDGET：向 DDS 提示可接受的最大延迟
// DDS 可据此优化批处理策略（非强制保证）
rclcpp::QoS qos(rclcpp::KeepLast(1));
qos.latency_budget(rclcpp::Duration(0, 500000));  // 500μs 预算

// 对 BEST_EFFORT 传输，设置较小的 LATENCY_BUDGET 可减少 DDS 的批处理等待
```

---

## 五、SCHED_DEADLINE — 为 DDS 线程设置截止时间调度

### 5.1 原理

`SCHED_DEADLINE` 实现了 Earliest Deadline First（EDF）调度，比 `SCHED_FIFO` 更灵活：

```
SCHED_DEADLINE 参数：
  runtime:  每周期允许使用的 CPU 时间（例：500μs）
  deadline: 每周期的截止时间（例：1ms）
  period:   周期时长（例：1ms）
```

### 5.2 为 DDS recv 线程设置 SCHED_DEADLINE

```bash
# 获取 DDS recv 线程的 TID
DDS_TID=$(ls /proc/<CM_PID>/task/ | head -1)

# 设置 SCHED_DEADLINE（需要 root）
# runtime=500μs, deadline=1ms, period=1ms
chrt --deadline --sched-runtime 500000 --sched-deadline 1000000 \
     --sched-period 1000000 -p $DDS_TID
```

### 5.3 风险与注意事项

⚠️ **SCHED_DEADLINE 使用风险**：
- 若 `runtime/period > 系统总 CPU 容量`，会触发 `EBUSY`（admission control 拒绝）
- 过多 SCHED_DEADLINE 任务可能导致普通任务完全饥饿
- 建议仅对关键 DDS 线程使用，其余使用 SCHED_FIFO

---

## 六、网络栈调优

### 6.1 UDP Socket Buffer 大小

```bash
# 查看当前 UDP 缓冲区大小
cat /proc/sys/net/core/rmem_max    # UDP 接收缓冲区最大值
cat /proc/sys/net/core/wmem_max    # UDP 发送缓冲区最大值
cat /proc/sys/net/core/rmem_default

# 增大缓冲区（防止高频消息时 UDP 丢包）
sudo sysctl -w net.core.rmem_max=134217728    # 128MB
sudo sysctl -w net.core.wmem_max=134217728    # 128MB
sudo sysctl -w net.core.rmem_default=26214400  # 25MB

# 持久化到 /etc/sysctl.d/99-ros2-rt.conf
cat > /etc/sysctl.d/99-ros2-rt.conf << 'EOF'
net.core.rmem_max = 134217728
net.core.wmem_max = 134217728
net.core.rmem_default = 26214400
net.core.netdev_max_backlog = 30000
net.core.optmem_max = 25165824
EOF
sudo sysctl --system
```

### 6.2 IRQ Affinity — 网卡中断绑定

将网卡 IRQ 绑定到非 RT 核，防止网络中断干扰控制线程：

```bash
# 关闭 irqbalance（它会自动重新分配 IRQ，干扰手动设置）
sudo systemctl stop irqbalance
sudo systemctl disable irqbalance

# 找到网卡 IRQ
cat /proc/interrupts | grep eth0

# 将 IRQ 绑定到 Core 0,1（非 RT 核）
# IRQ #XX 对应 eth0
echo 3 | sudo tee /proc/irq/XX/smp_affinity  # 0b0011 = Core 0,1

# 脚本：批量设置所有网卡 IRQ 到非 RT 核
for irq_dir in /proc/irq/*/; do
    irq_num=$(basename $irq_dir)
    device=$(cat ${irq_dir}smp_affinity_list 2>/dev/null)
    # 将非 RT 设备 IRQ 绑定到 CPU 0-1
    if [[ -f "${irq_dir}actions" ]]; then
        action=$(cat ${irq_dir}actions)
        if [[ "$action" == *"eth"* ]] || [[ "$action" == *"enp"* ]]; then
            echo "0-1" | sudo tee ${irq_dir}smp_affinity_list > /dev/null
        fi
    fi
done
```

### 6.3 NAPI 轮询与中断合并

```bash
# 查看网卡中断合并设置
ethtool -c eth0

# 减少中断合并（降低延迟，增加 CPU 负担）
sudo ethtool -C eth0 rx-usecs 0 tx-usecs 0
sudo ethtool -C eth0 rx-frames 1 tx-frames 1

# 对 ROS 2 控制内容（本机 loopback 通信）：
# loopback 接口走内核 virtual 网络，不受 ethtool 影响
# 最佳方案：改用 iceoryx 共享内存，彻底绕过网络栈
```

---

## 七、选型决策矩阵

### 7.1 通信场景与 RMW 选型

| 通信场景 | 推荐 RMW | QoS 配置 | 理由 |
|---------|---------|---------|------|
| **控制环内**（CM ↔ hw_interface，同机） | iceoryx2 | — | 零拷贝，<1μs |
| **控制环内**（controller ↔ controller，同机） | iceoryx2 或 CycloneDDS SHM | BEST_EFFORT, KEEP_LAST 1 | 最低延迟 |
| **传感器数据**（camera/lidar → VLA，同机） | CycloneDDS SHM 或 iceoryx2 | BEST_EFFORT, KEEP_LAST 1 | 高吞吐，允许丢帧 |
| **跨机器控制**（主控 SoC → 从属节点） | CycloneDDS UDP | BEST_EFFORT | 低延迟 UDP |
| **遥测 / 监控**（→ Foxglove） | CycloneDDS UDP | RELIABLE, TRANSIENT_LOCAL | 不丢数据 |
| **rosbag2 录制** | CycloneDDS UDP | 匹配发布者 QoS | 不影响 RT 核 |
| **参数服务 / 服务发现** | CycloneDDS UDP | RELIABLE | 需要可靠 |

### 7.2 具身智能完整 RMW 配置示例

```bash
# launch 文件中配置不同节点使用不同 RMW 的方案（使用 composable nodes）

# 方案：所有节点运行在同一机器，使用环境变量统一 RMW
# 本机内通信：启用 CycloneDDS SHM（等价于 iceoryx 效果）
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds_shm.xml

# 启动 iceoryx RouDi（CycloneDDS SHM 需要）
iox-roudi &

# 启动控制系统
ros2 launch chassis_bringup rt_control_system.launch.py
```

```xml
<!-- cyclonedds_shm.xml：本机全部走共享内存 -->
<CycloneDDS>
  <Domain>
    <SharedMemory>
      <Enable>true</Enable>
      <SubQueueCapacity>256</SubQueueCapacity>
      <PubHistoryCapacity>16</PubHistoryCapacity>
    </SharedMemory>
    <General>
      <!-- 仅允许 loopback，防止意外走 UDP -->
      <NetworkInterfaceAddress>lo</NetworkInterfaceAddress>
    </General>
    <Internal>
      <SendSocketBufferSize>2MB</SendSocketBufferSize>
      <RecvSocketBufferSize>10MB</RecvSocketBufferSize>
    </Internal>
  </Domain>
</CycloneDDS>
```
