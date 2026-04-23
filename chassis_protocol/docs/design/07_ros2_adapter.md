# 07 ROS2 适配层

## 1. 概述

文件：`ros2_adapter/chassis_node.{hpp,cpp}`，`ros2_adapter/chassis_node_main.cpp`

`ChassisNode` 是 HAL 层与 ROS2 生态之间的桥接类，负责：

1. 读取 ROS2 参数，构造对应的 `ITransport` 和 `IChassisController`
2. 订阅 `/cmd_vel`，转发给 `IChassisController::sendVelocity()`
3. 注册 HAL 回调，将 `OdometryData` / `ChassisStatus` / `BatteryData` 转换为标准 ROS2 消息发布
4. 定时检测连接状态，必要时触发 `ChassisHal::connect()` 重连

---

## 2. 节点入口

```cpp
// ros2_adapter/chassis_node_main.cpp
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("chassis_node");
    chassis_ros2::ChassisNode chassis_node(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

`ChassisNode` 持有 `rclcpp::Node::SharedPtr`，而不继承自 `rclcpp::Node`，方便测试时注入 mock node。

---

## 3. ROS2 参数

在 `declareParameters()` 中声明，在 `initController()` 中读取：

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `transport_type` | string | `"tcp"` | `"tcp"` / `"udp"` / `"rs485"` |
| `host` | string | `"192.168.1.100"` | TCP/UDP 远端主机 |
| `port` | int | `8899` | TCP/UDP 端口 |
| `device` | string | `"/dev/ttyUSB0"` | RS485 设备路径 |
| `baud_rate` | int | `115200` | RS485 波特率 |
| `chassis_type` | string | `"differential"` | `"differential"` / `"mecanum"` |
| `reconnect_interval_s` | double | `5.0` | 重连检测间隔（秒） |
| `frame_id` | string | `"odom"` | 里程计消息 header.frame_id |
| `child_frame_id` | string | `"base_link"` | 里程计消息 child_frame_id |

---

## 4. 订阅与发布

```
订阅（Subscriptions）
  /cmd_vel  ← geometry_msgs/msg/Twist

发布（Publishers）
  /odom           → nav_msgs/msg/Odometry        （队列深度 10）
  /battery_state  → sensor_msgs/msg/BatteryState  （队列深度 10）
  /chassis_status → std_msgs/msg/String           （JSON 格式，队列深度 10）
  /diagnostics    → diagnostic_msgs/msg/DiagnosticArray （队列深度 10）
```

---

## 5. 控制器初始化流程

```cpp
void ChassisNode::initController() {
    // 1. 读取参数
    transport_type_ = node_->get_parameter("transport_type").as_string();
    // ... 其他参数 ...

    // 2. 构造 TransportConfig
    chassis::TransportConfig cfg;
    cfg.host        = host_;
    cfg.port        = port_;
    cfg.device_path = device_;
    cfg.baud_rate   = baud_rate_;
    cfg.timeout_ms  = 3000;  // 硬编码连接超时 3s

    // 3. 选择传输类型
    chassis::TransportType ttype = chassis::TransportType::TCP;
    if (transport_type_ == "udp")   ttype = chassis::TransportType::UDP;
    if (transport_type_ == "rs485") ttype = chassis::TransportType::RS485;

    auto transport = chassis::ITransport::create(ttype, cfg);

    // 4. 选择控制器类型并连接
    if (chassis_type_ == "mecanum") {
        auto ctrl = std::make_unique<chassis::MecanumController>(std::move(transport));
        ctrl->connect();    // 失败仅打 WARN，由重连定时器处理
        controller_ = std::move(ctrl);
    } else {
        auto ctrl = std::make_unique<chassis::DifferentialController>(std::move(transport));
        ctrl->connect();
        controller_ = std::move(ctrl);
    }
}
```

---

## 6. `/cmd_vel` 回调

```cpp
void ChassisNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!controller_) return;
    controller_->sendVelocity(
        msg->linear.x,    // vx
        msg->linear.y,    // vy（差速底盘内部会强制为 0）
        msg->angular.z    // omega
    );
}
```

---

## 7. 里程计回调 → `/odom`

```cpp
void ChassisNode::onOdometry(const chassis::OdometryData& odom) {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp    = rclcpp::Time(odom.timestamp_us * 1000ULL);  // us → ns
    msg.header.frame_id = frame_id_;          // 默认 "odom"
    msg.child_frame_id  = child_frame_id_;    // 默认 "base_link"

    // 位姿
    msg.pose.pose.position.x    = odom.x;
    msg.pose.pose.position.y    = odom.y;
    msg.pose.pose.position.z    = odom.z;
    msg.pose.pose.orientation.x = odom.qx;
    msg.pose.pose.orientation.y = odom.qy;
    msg.pose.pose.orientation.z = odom.qz;
    msg.pose.pose.orientation.w = odom.qw;

    // 速度
    msg.twist.twist.linear.x  = odom.vx;
    msg.twist.twist.linear.y  = odom.vy;
    msg.twist.twist.angular.z = odom.omega;

    // 协方差（各 36 元素）
    for (size_t i = 0; i < 36; ++i) {
        msg.pose.covariance[i]  = odom.cov_pose[i];
        msg.twist.covariance[i] = odom.cov_twist[i];
    }

    odom_pub_->publish(msg);
}
```

**时间戳转换**：HAL 使用 `uint64_t` 微秒（`steady_clock`），ROS2 使用纳秒 int64，转换公式 `ns = us * 1000`。注意 `steady_clock` 不是 Unix epoch；若需绝对时间戳，需在 MCU 端或此处做时钟同步。

---

## 8. 状态回调 → `/chassis_status` + `/diagnostics`

### 8.1 `/chassis_status`（JSON 字符串）

```cpp
std::ostringstream oss;
oss << "{"
    << "\"timestamp_us\":" << status.timestamp_us << ","
    << "\"error_code\":"   << status.error_code   << ","
    << "\"error_msg\":\""  << status.error_msg    << "\","
    << "\"is_estop\":"     << (status.is_estop ? "true" : "false")
    << "}";
```

示例输出：
```json
{"timestamp_us":1234567890,"error_code":0,"error_msg":"","is_estop":false}
```

### 8.2 `/diagnostics`

```cpp
diagnostic_msgs::msg::DiagnosticStatus ds;
ds.name        = "chassis";
ds.hardware_id = "chassis_hal";
ds.level       = error_code == 0 ? OK : ERROR;
ds.message     = error_code == 0 ? "OK" : error_msg;
// KeyValue 列表
// {"error_code", to_string(error_code)}
// {"is_estop",   is_estop ? "true" : "false"}
```

---

## 9. 电池回调 → `/battery_state`

```cpp
sensor_msgs::msg::BatteryState msg;
msg.voltage      = float(battery.voltage);
msg.current      = float(battery.current);
msg.percentage   = float(battery.percentage / 100.0);  // 0–1 归一化
msg.temperature  = float(battery.temperature);
msg.present      = true;
msg.power_supply_status     = is_charging
    ? POWER_SUPPLY_STATUS_CHARGING
    : POWER_SUPPLY_STATUS_DISCHARGING;
msg.power_supply_health     = POWER_SUPPLY_HEALTH_GOOD;
msg.power_supply_technology = POWER_SUPPLY_TECHNOLOGY_LIPO;
// msg.cell_voltage = float(v) for each v in cell_voltages
```

---

## 10. 重连定时器

```cpp
reconnect_timer_ = node_->create_wall_timer(
    std::chrono::duration<double>(reconnect_interval_s_),  // 默认 5s
    [this]() {
        auto* hal = dynamic_cast<chassis::ChassisHal*>(controller_.get());
        if (hal && !hal->isConnected()) {
            RCLCPP_WARN(node_->get_logger(), "Chassis disconnected, reconnecting...");
            hal->connect();
        }
    });
```

**双重重连机制**：

| 机制 | 触发者 | 时机 |
|------|--------|------|
| `ChassisHal::reconnectLoop()` | HAL 内部线程 | 200ms 检测，立即反应 |
| `ChassisNode::reconnect_timer_` | ROS2 wall_timer | 5s 兜底，适用于 HAL 重连失败场景 |

---

## 11. HAL 回调注册时机

在 `ChassisNode` 构造函数中，控制器创建完毕后立即注册：

```cpp
controller_->setOdometryCallback(
    [this](const chassis::OdometryData& odom) { onOdometry(odom); });
controller_->setStatusCallback(
    [this](const chassis::ChassisStatus& st)  { onStatus(st); });
controller_->setBatteryCallback(
    [this](const chassis::BatteryData& bat)   { onBattery(bat); });
```

**线程安全注意**：回调在接收线程上下文中被调用，而 `odom_pub_->publish()` 等操作在单线程 executor 下与 spin 线程并发。生产环境建议使用 `rclcpp::CallbackGroup` 或将发布操作 post 到 executor 队列。

---

## 12. 析构顺序

```cpp
ChassisNode::~ChassisNode() {
    auto* hal = dynamic_cast<chassis::ChassisHal*>(controller_.get());
    if (hal) hal->disconnect();   // 停传输和重连线程
    // controller_ unique_ptr 析构 → ChassisHal::~ChassisHal() → 再次 disconnect()（幂等）
}
```
