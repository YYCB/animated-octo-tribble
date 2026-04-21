#include "chassis_node.hpp"

#include "../chassis/differential/differential_controller.hpp"
#include "../chassis/mecanum/mecanum_controller.hpp"
#include "../transport/itransport.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <array>
#include <sstream>
#include <stdexcept>
#include <string>

namespace chassis_ros2 {

ChassisNode::ChassisNode(rclcpp::Node::SharedPtr node)
    : node_(std::move(node))
{
    declareParameters();
    initController();

    // Publishers
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    battery_pub_ = node_->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);
    status_pub_ = node_->create_publisher<std_msgs::msg::String>("chassis_status", 10);
    diag_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

    // Subscriber
    cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            cmdVelCallback(msg);
        });

    // Register HAL callbacks
    if (controller_) {
        controller_->setOdometryCallback(
            [this](const chassis::OdometryData& odom) { onOdometry(odom); });
        controller_->setStatusCallback(
            [this](const chassis::ChassisStatus& st) { onStatus(st); });
        controller_->setBatteryCallback(
            [this](const chassis::BatteryData& bat) { onBattery(bat); });
    }

    // Reconnect timer
    reconnect_timer_ = node_->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(reconnect_interval_s_)),
        [this]() {
            if (!controller_) return;
            auto* hal = dynamic_cast<chassis::ChassisHal*>(controller_.get());
            if (hal && !hal->isConnected()) {
                RCLCPP_WARN(node_->get_logger(), "Chassis disconnected, attempting reconnect...");
                hal->connect();
            }
        });
    (void)reconnect_timer_; // suppress unused warning
}

ChassisNode::~ChassisNode() {
    auto* hal = dynamic_cast<chassis::ChassisHal*>(controller_.get());
    if (hal) {
        hal->disconnect();
    }
}

void ChassisNode::declareParameters() {
    node_->declare_parameter("transport_type", std::string("tcp"));
    node_->declare_parameter("host",           std::string("192.168.1.100"));
    node_->declare_parameter("port",           8899);
    node_->declare_parameter("device",         std::string("/dev/ttyUSB0"));
    node_->declare_parameter("baud_rate",      115200);
    node_->declare_parameter("chassis_type",   std::string("differential"));
    node_->declare_parameter("reconnect_interval_s", 5.0);
    node_->declare_parameter("frame_id",       std::string("odom"));
    node_->declare_parameter("child_frame_id", std::string("base_link"));
}

void ChassisNode::initController() {
    transport_type_      = node_->get_parameter("transport_type").as_string();
    host_                = node_->get_parameter("host").as_string();
    port_                = static_cast<int>(node_->get_parameter("port").as_int());
    device_              = node_->get_parameter("device").as_string();
    baud_rate_           = static_cast<int>(node_->get_parameter("baud_rate").as_int());
    chassis_type_        = node_->get_parameter("chassis_type").as_string();
    reconnect_interval_s_ = node_->get_parameter("reconnect_interval_s").as_double();
    frame_id_            = node_->get_parameter("frame_id").as_string();
    child_frame_id_      = node_->get_parameter("child_frame_id").as_string();

    // Build transport config
    chassis::TransportConfig cfg;
    cfg.host        = host_;
    cfg.port        = static_cast<uint16_t>(port_);
    cfg.device_path = device_;
    cfg.baud_rate   = static_cast<uint32_t>(baud_rate_);
    cfg.timeout_ms  = 3000;

    chassis::TransportType ttype = chassis::TransportType::TCP;
    if (transport_type_ == "udp") {
        ttype = chassis::TransportType::UDP;
    } else if (transport_type_ == "rs485") {
        ttype = chassis::TransportType::RS485;
    }

    auto transport = chassis::ITransport::create(ttype, cfg);

    if (chassis_type_ == "mecanum") {
        auto ctrl = std::make_unique<chassis::MecanumController>(std::move(transport));
        if (!ctrl->connect()) {
            RCLCPP_WARN(node_->get_logger(),
                        "Initial chassis connection failed; will retry.");
        }
        controller_ = std::move(ctrl);
    } else {
        // Default to differential
        auto ctrl = std::make_unique<chassis::DifferentialController>(std::move(transport));
        if (!ctrl->connect()) {
            RCLCPP_WARN(node_->get_logger(),
                        "Initial chassis connection failed; will retry.");
        }
        controller_ = std::move(ctrl);
    }
}

void ChassisNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!controller_) return;
    controller_->sendVelocity(msg->linear.x, msg->linear.y, msg->angular.z);
}

void ChassisNode::onOdometry(const chassis::OdometryData& odom) {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp    = rclcpp::Time(static_cast<int64_t>(odom.timestamp_us * 1000ULL));
    msg.header.frame_id = frame_id_;
    msg.child_frame_id  = child_frame_id_;

    msg.pose.pose.position.x    = odom.x;
    msg.pose.pose.position.y    = odom.y;
    msg.pose.pose.position.z    = odom.z;
    msg.pose.pose.orientation.x = odom.qx;
    msg.pose.pose.orientation.y = odom.qy;
    msg.pose.pose.orientation.z = odom.qz;
    msg.pose.pose.orientation.w = odom.qw;

    msg.twist.twist.linear.x  = odom.vx;
    msg.twist.twist.linear.y  = odom.vy;
    msg.twist.twist.angular.z = odom.omega;

    // Copy covariance (36 elements each)
    for (std::size_t i = 0; i < 36; ++i) {
        msg.pose.covariance[i]  = odom.cov_pose[i];
        msg.twist.covariance[i] = odom.cov_twist[i];
    }

    odom_pub_->publish(msg);
}

void ChassisNode::onStatus(const chassis::ChassisStatus& status) {
    // Publish as JSON-ish string
    std::ostringstream oss;
    oss << "{"
        << "\"timestamp_us\":" << status.timestamp_us << ","
        << "\"error_code\":"   << status.error_code   << ","
        << "\"error_msg\":\""  << status.error_msg    << "\","
        << "\"is_estop\":"     << (status.is_estop ? "true" : "false")
        << "}";

    std_msgs::msg::String msg;
    msg.data = oss.str();
    status_pub_->publish(msg);

    publishDiagnostics(status);
}

void ChassisNode::onBattery(const chassis::BatteryData& battery) {
    sensor_msgs::msg::BatteryState msg;
    msg.header.stamp = node_->now();
    msg.voltage      = static_cast<float>(battery.voltage);
    msg.current      = static_cast<float>(battery.current);
    msg.percentage   = static_cast<float>(battery.percentage / 100.0);
    msg.temperature  = static_cast<float>(battery.temperature);
    msg.present      = true;
    msg.power_supply_status =
        battery.is_charging
            ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING
            : sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    msg.power_supply_technology =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;

    msg.cell_voltage.reserve(battery.cell_voltages.size());
    for (double cv : battery.cell_voltages) {
        msg.cell_voltage.push_back(static_cast<float>(cv));
    }

    battery_pub_->publish(msg);
}

void ChassisNode::publishDiagnostics(const chassis::ChassisStatus& status) {
    diagnostic_msgs::msg::DiagnosticArray diag_array;
    diag_array.header.stamp = node_->now();

    diagnostic_msgs::msg::DiagnosticStatus ds;
    ds.name      = "chassis";
    ds.hardware_id = "chassis_hal";
    ds.level     = status.error_code == 0
                       ? diagnostic_msgs::msg::DiagnosticStatus::OK
                       : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    ds.message   = status.error_code == 0 ? "OK" : status.error_msg;

    diagnostic_msgs::msg::KeyValue kv_code;
    kv_code.key   = "error_code";
    kv_code.value = std::to_string(status.error_code);
    ds.values.push_back(kv_code);

    diagnostic_msgs::msg::KeyValue kv_estop;
    kv_estop.key   = "is_estop";
    kv_estop.value = status.is_estop ? "true" : "false";
    ds.values.push_back(kv_estop);

    diag_array.status.push_back(ds);
    diag_pub_->publish(diag_array);
}

} // namespace chassis_ros2
