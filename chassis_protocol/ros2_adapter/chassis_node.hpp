#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include "../chassis/ichassis_controller.hpp"

#include <memory>
#include <string>

namespace chassis_ros2 {

class ChassisNode {
public:
    explicit ChassisNode(rclcpp::Node::SharedPtr node);
    ~ChassisNode();

private:
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<chassis::IChassisController> controller_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr       cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr            odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr     battery_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr              status_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

    rclcpp::TimerBase::SharedPtr reconnect_timer_;

    // Parameters
    std::string transport_type_;
    std::string host_;
    int         port_{0};
    std::string device_;
    int         baud_rate_{115200};
    std::string chassis_type_;
    double      reconnect_interval_s_{5.0};
    std::string frame_id_{"odom"};
    std::string child_frame_id_{"base_link"};

    void declareParameters();
    void initController();

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void onOdometry(const chassis::OdometryData& odom);
    void onStatus(const chassis::ChassisStatus& status);
    void onBattery(const chassis::BatteryData& battery);
    void publishDiagnostics(const chassis::ChassisStatus& status);
};

} // namespace chassis_ros2
