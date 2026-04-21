#include <rclcpp/rclcpp.hpp>
#include "chassis_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("chassis_node");
    chassis_ros2::ChassisNode chassis_node(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
