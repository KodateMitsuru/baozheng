#include <rclcpp/rate.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include "servo_node/servo_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto servo_node = std::make_shared<ServoNode>(std::string("servo_node"));
    servo_node->main_loop();

    rclcpp::spin(servo_node);
    rclcpp::shutdown();
    return 0;
}