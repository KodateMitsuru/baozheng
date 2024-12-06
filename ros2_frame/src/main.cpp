#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include "ros2_frame/servo_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto servo_node = std::make_shared<ServoNode>(std::string("servo_node"),180);
    servo_node->main_loop();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(servo_node);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}