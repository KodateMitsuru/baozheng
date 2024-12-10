#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <rclcpp/subscription.hpp>
#include <string>


class AudioNode : public rclcpp::Node {
public:
    AudioNode(std::string node_name);
    void play_audio(const std::string &file_path);
};