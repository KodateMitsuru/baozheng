#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <rclcpp/subscription.hpp>
#include <string>
#include "kalman.hpp"

class ServoNode : public rclcpp::Node {
public:
    ServoNode(std::string node_name); 
    void init_servo();
    void drive_servo();
    void close_servo();
    void main_loop();
    void changeMode(const std_msgs::msg::Int8 mode);

private:
    int mode = 0;
    int hz;
    static constexpr int V_Z = 1; 
    static constexpr int V_X = 2; 
    double recieved_angle;
    double filtered_angle;
    double current_angle;

    // 定义初始参数
    Eigen::Matrix<double, V_X, V_X> A; // 状态转移矩阵
    Eigen::Matrix<double, V_Z, V_X> H; // 观测矩阵
    Eigen::Matrix<double, V_X, V_X> R; // 过程噪声协方差矩阵
    Eigen::Matrix<double, V_Z, V_Z> Q; // 观测噪声协方差矩阵
    Eigen::Matrix<double, V_X, 1> x_k1; // 初始状态估计
    
    Kalman<V_Z, V_X> kf;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr servo_sub_;

    // rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_pub_;
};