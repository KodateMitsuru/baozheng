#include "servo_node/servo_node.hpp"

namespace py = pybind11;

ServoNode::ServoNode(std::string node_name) : 
    Node(node_name){
    
    //declare parameter
    this->declare_parameter("hz", 1);
    this->declare_parameter("alpha", 180);

    //set parameter
    hz = this->get_parameter("hz").as_int();
    alpha = this->get_parameter("alpha").as_int();

    //subscribe to the servo topic
    RCLCPP_INFO(this->get_logger(), "hz: %d", hz);
    servo_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        "/servo_node/mode",
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        [this](const std_msgs::msg::Int8::SharedPtr msg) {
            changeMode(*msg);
        }
    );
}

void ServoNode::changeMode(const std_msgs::msg::Int8 mode) {
    this->mode = mode.data;
    RCLCPP_INFO(this->get_logger(), "mode changed to %d", mode.data);
}

void ServoNode::init_servo() {
    // 初始化python接口
    py::initialize_interpreter();
    RCLCPP_INFO(this->get_logger(), "Current working directory: %s", py::module_::import("os").attr("getcwd")().cast<std::string>().c_str());
    py::exec(R"(import sys,os)");
    py::exec(R"(sys.path.append(os.getcwd()+'/servo_node/src/'))");
    RCLCPP_INFO(this->get_logger(), "sys.path: %s", py::str(py::module_::import("sys").attr("path")).cast<std::string>().c_str());

    try {
        servo = py::module_::import("servo");
    } catch (py::error_already_set &e) {
        RCLCPP_ERROR(this->get_logger(), "Can't import! Error: %s", e.what());
        close_servo();
        rclcpp::shutdown();
    }

    RCLCPP_INFO(this->get_logger(), "python module init successfully!");
    RCLCPP_INFO(this->get_logger(), "servo start init!");

    // 初始化参数
    A << 1, 1, 0, 1; // 状态转移矩阵
    H << 1, 0; // 观测矩阵
    R << 0.01, 0, 0, 2; // 过程噪声协方差矩阵
    Q << 2; // 观测噪声协方差矩阵
    x_k1 << 0, 0; // 初始状态估计

    // 使用reset方法初始化卡尔曼滤波器
    kf.reset(A, H, R, Q, x_k1, 1.0);

    RCLCPP_INFO(this->get_logger(), "kalman fliter init successfully!");

    //something to start the servo

    RCLCPP_INFO(this->get_logger(), "servo init successfully!");
}

void ServoNode::drive_servo() {

    auto updatedState = kf.update(Eigen::Matrix<double, V_Z, 1>(recieved_angle), 1.0);
    filtered_angle = double(updatedState[0]);
    switch(this->mode){
        case 0:
            break;
        case 1:{
            //something to drive the servo
            try {
                servo.attr("servo_rotate")(alpha);
                RCLCPP_INFO(this->get_logger(), "servo_rotate(%d)", alpha);
            } catch (py::error_already_set &e) {
                RCLCPP_ERROR(this->get_logger(), "Cant call servo_rotate! Error: %s", e.what());
                close_servo();
                rclcpp::shutdown();
            }
            break;
        }
        default:
            RCLCPP_ERROR(this->get_logger(), "%d is not a valid mode!", this->mode);
    }
};

void ServoNode::close_servo() {
    //something to close the servo
    //结束python接口初始化
    py::finalize_interpreter();
    RCLCPP_INFO(this->get_logger(),  "closed successfully!");
}



void ServoNode::main_loop() {
    std::thread([this](){
        init_servo();
        rclcpp::Rate rate(hz);
        while(rclcpp::ok()) {
            rate.sleep();
            drive_servo();
        }
        close_servo();
    }).detach();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto servo_node = std::make_shared<ServoNode>(std::string("servo_node"));
    
    servo_node->main_loop();

    rclcpp::spin(servo_node);
    rclcpp::shutdown();
    return 0;
}


