#include <ao/ao.h>
#include <audio_node/audio_node.hpp>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sndfile.h>
#include <string>
#include <thread>

AudioNode::AudioNode(const std::string node_name) : Node(node_name) {
    
    // declare parameter
    this->declare_parameter("audio_file", "assets/audio.wav");
    
    // subscribe to the audio topic
    audio_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/audio_node/play",
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        [this](const std_msgs::msg::String file_path) {
            this->play_audio(file_path.data);
        }
    );
}

AudioNode::play_audio(const std::string &file_path) {
    // initialize audio output
    ao_initialize();
    int driver = ao_default_driver_id();
    ao_sample_format format;
    format.bits = 16;
    format.channels = 2;
    format.rate = 44100;
    format.byte_format = AO_FMT_LITTLE;
    format.matrix = 0;
    ao_device *device = ao_open_live(driver, &format, nullptr);
    if (device == nullptr) {
        RCLCPP_ERROR(this->get_logger(), "Error opening device");
        return;
    }

    // open audio file
    SF_INFO sfinfo;
    SNDFILE *sndfile = sf_open(file_path.c_str(), SFM_READ, &sfinfo);
    if (sndfile == nullptr) {
        RCLCPP_ERROR(this->get_logger(), "Error opening file");
        return;
    }

    // read audio file
    const int buffer_size = 4096;
    int *buffer = new int[buffer_size];
    while (true) {
        sf_count_t read = sf_read_int(sndfile, buffer, buffer_size);
        if (read == 0) {
            break;
        }
        ao_play(device, (char *)buffer, read * sizeof(int));
    }

    // close audio file
    sf_close(sndfile);
    delete[] buffer;

    // close audio output
    ao_close(device);
    ao_shutdown();
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AudioNode>(std::string("servo_node"));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
