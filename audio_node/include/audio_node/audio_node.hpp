#ifndef AUDIO_NODE_HPP
#define AUDIO_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/subscription.hpp>
#include <string>
#include <alsa/asoundlib.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libswresample/swresample.h>
}



class AudioNode : public rclcpp::Node {
    public:
        AudioNode(std::string node_name);
        void init_audio();
        void play_audio(const std::string &file_path);
        void close_audio();
    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr audio_sub_;
        AVFormatContext* format_ctx;
        AVCodecContext* codec_ctx;
        AVCodec* codec;
        AVPacket* packet;
        AVFrame* frame;
        SwrContext *swr_ctx;
        snd_pcm_t *pcm_handle;
        snd_pcm_hw_params_t *params;
        static bool is_audio_playing;
};

#endif /* AUDIO_NODE_HPP */