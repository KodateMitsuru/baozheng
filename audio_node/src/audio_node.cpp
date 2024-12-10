#include <audio_node/audio_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <iostream>
#include <vector>
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
#include <libswresample/swresample.h>
#include <libavutil/channel_layout.h>
#include <alsa/asoundlib.h>

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

void AudioNode::play_audio(const std::string &file_path) {


        AVFormatContext *format_ctx = avformat_alloc_context();
        if (avformat_open_input(&format_ctx, file_path.c_str(), nullptr, nullptr) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", file_path.c_str());
            return;
        }

        if (avformat_find_stream_info(format_ctx, nullptr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Could not find stream information");
            avformat_close_input(&format_ctx);
            return;
        }

        const AVCodec *codec = nullptr;
        AVCodecParameters *codec_params = nullptr;
        int audio_stream_index = -1;

        for (unsigned int i = 0; i < format_ctx->nb_streams; i++) {
            AVCodecParameters *local_codec_params = format_ctx->streams[i]->codecpar;
            const AVCodec *local_codec = avcodec_find_decoder(local_codec_params->codec_id);

            if (local_codec_params->codec_type == AVMEDIA_TYPE_AUDIO) {
                audio_stream_index = i;
                codec = local_codec;
                codec_params = local_codec_params;
                break;
            }
        }

        if (audio_stream_index == -1) {
            RCLCPP_ERROR(this->get_logger(), "Could not find audio stream");
            avformat_close_input(&format_ctx);
            return;
        }

        AVCodecContext *codec_ctx = avcodec_alloc_context3(codec);
        if (!codec_ctx) {
            RCLCPP_ERROR(this->get_logger(), "Could not allocate codec context");
            avformat_close_input(&format_ctx);
            return;
        }

        if (avcodec_parameters_to_context(codec_ctx, codec_params) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Could not initialize codec context");
            avcodec_free_context(&codec_ctx);
            avformat_close_input(&format_ctx);
            return;
        }

        if (avcodec_open2(codec_ctx, codec, nullptr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Could not open codec");
            avcodec_free_context(&codec_ctx);
            avformat_close_input(&format_ctx);
            return;
        }

        SwrContext *swr_ctx = swr_alloc();
        int64_t in_channel_layout = av_get_channel_layout_nb_channels(codec_ctx->ch_layout.nb_channels);
        int64_t out_channel_layout = av_get_channel_layout_nb_channels(codec_ctx->ch_layout.nb_channels);
        av_opt_set_int(swr_ctx, "in_channel_layout", in_channel_layout, 0);
        av_opt_set_int(swr_ctx, "out_channel_layout", out_channel_layout, 0);
        av_opt_set_int(swr_ctx, "in_sample_rate", codec_ctx->sample_rate, 0);
        av_opt_set_int(swr_ctx, "out_sample_rate", codec_ctx->sample_rate, 0);
        av_opt_set_sample_fmt(swr_ctx, "in_sample_fmt", codec_ctx->sample_fmt, 0);
        av_opt_set_sample_fmt(swr_ctx, "out_sample_fmt", AV_SAMPLE_FMT_S16, 0);
        swr_init(swr_ctx);

        snd_pcm_t *pcm_handle;
        snd_pcm_hw_params_t *params;
        unsigned int sample_rate = codec_ctx->sample_rate;
        int dir;

        if (snd_pcm_open(&pcm_handle, "default", SND_PCM_STREAM_PLAYBACK, 0) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Could not open audio device");
            swr_free(&swr_ctx);
            avcodec_free_context(&codec_ctx);
            avformat_close_input(&format_ctx);
            return;
        }

        snd_pcm_hw_params_alloca(&params);
        snd_pcm_hw_params_any(pcm_handle, params);
        snd_pcm_hw_params_set_access(pcm_handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
        snd_pcm_hw_params_set_format(pcm_handle, params, SND_PCM_FORMAT_S16_LE);
        snd_pcm_hw_params_set_channels(pcm_handle, params, codec_ctx->channels);
        snd_pcm_hw_params_set_rate_near(pcm_handle, params, &sample_rate, &dir);
        snd_pcm_hw_params(pcm_handle, params);

        AVPacket *packet = av_packet_alloc();
        AVFrame *frame = av_frame_alloc();
        uint8_t **converted_data = nullptr;
        int max_samples = 0;

        while (av_read_frame(format_ctx, packet) >= 0) {
            if (packet->stream_index == audio_stream_index) {
                if (avcodec_send_packet(codec_ctx, packet) >= 0) {
                    while (avcodec_receive_frame(codec_ctx, frame) >= 0) {
                        int num_samples = swr_get_out_samples(swr_ctx, frame->nb_samples);
                        if (num_samples > max_samples) {
                            if (converted_data) {
                                av_freep(&converted_data[0]);
                            }
                            av_samples_alloc_array_and_samples(&converted_data, nullptr, codec_ctx->channels, num_samples, AV_SAMPLE_FMT_S16, 0);
                            max_samples = num_samples;
                        }

                        swr_convert(swr_ctx, converted_data, num_samples, (const uint8_t **)frame->data, frame->nb_samples);
                        snd_pcm_writei(pcm_handle, converted_data[0], num_samples);
                    }
                }
            }
            av_packet_unref(packet);
        }

        if (converted_data) {
            av_freep(&converted_data[0]);
        }
        av_freep(&converted_data);
        av_frame_free(&frame);
        av_packet_free(&packet);
        snd_pcm_drain(pcm_handle);
        snd_pcm_close(pcm_handle);
        swr_free(&swr_ctx);
        avcodec_free_context(&codec_ctx);
        avformat_close_input(&format_ctx);
    }


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AudioNode>(std::string("audio_node"));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
