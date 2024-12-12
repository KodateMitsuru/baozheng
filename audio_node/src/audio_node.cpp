#include <audio_node/audio_node.hpp>
#include <thread>

AudioNode::AudioNode(const std::string node_name) : Node(node_name){
    // subscribe to the audio topic
    audio_sub_play = this->create_subscription<std_msgs::msg::String>(
        "/audio_node/play",
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        [this](const std_msgs::msg::String file_path) {
            if (is_audio_playing) {
                RCLCPP_ERROR(get_logger(), "Audio is already playing");
            } else std::thread([this, file_path](){this->play_audio(file_path.data);}).detach();
        }
    );
    audio_sub_stop = this->create_subscription<std_msgs::msg::Bool>(
        "/audio_node/stop",
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        [this](const std_msgs::msg::Bool stop) {
            if (stop.data) {
                if (is_audio_playing.load()) {
                    is_audio_playing.store(false);
                    RCLCPP_INFO(this->get_logger(), "Stopping audio playback");
                } else RCLCPP_ERROR(this->get_logger(), "No audio is playing");
            }
        }
    );
}

std::atomic<bool>  AudioNode::is_audio_playing;

void AudioNode::init_audio() {
    int ret = snd_pcm_open(&pcm_handle, "default", SND_PCM_STREAM_PLAYBACK, 0);
    if (ret < 0) {
        RCLCPP_ERROR(this->get_logger(), "Could not open PCM device: %s", snd_strerror(ret));
        rclcpp::shutdown();
    }
    // 设置硬件参数
    snd_pcm_hw_params_alloca(&params);
    snd_pcm_hw_params_any(pcm_handle, params);
    snd_pcm_hw_params_set_access(pcm_handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(pcm_handle, params, SND_PCM_FORMAT_S16_LE);
    snd_pcm_hw_params_set_channels(pcm_handle, params, 2);
    snd_pcm_hw_params_set_rate(pcm_handle, params, 44100, 0);
    // 设置硬件参数后，应用它们
    if (snd_pcm_hw_params(pcm_handle, params) < 0) {
        RCLCPP_ERROR(this->get_logger(), "无法设置 PCM 硬件参数");
        rclcpp::shutdown();
    }
}

void AudioNode::play_audio(const std::string &file_path) {
    this->init_audio();

    RCLCPP_INFO(this->get_logger(), "Playing audio file: %s", file_path.c_str());
    // Open the audio file
    if (avformat_open_input(&format_ctx, file_path.c_str(), nullptr, nullptr) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Could not open audio file: %s", file_path.c_str());
        return;
    }

    // Retrieve stream information
    if (avformat_find_stream_info(format_ctx, nullptr) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Could not find stream information");
        return;
    }

    // Find the first audio stream
    int audio_stream_index = -1;
    for (unsigned i = 0; i < format_ctx->nb_streams; i++) {
        if (format_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_AUDIO) {
            audio_stream_index = i;
            break;
        }
    }

    if (audio_stream_index == -1) {
        RCLCPP_ERROR(this->get_logger(), "Could not find audio stream");
        return;
    }
    
    // Open the codec context for the audio stream
    AVCodecContext *codec_ctx = avcodec_alloc_context3(nullptr);
    if (!codec_ctx) {
        RCLCPP_ERROR(this->get_logger(), "Could not allocate codec context");
        return;
    }
    if (avcodec_parameters_to_context(codec_ctx, format_ctx->streams[audio_stream_index]->codecpar) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Could not copy codec parameters to codec context");
        return;
    }
    if (!avcodec_find_decoder(codec_ctx->codec_id)) {
        RCLCPP_ERROR(this->get_logger(), "Could not find decoder");
        return;
    }
    codec_ctx->pkt_timebase = format_ctx->streams[audio_stream_index]->time_base;
    if (avcodec_open2(codec_ctx, avcodec_find_decoder(codec_ctx->codec_id), nullptr) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Could not open codec");
        return;
    }
    AVChannelLayout out_ch_layout = AV_CHANNEL_LAYOUT_STEREO;
    AVChannelLayout in_ch_layout = codec_ctx->ch_layout;
    swr_alloc_set_opts2(
        &swr_ctx,
        &out_ch_layout,
        AV_SAMPLE_FMT_S16,                   // 输出采样格式（16 位）
        44100,                               // 输出采样率
        &in_ch_layout,  // 输入通道布局
        codec_ctx->sample_fmt,               // 输入采样格式
        codec_ctx->sample_rate,              // 输入采样率
        0, nullptr
    );
    if (swr_init(swr_ctx) < 0) {
        RCLCPP_ERROR(this->get_logger(), "无法初始化重采样上下文");
        return;
    }
    // Allocate packet and frame
    packet = av_packet_alloc();
    frame = av_frame_alloc();
    is_audio_playing.store(true);
    while (av_read_frame(format_ctx, packet) >= 0 && rclcpp::ok() && is_audio_playing.load()) {
        if (packet->stream_index == audio_stream_index) {
            if (avcodec_send_packet(codec_ctx, packet) < 0) {
                break;
            }
            while (avcodec_receive_frame(codec_ctx, frame) >= 0 && rclcpp::ok() && is_audio_playing.load()) {
                // 计算输出采样数
                int out_samples = av_rescale_rnd(
                    swr_get_delay(swr_ctx, codec_ctx->sample_rate) + frame->nb_samples,
                    44100, codec_ctx->sample_rate, AV_ROUND_UP
                );

                // 分配输出缓冲区
                int out_channels = 2; // 立体声
                int out_buffer_size = av_samples_get_buffer_size(
                    nullptr, out_channels, out_samples, AV_SAMPLE_FMT_S16, 1
                );
                uint8_t *out_buffer = (uint8_t *)av_malloc(out_buffer_size);

                // 进行重采样和格式转换
                int converted_samples = swr_convert(
                    swr_ctx,
                    &out_buffer,
                    out_samples,
                    (const uint8_t **)frame->data,
                    frame->nb_samples
                );
                if (converted_samples < 0) {
                    RCLCPP_ERROR(this->get_logger(), "重采样时出错");
                    av_free(out_buffer);
                    break;
                }

                // 写入 PCM 设备
                int frames_written = snd_pcm_writei(pcm_handle, out_buffer, converted_samples);
                if (frames_written < 0) {
                    if (frames_written == -EPIPE) {
                        // 缓冲区溢出，需要恢复 PCM 设备
                        snd_pcm_prepare(pcm_handle);
                    } else if (frames_written == -ESTRPIPE) {
                        // 流暂停，需要恢复
                        while ((frames_written = snd_pcm_resume(pcm_handle)) == -EAGAIN) {
                            sleep(1); // 等待一段时间后重试
                        }
                        if (frames_written < 0) {
                            snd_pcm_prepare(pcm_handle);
                        }
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "PCM 写入错误: %s", snd_strerror(frames_written));
                    }
                }

                av_free(out_buffer);
            }
        }
        av_packet_unref(packet);
    }
    is_audio_playing.store(false);
    RCLCPP_INFO(this->get_logger(), "Audio playback finished");

    // 清理
    swr_free(&swr_ctx);
    av_frame_free(&frame);
    av_packet_free(&packet);
    avcodec_free_context(&codec_ctx);
    avformat_close_input(&format_ctx);
    this->close_audio();
}

void AudioNode::close_audio() {
    snd_pcm_drop(pcm_handle);
    snd_pcm_close(pcm_handle);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AudioNode>(std::string("audio_node"));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
