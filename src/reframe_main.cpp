#include "osv_metadata.h"
#include "osv_vulkan.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

static void writePPM(const std::string& path, int width, int height, const uint8_t* rgba)
{
    std::ofstream ofs(path, std::ios::binary);
    ofs << "P6\n" << width << " " << height << "\n255\n";
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const uint8_t* p = &rgba[(y * width + x) * 4];
            ofs.put(p[0]);
            ofs.put(p[1]);
            ofs.put(p[2]);
        }
    }
    std::cout << "[OSV-Reframe] Saved rectilinear frame to: " << path << std::endl;
}

static bool decodeFrame(AVFormatContext* fmt_ctx,
                        int stream_idx,
                        double seek_sec,
                        std::vector<uint8_t>& out_rgba,
                        int& width,
                        int& height)
{
    AVStream* stream = fmt_ctx->streams[stream_idx];
    const AVCodec* codec = avcodec_find_decoder(stream->codecpar->codec_id);
    if (!codec)
        return false;

    AVCodecContext* c_ctx = avcodec_alloc_context3(codec);
    avcodec_parameters_to_context(c_ctx, stream->codecpar);
    if (avcodec_open2(c_ctx, codec, nullptr) < 0) {
        avcodec_free_context(&c_ctx);
        return false;
    }

    if (seek_sec > 0.0) {
        int64_t ts = static_cast<int64_t>(seek_sec * AV_TIME_BASE);
        av_seek_frame(fmt_ctx, -1, ts, AVSEEK_FLAG_BACKWARD);
        avcodec_flush_buffers(c_ctx);
    }

    AVPacket* pkt = av_packet_alloc();
    AVFrame* frame = av_frame_alloc();
    AVFrame* frame_rgb = av_frame_alloc();

    width = c_ctx->width;
    height = c_ctx->height;
    out_rgba.resize(static_cast<size_t>(width) * height * 4);

    av_image_alloc(frame_rgb->data, frame_rgb->linesize, width, height, AV_PIX_FMT_RGBA, 1);

    SwsContext* sws = sws_getContext(width,
                                     height,
                                     c_ctx->pix_fmt,
                                     width,
                                     height,
                                     AV_PIX_FMT_RGBA,
                                     SWS_FAST_BILINEAR,
                                     nullptr,
                                     nullptr,
                                     nullptr);

    bool got_frame = false;
    while (av_read_frame(fmt_ctx, pkt) >= 0) {
        if (pkt->stream_index == stream_idx) {
            if (avcodec_send_packet(c_ctx, pkt) == 0) {
                while (avcodec_receive_frame(c_ctx, frame) == 0) {
                    double pts_sec = frame->pts * av_q2d(stream->time_base);
                    if (pts_sec >= seek_sec - 0.05) {
                        sws_scale(sws,
                                  frame->data,
                                  frame->linesize,
                                  0,
                                  height,
                                  frame_rgb->data,
                                  frame_rgb->linesize);
                        for (int y = 0; y < height; ++y) {
                            std::memcpy(&out_rgba[static_cast<size_t>(y) * width * 4],
                                        frame_rgb->data[0] + y * frame_rgb->linesize[0],
                                        static_cast<size_t>(width) * 4);
                        }
                        got_frame = true;
                        av_packet_unref(pkt);
                        break;
                    }
                }
            }
        }
        av_packet_unref(pkt);
        if (got_frame)
            break;
    }

    sws_freeContext(sws);
    av_freep(&frame_rgb->data[0]);
    av_frame_free(&frame_rgb);
    av_frame_free(&frame);
    av_packet_free(&pkt);
    avcodec_free_context(&c_ctx);

    return got_frame;
}

static std::string resolveReframeShaderPath()
{
    if (const char* env_shader = std::getenv("OSV_REFRAME_SHADER_PATH")) {
        if (std::filesystem::exists(env_shader)) {
            return env_shader;
        }
    }
    try {
        std::filesystem::path exe_dir = std::filesystem::read_symlink("/proc/self/exe").parent_path();
        std::vector<std::filesystem::path> candidates = {
            exe_dir / "shaders" / "direct_reframe.spv",
            exe_dir / ".." / "shaders" / "direct_reframe.spv",
        };
        for (const auto& c : candidates) {
            if (std::filesystem::exists(c)) {
                return c.string();
            }
        }
    } catch (...) {
    }
    return "shaders/direct_reframe.spv";
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0]
                  << " <osv_file> [yaw_deg] [pitch_deg] [roll_deg] [fov_deg] [out_ppm]\n";
        return 1;
    }

    std::string osv_path = argv[1];
    ReframeParams params;
    if (argc > 2) params.yaw_deg = std::stof(argv[2]);
    if (argc > 3) params.pitch_deg = std::stof(argv[3]);
    if (argc > 4) params.roll_deg = std::stof(argv[4]);
    if (argc > 5) params.fov_deg = std::stof(argv[5]);
    std::string out_ppm = (argc > 6) ? argv[6] : "reframed_direct.ppm";

    OsvTelemetrySource telemetry;
    DualLensCalibration cal;
    if (!telemetry.parseFromFile(osv_path, cal)) {
        std::cerr << "Failed to extract metadata and calibration from " << osv_path << std::endl;
        return 1;
    }

    AVFormatContext* fmt_ctx = nullptr;
    if (avformat_open_input(&fmt_ctx, osv_path.c_str(), nullptr, nullptr) < 0) {
        std::cerr << "Failed to open OSV file!" << std::endl;
        return 1;
    }
    if (avformat_find_stream_info(fmt_ctx, nullptr) < 0) {
        avformat_close_input(&fmt_ctx);
        return 1;
    }

    std::vector<uint8_t> frameSlave, frameMaster;
    int w0 = 0, h0 = 0, w1 = 0, h1 = 0;
    if (!decodeFrame(fmt_ctx, 0, 0.0, frameSlave, w0, h0)
        || !decodeFrame(fmt_ctx, 1, 0.0, frameMaster, w1, h1)) {
        std::cerr << "Failed to decode dual fisheye streams!" << std::endl;
        avformat_close_input(&fmt_ctx);
        return 1;
    }
    avformat_close_input(&fmt_ctx);

    DualLensVulkanStitcher stitcher;
    if (!stitcher.initializeReframe(resolveReframeShaderPath())) {
        std::cerr << "Failed to initialize direct reframe Vulkan pipeline!" << std::endl;
        return 1;
    }

    uint32_t out_w = 1920;
    uint32_t out_h = 1080;
    std::vector<uint8_t> out_rect(static_cast<size_t>(out_w) * out_h * 4);

    float gyro[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    telemetry.getStabilizationQuat(0, 1, gyro);

    std::cout << "[OSV-Reframe] Direct single-pass projection (" << out_w << "x" << out_h
              << ", yaw=" << params.yaw_deg << ", pitch=" << params.pitch_deg
              << ", roll=" << params.roll_deg << ", fov=" << params.fov_deg << ")..." << std::endl;

    if (!stitcher.reframeFrames(frameMaster.data(),
                                frameSlave.data(),
                                static_cast<uint32_t>(w1),
                                static_cast<uint32_t>(h1),
                                cal.lens_master,
                                cal.lens_slave,
                                gyro,
                                params,
                                out_w,
                                out_h,
                                out_rect.data())) {
        std::cerr << "[OSV-Reframe] Direct reframe failed!" << std::endl;
        return 1;
    }

    writePPM(out_ppm, static_cast<int>(out_w), static_cast<int>(out_h), out_rect.data());
    return 0;
}
