#include "osv_metadata.h"
#include "osv_vulkan.h"

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
}

#include <iostream>
#include <vector>
#include <fstream>
#include <cstring>
#include <filesystem>

static void writePPM(const std::string& path, int width, int height, const uint8_t* rgba) {
    std::ofstream ofs(path, std::ios::binary);
    ofs << "P6\n" << width << " " << height << "\n255\n";
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const uint8_t* p = &rgba[(y * width + x) * 4];
            ofs.put(p[0]); // R
            ofs.put(p[1]); // G
            ofs.put(p[2]); // B
        }
    }
    std::cout << "[OSV] Saved stitched frame to: " << path << std::endl;
}

static bool decodeFrame(AVFormatContext* fmt_ctx, int stream_idx, double seek_sec, std::vector<uint8_t>& out_rgba, int& width, int& height) {
    AVStream* stream = fmt_ctx->streams[stream_idx];
    const AVCodec* codec = avcodec_find_decoder(stream->codecpar->codec_id);
    if (!codec) return false;

    AVCodecContext* c_ctx = avcodec_alloc_context3(codec);
    avcodec_parameters_to_context(c_ctx, stream->codecpar);
    if (avcodec_open2(c_ctx, codec, nullptr) < 0) {
        avcodec_free_context(&c_ctx);
        return false;
    }

    if (seek_sec > 0.0) {
        int64_t ts = (int64_t)(seek_sec * AV_TIME_BASE);
        av_seek_frame(fmt_ctx, -1, ts, AVSEEK_FLAG_BACKWARD);
        avcodec_flush_buffers(c_ctx);
    }

    AVPacket* pkt = av_packet_alloc();
    AVFrame* frame = av_frame_alloc();
    AVFrame* frame_rgb = av_frame_alloc();

    width = c_ctx->width;
    height = c_ctx->height;
    out_rgba.resize(width * height * 4);

    av_image_alloc(frame_rgb->data, frame_rgb->linesize, width, height, AV_PIX_FMT_RGBA, 1);

    SwsContext* sws = sws_getContext(
        width, height, c_ctx->pix_fmt,
        width, height, AV_PIX_FMT_RGBA,
        SWS_FAST_BILINEAR, nullptr, nullptr, nullptr
    );

    bool got_frame = false;

    while (av_read_frame(fmt_ctx, pkt) >= 0) {
        if (pkt->stream_index == stream_idx) {
            if (avcodec_send_packet(c_ctx, pkt) == 0) {
                while (avcodec_receive_frame(c_ctx, frame) == 0) {
                    double pts_sec = frame->pts * av_q2d(stream->time_base);
                    if (pts_sec >= seek_sec - 0.05) {
                        sws_scale(sws, frame->data, frame->linesize, 0, height, frame_rgb->data, frame_rgb->linesize);
                        for (int y = 0; y < height; ++y) {
                            std::memcpy(&out_rgba[y * width * 4], frame_rgb->data[0] + y * frame_rgb->linesize[0], width * 4);
                        }
                        got_frame = true;
                        av_packet_unref(pkt);
                        break;
                    }
                }
            }
        }
        av_packet_unref(pkt);
        if (got_frame) break;
    }

    sws_freeContext(sws);
    av_freep(&frame_rgb->data[0]);
    av_frame_free(&frame_rgb);
    av_frame_free(&frame);
    av_packet_free(&pkt);
    avcodec_free_context(&c_ctx);

    return got_frame;
}

int main(int argc, char** argv) {
    std::string osv_path = "/home/fra/Videos/2026-11-03/CAM_20261103144622_0006_D.OSV";
    bool raw_images_mode = false;
    std::string raw_master_path, raw_slave_path;
    std::string custom_out_ppm;

    if (argc >= 4 && std::string(argv[1]) == "--raw") {
        raw_images_mode = true;
        raw_master_path = argv[2];
        raw_slave_path = argv[3];
        osv_path = argv[4];
        if (argc > 5) {
            custom_out_ppm = argv[5];
        }
    } else if (argc > 1) {
        osv_path = argv[1];
    }

    std::cout << "========================================" << std::endl;
    std::cout << "DJI Osmo 360 Native Vulkan Stitcher" << std::endl;
    std::cout << "Processing: " << osv_path << std::endl;
    std::cout << "========================================" << std::endl;

    // Step 1: Parse Metadata
    OsvMetadata meta;
    if (!OsvMetadataParser::parseFromFile(osv_path, meta)) {
        std::cerr << "Failed to extract metadata and calibration!" << std::endl;
        return 1;
    }

    std::cout << "[Metadata] Camera Model: " << meta.camera_model << std::endl;
    std::cout << "[Metadata] Proto Schema: " << meta.proto_name << std::endl;
    std::cout << "[Metadata] Serial:       " << meta.serial_number << std::endl;
    std::cout << "[Calibration] Lens 1 (Slave, Stream 0):" << std::endl;
    std::cout << "  fx=" << meta.lens_front.fx << " fy=" << meta.lens_front.fy 
              << " cx=" << meta.lens_front.cx << " cy=" << meta.lens_front.cy << std::endl;
    std::cout << "  k=[" << meta.lens_front.k1 << ", " << meta.lens_front.k2 
              << ", " << meta.lens_front.k3 << ", " << meta.lens_front.k4 << "]" << std::endl;
    std::cout << "  rot=[w=" << meta.lens_front.quat[0] << ", x=" << meta.lens_front.quat[1] 
              << ", y=" << meta.lens_front.quat[2] << ", z=" << meta.lens_front.quat[3] << "]" << std::endl;

    std::cout << "[Calibration] Lens 2 (Master, Stream 1):" << std::endl;
    std::cout << "  fx=" << meta.lens_rear.fx << " fy=" << meta.lens_rear.fy 
              << " cx=" << meta.lens_rear.cx << " cy=" << meta.lens_rear.cy << std::endl;
    std::cout << "  k=[" << meta.lens_rear.k1 << ", " << meta.lens_rear.k2 
              << ", " << meta.lens_rear.k3 << ", " << meta.lens_rear.k4 << "]" << std::endl;
    std::cout << "  rot=[w=" << meta.lens_rear.quat[0] << ", x=" << meta.lens_rear.quat[1] 
              << ", y=" << meta.lens_rear.quat[2] << ", z=" << meta.lens_rear.quat[3] << "]" << std::endl;

    // Step 2: Decode 1 frame from each lens stream
    std::vector<uint8_t> frameSlave, frameMaster;
    int w0 = 0, h0 = 0, w1 = 0, h1 = 0;
    double seek_sec = 0.0;

    if (raw_images_mode) {
        std::cout << "[Decode] Loading raw Master frame from: " << raw_master_path << std::endl;
        AVFormatContext* fmt_m = nullptr;
        if (avformat_open_input(&fmt_m, raw_master_path.c_str(), nullptr, nullptr) >= 0) {
            avformat_find_stream_info(fmt_m, nullptr);
            decodeFrame(fmt_m, 0, 0.0, frameMaster, w1, h1);
            avformat_close_input(&fmt_m);
        }
        std::cout << "[Decode] Loading raw Slave frame from: " << raw_slave_path << std::endl;
        AVFormatContext* fmt_s = nullptr;
        if (avformat_open_input(&fmt_s, raw_slave_path.c_str(), nullptr, nullptr) >= 0) {
            avformat_find_stream_info(fmt_s, nullptr);
            decodeFrame(fmt_s, 0, 0.0, frameSlave, w0, h0);
            avformat_close_input(&fmt_s);
        }
    } else {
        AVFormatContext* fmt_ctx = nullptr;
        if (avformat_open_input(&fmt_ctx, osv_path.c_str(), nullptr, nullptr) < 0) {
            std::cerr << "Failed to open video file" << std::endl;
            return 1;
        }
        avformat_find_stream_info(fmt_ctx, nullptr);

        if (argc > 2) {
            seek_sec = std::stod(argv[2]);
        }

        std::cout << "[Decode] Decoding Slave Lens (Stream 0) at " << seek_sec << "s..." << std::endl;
        decodeFrame(fmt_ctx, 0, seek_sec, frameSlave, w0, h0);

        // Reopen context for stream 1 seek
        avformat_close_input(&fmt_ctx);
        fmt_ctx = nullptr;
        avformat_open_input(&fmt_ctx, osv_path.c_str(), nullptr, nullptr);
        avformat_find_stream_info(fmt_ctx, nullptr);

        std::cout << "[Decode] Decoding Master Lens (Stream 1) at " << seek_sec << "s..." << std::endl;
        decodeFrame(fmt_ctx, 1, seek_sec, frameMaster, w1, h1);

        avformat_close_input(&fmt_ctx);
    }

    std::cout << "[Decode] Stream 0: " << w0 << "x" << h0 << " | Stream 1: " << w1 << "x" << h1 << std::endl;
    if (w0 == 0 || w1 == 0) {
        std::cerr << "Error decoding frames!" << std::endl;
        return 1;
    }

    // Step 3: Initialize Vulkan Stitcher
    OsvVulkanStitcher stitcher;
    std::string shader_path = "shaders/osv_stitch.spv";
    const char* env_shader = std::getenv("OSV_SHADER_PATH");
    if (env_shader && std::filesystem::exists(env_shader)) {
        shader_path = env_shader;
    } else if (!std::filesystem::exists(shader_path)) {
        try {
            std::filesystem::path exe_dir = std::filesystem::read_symlink("/proc/self/exe").parent_path();
            std::vector<std::filesystem::path> candidates = {
                exe_dir / "shaders" / "osv_stitch.spv",
                exe_dir / ".." / "shaders" / "osv_stitch.spv",
                "../shaders/osv_stitch.spv"
            };
            for (const auto& c : candidates) {
                if (std::filesystem::exists(c)) {
                    shader_path = c.string();
                    break;
                }
            }
        } catch (...) {
            if (std::filesystem::exists("../shaders/osv_stitch.spv")) {
                shader_path = "../shaders/osv_stitch.spv";
            }
        }
    }
    if (!stitcher.initialize(shader_path)) {
        std::cerr << "Failed to initialize Vulkan compute pipeline!" << std::endl;
        return 1;
    }

    // Step 4: Stitch to Equirectangular (3840x1920)
    uint32_t out_w = 3840;
    uint32_t out_h = 1920;
    std::vector<uint8_t> out_equirect(out_w * out_h * 4);

    const char* env_radius = std::getenv("OSV_STITCH_RADIUS");
    if (env_radius) {
        stitcher.setStitchRadius(std::atof(env_radius));
    }

    std::cout << "[Vulkan] Executing GPU compute stitch (" << out_w << "x" << out_h << ")..." << std::endl;
    float gyro[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // Identity (w, x, y, z)
    int frame_idx = (int)(seek_sec * 25.0);
    meta.getStabilizationQuat(frame_idx, 1, gyro);

    bool ok = stitcher.stitchFrames(
        frameMaster.data(),
        frameSlave.data(),
        w1,
        h1,
        meta.lens_rear,  // Master (Lens 2)
        meta.lens_front, // Slave (Lens 1)
        gyro,
        out_w,
        out_h,
        out_equirect.data()
    );

    if (ok) {
        std::cout << "[Vulkan] Stitching completed successfully!" << std::endl;
        std::string out_ppm = "stitched_vulkan_preview.ppm";
        if (!custom_out_ppm.empty()) {
            out_ppm = custom_out_ppm;
        } else if (argc > 3) {
            out_ppm = argv[3];
        }
        writePPM(out_ppm, out_w, out_h, out_equirect.data());
    } else {
        std::cerr << "[Vulkan] Stitching failed!" << std::endl;
        return 1;
    }

    return 0;
}
