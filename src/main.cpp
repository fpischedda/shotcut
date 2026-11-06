#include "osv_metadata.h"
#include "osv_vulkan.h"
#include <iostream>
#include <vector>
#include <fstream>
#include <cstring>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
}

// Simple PPM image writer (no external dependency needed)
void writePPM(const std::string& filename, int width, int height, const uint8_t* rgba) {
    std::ofstream out(filename, std::ios::binary);
    out << "P6\n" << width << " " << height << "\n255\n";
    for (int i = 0; i < width * height; ++i) {
        out.put(rgba[i * 4 + 0]); // R
        out.put(rgba[i * 4 + 1]); // G
        out.put(rgba[i * 4 + 2]); // B
    }
    out.close();
    std::cout << "[OSV] Saved stitched frame to: " << filename << std::endl;
}

bool decodeFrame(AVFormatContext* fmt_ctx, int stream_idx, int64_t target_ts_us, std::vector<uint8_t>& out_rgba, int& width, int& height) {
    AVCodecParameters* codecpar = fmt_ctx->streams[stream_idx]->codecpar;
    const AVCodec* codec = avcodec_find_decoder(codecpar->codec_id);
    if (!codec) return false;

    AVCodecContext* c_ctx = avcodec_alloc_context3(codec);
    avcodec_parameters_to_context(c_ctx, codecpar);
    if (avcodec_open2(c_ctx, codec, nullptr) < 0) {
        avcodec_free_context(&c_ctx);
        return false;
    }

    width = c_ctx->width;
    height = c_ctx->height;
    out_rgba.resize(width * height * 4);

    AVPacket* pkt = av_packet_alloc();
    AVFrame* frame = av_frame_alloc();
    AVFrame* frame_rgb = av_frame_alloc();

    av_image_alloc(frame_rgb->data, frame_rgb->linesize, width, height, AV_PIX_FMT_RGBA, 1);
    SwsContext* sws = sws_getContext(width, height, c_ctx->pix_fmt, width, height, AV_PIX_FMT_RGBA, SWS_BILINEAR, nullptr, nullptr, nullptr);

    bool got_frame = false;
    av_seek_frame(fmt_ctx, stream_idx, 0, AVSEEK_FLAG_BACKWARD);

    while (av_read_frame(fmt_ctx, pkt) >= 0) {
        if (pkt->stream_index == stream_idx) {
            if (avcodec_send_packet(c_ctx, pkt) == 0) {
                if (avcodec_receive_frame(c_ctx, frame) == 0) {
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
        av_packet_unref(pkt);
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
    if (argc > 1) {
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
    std::cout << "[Calibration] Lens 1 (Front):" << std::endl;
    std::cout << "  fx=" << meta.lens_front.fx << " fy=" << meta.lens_front.fy 
              << " cx=" << meta.lens_front.cx << " cy=" << meta.lens_front.cy << std::endl;
    std::cout << "  k=[" << meta.lens_front.k1 << ", " << meta.lens_front.k2 
              << ", " << meta.lens_front.k3 << ", " << meta.lens_front.k4 << "]" << std::endl;
    std::cout << "  rot=[" << meta.lens_front.quat[0] << ", " << meta.lens_front.quat[1] 
              << ", " << meta.lens_front.quat[2] << ", " << meta.lens_front.quat[3] << "]" << std::endl;

    std::cout << "[Calibration] Lens 2 (Rear):" << std::endl;
    std::cout << "  fx=" << meta.lens_rear.fx << " fy=" << meta.lens_rear.fy 
              << " cx=" << meta.lens_rear.cx << " cy=" << meta.lens_rear.cy << std::endl;
    std::cout << "  k=[" << meta.lens_rear.k1 << ", " << meta.lens_rear.k2 
              << ", " << meta.lens_rear.k3 << ", " << meta.lens_rear.k4 << "]" << std::endl;
    std::cout << "  rot=[" << meta.lens_rear.quat[0] << ", " << meta.lens_rear.quat[1] 
              << ", " << meta.lens_rear.quat[2] << ", " << meta.lens_rear.quat[3] << "]" << std::endl;

    // Step 2: Decode 1 frame from each lens stream
    AVFormatContext* fmt_ctx = nullptr;
    if (avformat_open_input(&fmt_ctx, osv_path.c_str(), nullptr, nullptr) < 0) {
        std::cerr << "Failed to open video file" << std::endl;
        return 1;
    }
    avformat_find_stream_info(fmt_ctx, nullptr);

    std::vector<uint8_t> frameFront, frameRear;
    int w1 = 0, h1 = 0, w2 = 0, h2 = 0;

    std::cout << "[Decode] Decoding Lens 1 (Stream 0)..." << std::endl;
    decodeFrame(fmt_ctx, 0, 0, frameFront, w1, h1);

    std::cout << "[Decode] Decoding Lens 2 (Stream 1)..." << std::endl;
    decodeFrame(fmt_ctx, 1, 0, frameRear, w2, h2);

    avformat_close_input(&fmt_ctx);

    std::cout << "[Decode] Lens 1: " << w1 << "x" << h1 << " | Lens 2: " << w2 << "x" << h2 << std::endl;
    if (w1 == 0 || w2 == 0) {
        std::cerr << "Error decoding frames!" << std::endl;
        return 1;
    }

    // Step 3: Initialize Vulkan Stitcher
    OsvVulkanStitcher stitcher;
    std::string shader_path = "/home/fra/personal/osmo-360/shaders/osv_stitch.spv";
    if (!stitcher.initialize(shader_path)) {
        std::cerr << "Failed to initialize Vulkan compute pipeline!" << std::endl;
        return 1;
    }

    // Step 4: Stitch to Equirectangular (e.g. 3840x1920 preview or full 7680x3840)
    uint32_t out_w = 3840;
    uint32_t out_h = 1920;
    std::vector<uint8_t> out_equirect(out_w * out_h * 4);

    std::cout << "[Vulkan] Executing GPU compute stitch (" << out_w << "x" << out_h << ")..." << std::endl;
    float default_gyro[4] = {0.0f, 0.0f, 0.0f, 1.0f};

    bool ok = stitcher.stitchFrames(
        frameFront.data(),
        frameRear.data(),
        w1,
        h1,
        meta.lens_front,
        meta.lens_rear,
        default_gyro,
        out_w,
        out_h,
        out_equirect.data()
    );

    if (ok) {
        std::cout << "[Vulkan] Stitching completed successfully!" << std::endl;
        std::string out_ppm = "/home/fra/personal/osmo-360/stitched_vulkan_preview.ppm";
        writePPM(out_ppm, out_w, out_h, out_equirect.data());
    } else {
        std::cerr << "[Vulkan] Stitching failed!" << std::endl;
        return 1;
    }

    return 0;
}
