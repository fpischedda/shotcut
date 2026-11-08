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

static bool decodeFrame(AVFormatContext* fmt_ctx, int stream_idx, int frame_target, std::vector<uint8_t>& out_rgba, int& width, int& height) {
    AVStream* stream = fmt_ctx->streams[stream_idx];
    const AVCodec* codec = avcodec_find_decoder(stream->codecpar->codec_id);
    if (!codec) return false;

    AVCodecContext* c_ctx = avcodec_alloc_context3(codec);
    avcodec_parameters_to_context(c_ctx, stream->codecpar);
    if (avcodec_open2(c_ctx, codec, nullptr) < 0) {
        avcodec_free_context(&c_ctx);
        return false;
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

    int cur_frame = 0;
    bool got_frame = false;

    while (av_read_frame(fmt_ctx, pkt) >= 0) {
        if (pkt->stream_index == stream_idx) {
            if (avcodec_send_packet(c_ctx, pkt) == 0) {
                while (avcodec_receive_frame(c_ctx, frame) == 0) {
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
    AVFormatContext* fmt_ctx = nullptr;
    if (avformat_open_input(&fmt_ctx, osv_path.c_str(), nullptr, nullptr) < 0) {
        std::cerr << "Failed to open video file" << std::endl;
        return 1;
    }
    avformat_find_stream_info(fmt_ctx, nullptr);

    std::vector<uint8_t> frameSlave, frameMaster;
    int w0 = 0, h0 = 0, w1 = 0, h1 = 0;

    std::cout << "[Decode] Decoding Slave Lens (Stream 0)..." << std::endl;
    decodeFrame(fmt_ctx, 0, 0, frameSlave, w0, h0);

    std::cout << "[Decode] Decoding Master Lens (Stream 1)..." << std::endl;
    decodeFrame(fmt_ctx, 1, 0, frameMaster, w1, h1);

    avformat_close_input(&fmt_ctx);

    std::cout << "[Decode] Stream 0: " << w0 << "x" << h0 << " | Stream 1: " << w1 << "x" << h1 << std::endl;
    if (w0 == 0 || w1 == 0) {
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

    // Step 4: Stitch to Equirectangular (3840x1920)
    uint32_t out_w = 3840;
    uint32_t out_h = 1920;
    std::vector<uint8_t> out_equirect(out_w * out_h * 4);

    std::cout << "[Vulkan] Executing GPU compute stitch (" << out_w << "x" << out_h << ")..." << std::endl;
    float default_gyro[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // Identity (w, x, y, z)

    bool ok = stitcher.stitchFrames(
        frameMaster.data(),
        frameSlave.data(),
        w1,
        h1,
        meta.lens_rear,  // Master (Lens 2)
        meta.lens_front, // Slave (Lens 1)
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
