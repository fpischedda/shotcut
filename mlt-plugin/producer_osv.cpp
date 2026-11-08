#include "osv_metadata.h"
#include "osv_vulkan.h"

extern "C" {
#include <framework/mlt.h>
}

#include <iostream>
#include <memory>
#include <cstring>
#include <string>
#include <filesystem>

struct ProducerOsvContext {
    mlt_producer producer_slave = nullptr;  // Stream 0: Slave lens (Rear)
    mlt_producer producer_master = nullptr; // Stream 1: Master lens (Front)
    mlt_producer producer_audio = nullptr;  // Stream 2: Audio
    mlt_producer producer_lrf = nullptr;    // Companion .LRF pre-stitched proxy

    OsvMetadata metadata;
    std::unique_ptr<OsvVulkanStitcher> stitcher;
    bool stitcher_initialized = false;
    bool has_lrf = false;
    std::string lrf_path;

    mlt_position last_position = -1;
    mlt_position last_lrf_position = -1;
};

static int producer_get_image(mlt_frame frame, uint8_t** image, mlt_image_format* format, int* width, int* height, int writable) {
    mlt_producer producer = (mlt_producer)mlt_frame_pop_service(frame);
    if (!producer || !producer->child) return 1;
    ProducerOsvContext* ctx = (ProducerOsvContext*)producer->child;

    mlt_position position = mlt_frame_get_position(frame);

    // Fast-path: Companion .LRF proxy decoding for preview scaling or scrubbing
    // The camera's companion .LRF is 2048x1024 progressive H.264 pre-stitched equirectangular.
    int req_w = *width;
    int req_h = *height;
    bool use_lrf = ctx->has_lrf && ctx->producer_lrf && (req_w <= 2048 && req_h <= 1024);

    if (use_lrf) {
        if (position != ctx->last_lrf_position + 1) {
            mlt_producer_seek(ctx->producer_lrf, position);
        }
        ctx->last_lrf_position = position;

        mlt_frame lrf_frame = nullptr;
        mlt_service_get_frame(MLT_PRODUCER_SERVICE(ctx->producer_lrf), &lrf_frame, 0);
        if (lrf_frame) {
            mlt_image_format fmt = mlt_image_rgba;
            uint8_t* lrf_img = nullptr;
            int lw = req_w > 0 ? req_w : 2048;
            int lh = req_h > 0 ? req_h : 1024;
            if (mlt_frame_get_image(lrf_frame, &lrf_img, &fmt, &lw, &lh, 0) == 0 && lrf_img) {
                *width = lw;
                *height = lh;
                *format = mlt_image_rgba;
                int buf_size = lw * lh * 4;
                *image = (uint8_t*)mlt_pool_alloc(buf_size);
                std::memcpy(*image, lrf_img, buf_size);
                mlt_frame_close(lrf_frame);
                mlt_frame_set_image(frame, *image, buf_size, mlt_pool_release);
                return 0;
            }
            mlt_frame_close(lrf_frame);
        }
    }

    if (!ctx->producer_master || !ctx->producer_slave) return 1;

    // Avoid seeking on sequential playback to prevent GOP decoder flush
    if (position != ctx->last_position + 1) {
        mlt_producer_seek(ctx->producer_master, position);
        mlt_producer_seek(ctx->producer_slave, position);
    }
    ctx->last_position = position;

    mlt_frame frame_master = nullptr;
    mlt_frame frame_slave = nullptr;

    mlt_service_get_frame(MLT_PRODUCER_SERVICE(ctx->producer_master), &frame_master, 0);
    mlt_service_get_frame(MLT_PRODUCER_SERVICE(ctx->producer_slave), &frame_slave, 0);

    if (!frame_master || !frame_slave) {
        if (frame_master) mlt_frame_close(frame_master);
        if (frame_slave) mlt_frame_close(frame_slave);
        return 1;
    }

    uint8_t* img_master = nullptr;
    uint8_t* img_slave = nullptr;
    int w1 = 3840, h1 = 3840, w0 = 3840, h0 = 3840;
    mlt_image_format fmt = mlt_image_rgba;

    mlt_frame_get_image(frame_master, &img_master, &fmt, &w1, &h1, 0);
    mlt_frame_get_image(frame_slave, &img_slave, &fmt, &w0, &h0, 0);

    int out_w = *width > 0 ? *width : 3840;
    int out_h = *height > 0 ? *height : 1920;
    *width = out_w;
    *height = out_h;
    *format = mlt_image_rgba;

    int buf_size = out_w * out_h * 4;
    *image = (uint8_t*)mlt_pool_alloc(buf_size);

    if (img_master && img_slave && ctx->stitcher_initialized) {
        float default_gyro[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // Identity (w, x, y, z)
        ctx->stitcher->stitchFrames(
            img_master, img_slave,
            w1, h1,
            ctx->metadata.lens_rear,  // Master (Lens 2)
            ctx->metadata.lens_front, // Slave (Lens 1)
            default_gyro,
            out_w, out_h,
            *image
        );
    } else {
        std::memset(*image, 0, buf_size);
    }

    mlt_frame_close(frame_master);
    mlt_frame_close(frame_slave);

    mlt_frame_set_image(frame, *image, buf_size, mlt_pool_release);
    return 0;
}

static int producer_get_audio(mlt_frame frame, void** buffer, mlt_audio_format* format, int* frequency, int* channels, int* samples) {
    mlt_producer producer = (mlt_producer)mlt_frame_pop_audio(frame);
    if (!producer || !producer->child) return 1;
    ProducerOsvContext* ctx = (ProducerOsvContext*)producer->child;
    if (!ctx->producer_audio) return 1;

    mlt_position position = mlt_frame_get_position(frame);
    mlt_producer_seek(ctx->producer_audio, position);

    mlt_frame audio_frame = nullptr;
    mlt_service_get_frame(MLT_PRODUCER_SERVICE(ctx->producer_audio), &audio_frame, 0);
    if (!audio_frame) return 1;

    int ret = mlt_frame_get_audio(audio_frame, buffer, format, frequency, channels, samples);
    mlt_frame_close(audio_frame);
    return ret;
}

static int producer_get_frame(mlt_producer producer, mlt_frame_ptr frame, int index) {
    *frame = mlt_frame_init(MLT_PRODUCER_SERVICE(producer));
    if (*frame) {
        mlt_frame_set_position(*frame, mlt_producer_position(producer));
        mlt_frame_push_service(*frame, producer);
        mlt_frame_push_get_image(*frame, producer_get_image);
        mlt_frame_push_audio(*frame, producer);
        mlt_frame_push_audio(*frame, (void*)producer_get_audio);
        return 0;
    }
    return 1;
}

static void producer_close(mlt_producer producer) {
    if (producer && producer->child) {
        ProducerOsvContext* ctx = (ProducerOsvContext*)producer->child;
        if (ctx->producer_master) mlt_producer_close(ctx->producer_master);
        if (ctx->producer_slave) mlt_producer_close(ctx->producer_slave);
        if (ctx->producer_audio) mlt_producer_close(ctx->producer_audio);
        if (ctx->producer_lrf) mlt_producer_close(ctx->producer_lrf);
        delete ctx;
        producer->child = nullptr;
    }
    producer->close = nullptr;
    mlt_producer_close(producer);
}

extern "C" mlt_producer producer_osv_init(mlt_profile profile, mlt_service_type type, const char* id, const char* filename) {
    mlt_producer producer = mlt_producer_new(profile);
    if (!producer) return nullptr;

    ProducerOsvContext* ctx = new ProducerOsvContext();
    producer->child = ctx;
    producer->get_frame = producer_get_frame;
    producer->close = (mlt_destructor)producer_close;

    mlt_properties props = MLT_PRODUCER_PROPERTIES(producer);
    mlt_properties_set(props, "resource", filename);
    mlt_properties_set(props, "mlt_service", "osv");
    mlt_properties_set(props, "shotcut:producer", "avformat");

    // Critical seekability and stream properties for Shotcut UI & Filter dock
    mlt_properties_set_int(props, "seekable", 1);
    mlt_properties_set_int(props, "force_seekable", 1);
    mlt_properties_set_int(props, "video_index", 0);
    mlt_properties_set_int(props, "audio_index", 0);
    mlt_properties_set_int(props, "mute_on_pause", 0);

    // Inject Equirectangular 2:1 aspect ratio and video properties
    mlt_properties_set_int(props, "width", 3840);
    mlt_properties_set_int(props, "height", 1920);
    mlt_properties_set_double(props, "aspect_ratio", 2.0);
    mlt_properties_set_int(props, "progressive", 1);
    mlt_properties_set_double(props, "fps", 25.0);

    mlt_properties_set_int(props, "meta.media.width", 3840);
    mlt_properties_set_int(props, "meta.media.height", 1920);
    mlt_properties_set_double(props, "meta.media.aspect_ratio", 2.0);
    mlt_properties_set_int(props, "meta.media.sample_aspect_num", 1);
    mlt_properties_set_int(props, "meta.media.sample_aspect_den", 1);
    mlt_properties_set_int(props, "meta.media.progressive", 1);
    mlt_properties_set_double(props, "meta.media.frame_rate_num", 25.0);
    mlt_properties_set_double(props, "meta.media.frame_rate_den", 1.0);
    mlt_properties_set_int(props, "meta.media.0.codec.rotate", 0);
    mlt_properties_set_int(props, "rotate", 0);

    // Stream metadata structure for Shotcut Properties dock and MetadataModel
    mlt_properties_set_int(props, "meta.media.nb_streams", 2);

    // Stream 0: Equirectangular Video (Stitched dual fisheye)
    mlt_properties_set(props, "meta.media.0.stream.type", "video");
    mlt_properties_set(props, "meta.media.0.codec.name", "hevc");
    mlt_properties_set(props, "meta.media.0.codec.long_name", "DJI Osmo 360 Dual-Fisheye (Vulkan Stitched)");
    mlt_properties_set_int(props, "meta.media.0.codec.width", 3840);
    mlt_properties_set_int(props, "meta.media.0.codec.height", 1920);
    mlt_properties_set_double(props, "meta.media.0.codec.aspect_ratio", 2.0);
    mlt_properties_set_double(props, "meta.media.0.codec.frame_rate", 25.0);
    mlt_properties_set(props, "meta.media.0.codec.pix_fmt", "yuv420p");
    mlt_properties_set_int(props, "meta.media.0.codec.colorspace", 709);

    // Stream 1: Audio
    mlt_properties_set(props, "meta.media.1.stream.type", "audio");
    mlt_properties_set(props, "meta.media.1.codec.name", "aac");
    mlt_properties_set(props, "meta.media.1.codec.long_name", "AAC (Advanced Audio Coding)");
    mlt_properties_set_int(props, "meta.media.1.codec.channels", 2);
    mlt_properties_set(props, "meta.media.1.codec.sample_rate", "48000");
    mlt_properties_set(props, "meta.media.1.codec.sample_fmt", "fltp");

    // Parse OSV metadata
    if (filename && OsvMetadataParser::parseFromFile(filename, ctx->metadata)) {
        mlt_properties_set(props, "meta.camera.model", ctx->metadata.camera_model.c_str());
        mlt_properties_set(props, "meta.camera.serial", ctx->metadata.serial_number.c_str());
    }

    // Check for companion .LRF proxy file (e.g. CAM_..._D.LRF)
    if (filename) {
        std::string path_str(filename);
        size_t dot_pos = path_str.find_last_of('.');
        if (dot_pos != std::string::npos) {
            std::string ext = path_str.substr(dot_pos);
            std::string base = path_str.substr(0, dot_pos);
            std::string cand1 = base + (ext == ".OSV" ? ".LRF" : ".lrf");
            std::string cand2 = base + (ext == ".osv" ? ".lrf" : ".LRF");
            if (std::filesystem::exists(cand1)) {
                ctx->lrf_path = cand1;
                ctx->has_lrf = true;
            } else if (std::filesystem::exists(cand2)) {
                ctx->lrf_path = cand2;
                ctx->has_lrf = true;
            }
        }
    }

    if (ctx->has_lrf) {
        ctx->producer_lrf = mlt_factory_producer(profile, "avformat", ctx->lrf_path.c_str());
        mlt_properties_set(props, "meta.lrf_path", ctx->lrf_path.c_str());
        std::cout << "[OSV-MLT] Detected companion LRF proxy: " << ctx->lrf_path << std::endl;
    }

    // Stream 0: Slave lens (Rear)
    ctx->producer_slave = mlt_factory_producer(profile, "avformat", filename);
    if (ctx->producer_slave) {
        mlt_properties_set_int(MLT_PRODUCER_PROPERTIES(ctx->producer_slave), "video_index", 0);
    }

    // Stream 1: Master lens (Front)
    ctx->producer_master = mlt_factory_producer(profile, "avformat", filename);
    if (ctx->producer_master) {
        mlt_properties_set_int(MLT_PRODUCER_PROPERTIES(ctx->producer_master), "video_index", 1);
    }

    // Stream 2: Audio
    ctx->producer_audio = mlt_factory_producer(profile, "avformat", filename);
    if (ctx->producer_audio) {
        mlt_properties_set_int(MLT_PRODUCER_PROPERTIES(ctx->producer_audio), "audio_index", 0);
        mlt_properties_set_int(MLT_PRODUCER_PROPERTIES(ctx->producer_audio), "astream", 2);
    }

    // Set duration and timeline in/out bounds from primary stream
    mlt_position length = 0;
    if (ctx->producer_master) {
        length = mlt_producer_get_length(ctx->producer_master);
    } else if (ctx->producer_lrf) {
        length = mlt_producer_get_length(ctx->producer_lrf);
    }
    mlt_properties_set_position(props, "in", 0);
    mlt_properties_set_position(props, "out", length > 0 ? length - 1 : 0);
    mlt_properties_set_position(props, "length", length);

    // Initialize Vulkan compute stitcher
    ctx->stitcher = std::make_unique<OsvVulkanStitcher>();
    std::string shader_path = "/home/fra/personal/osmo-360/shaders/osv_stitch.spv";
    ctx->stitcher_initialized = ctx->stitcher->initialize(shader_path);

    // Optional auto-reframing filter attachment if requested via reframe=1 property
    if (mlt_properties_get_int(props, "reframe") == 1) {
        mlt_filter reframe_filter = mlt_factory_filter(profile, "frei0r.bigsh0t_eq_to_rect", nullptr);
        if (reframe_filter) {
            mlt_properties_set_double(MLT_FILTER_PROPERTIES(reframe_filter), "fov", 0.28); // ~90 degrees
            mlt_service_attach(MLT_PRODUCER_SERVICE(producer), reframe_filter);
        }
    }

    return producer;
}
