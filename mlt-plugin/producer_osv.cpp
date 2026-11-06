#include "osv_metadata.h"
#include "osv_vulkan.h"

extern "C" {
#include <framework/mlt.h>
}

#include <iostream>
#include <memory>
#include <cstring>

struct ProducerOsvContext {
    mlt_producer producer_front = nullptr;
    mlt_producer producer_rear = nullptr;
    mlt_producer producer_audio = nullptr;
    OsvMetadata metadata;
    std::unique_ptr<OsvVulkanStitcher> stitcher;
    bool stitcher_initialized = false;
};

static int producer_get_image(mlt_frame frame, uint8_t** image, mlt_image_format* format, int* width, int* height, int writable) {
    mlt_producer producer = (mlt_producer)mlt_frame_pop_service(frame);
    if (!producer || !producer->child) return 1;
    ProducerOsvContext* ctx = (ProducerOsvContext*)producer->child;

    mlt_properties frame_props = mlt_frame_properties(frame);
    mlt_position position = mlt_frame_get_position(frame);
    std::cout << "[OSV-MLT] producer_get_image pos=" << position << " requested=" << *width << "x" << *height << std::endl;

    if (!ctx->producer_front || !ctx->producer_rear) return 1;

    // Seek sub-producers to frame position
    mlt_producer_seek(ctx->producer_front, position);
    mlt_producer_seek(ctx->producer_rear, position);

    mlt_frame frame_front = nullptr;
    mlt_frame frame_rear = nullptr;

    mlt_service_get_frame(MLT_PRODUCER_SERVICE(ctx->producer_front), &frame_front, 0);
    mlt_service_get_frame(MLT_PRODUCER_SERVICE(ctx->producer_rear), &frame_rear, 0);

    if (!frame_front || !frame_rear) {
        if (frame_front) mlt_frame_close(frame_front);
        if (frame_rear) mlt_frame_close(frame_rear);
        return 1;
    }

    uint8_t* img_front = nullptr;
    uint8_t* img_rear = nullptr;
    int w1 = 3840, h1 = 3840, w2 = 3840, h2 = 3840;
    mlt_image_format fmt = mlt_image_rgba;

    mlt_frame_get_image(frame_front, &img_front, &fmt, &w1, &h1, 0);
    mlt_frame_get_image(frame_rear, &img_rear, &fmt, &w2, &h2, 0);

    std::cout << "[OSV-MLT] Got subframes: img_front=" << (void*)img_front << " (" << w1 << "x" << h1 << ") img_rear=" << (void*)img_rear << " (" << w2 << "x" << h2 << ")" << std::endl;

    int out_w = *width > 0 ? *width : 3840;
    int out_h = *height > 0 ? *height : 1920;
    *width = out_w;
    *height = out_h;
    *format = mlt_image_rgba;

    int buf_size = out_w * out_h * 4;
    *image = (uint8_t*)mlt_pool_alloc(buf_size);

    if (img_front && img_rear && ctx->stitcher_initialized) {
        float default_gyro[4] = {0.0f, 0.0f, 0.0f, 1.0f};
        ctx->stitcher->stitchFrames(
            img_front, img_rear,
            w1, h1,
            ctx->metadata.lens_front, ctx->metadata.lens_rear,
            default_gyro,
            out_w, out_h,
            *image
        );
    } else {
        std::memset(*image, 0, buf_size);
    }

    mlt_frame_close(frame_front);
    mlt_frame_close(frame_rear);

    // Register destructor for image memory
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
    ProducerOsvContext* ctx = (ProducerOsvContext*)producer->child;
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
        if (ctx->producer_front) mlt_producer_close(ctx->producer_front);
        if (ctx->producer_rear) mlt_producer_close(ctx->producer_rear);
        if (ctx->producer_audio) mlt_producer_close(ctx->producer_audio);
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

    // Parse OSV metadata
    if (filename && OsvMetadataParser::parseFromFile(filename, ctx->metadata)) {
        mlt_properties_set(props, "meta.camera.model", ctx->metadata.camera_model.c_str());
        mlt_properties_set(props, "meta.camera.serial", ctx->metadata.serial_number.c_str());
    }

    // Instantiate sub-producers using avformat
    ctx->producer_front = mlt_factory_producer(profile, "avformat", filename);
    if (ctx->producer_front) {
        mlt_properties_set_int(MLT_PRODUCER_PROPERTIES(ctx->producer_front), "video_index", 0);
    }

    ctx->producer_rear = mlt_factory_producer(profile, "avformat", filename);
    if (ctx->producer_rear) {
        mlt_properties_set_int(MLT_PRODUCER_PROPERTIES(ctx->producer_rear), "video_index", 1);
    }

    ctx->producer_audio = mlt_factory_producer(profile, "avformat", filename);
    if (ctx->producer_audio) {
        mlt_properties_set_int(MLT_PRODUCER_PROPERTIES(ctx->producer_audio), "audio_index", 2);
    }

    // Set duration from primary stream
    if (ctx->producer_front) {
        mlt_properties_set_position(props, "length", mlt_producer_get_length(ctx->producer_front));
    }

    // Initialize Vulkan compute stitcher
    ctx->stitcher = std::make_unique<OsvVulkanStitcher>();
    std::string shader_path = "/home/fra/personal/osmo-360/shaders/osv_stitch.spv";
    ctx->stitcher_initialized = ctx->stitcher->initialize(shader_path);

    return producer;
}
