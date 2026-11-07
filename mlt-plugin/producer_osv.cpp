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
#include <mutex>

struct ProducerOsvContext {
    mlt_profile profile = nullptr;
    std::string osv_path;
    std::mutex mutex;

    mlt_producer producer_slave = nullptr;  // Stream 0: Slave lens (Rear)
    mlt_producer producer_master = nullptr; // Stream 1: Master lens (Front)
    mlt_producer producer_audio = nullptr;  // Stream 2: Audio
    mlt_producer producer_lrf = nullptr;    // Companion .LRF side-by-side dual-fisheye proxy

    OsvMetadata metadata;
    std::unique_ptr<OsvVulkanStitcher> stitcher;
    bool stitcher_initialized = false;
    bool has_lrf = false;
    std::string lrf_path;

    // Stitched frame cache to make interactive viewport pan/tilt/zoom at a paused position instantaneous
    mlt_position cached_position = -1;
    int cached_w = 0;
    int cached_h = 0;
    bool cached_export_mode = false;
    int cached_stab_mode = -1;
    std::vector<uint8_t> cached_equirect;

    mlt_position last_position = -1;
    mlt_position last_lrf_position = -1;
    mlt_position last_audio_position = -1;

    bool ensureStitcher() {
        if (!stitcher) {
            stitcher = std::make_unique<OsvVulkanStitcher>();
            std::string shader_path = "/home/fra/personal/osmo-360/shaders/osv_stitch.spv";
            stitcher_initialized = stitcher->initialize(shader_path);
        }
        return stitcher_initialized;
    }

    bool ensureFullProducers() {
        if (!producer_master) {
            producer_master = mlt_factory_producer(profile, "avformat", osv_path.c_str());
            if (producer_master) {
                mlt_properties_set_int(MLT_PRODUCER_PROPERTIES(producer_master), "video_index", 1);
            }
        }
        if (!producer_slave) {
            producer_slave = mlt_factory_producer(profile, "avformat", osv_path.c_str());
            if (producer_slave) {
                mlt_properties_set_int(MLT_PRODUCER_PROPERTIES(producer_slave), "video_index", 0);
            }
        }
        return producer_master && producer_slave && ensureStitcher();
    }
};

static bool is_render_export(mlt_producer producer) {
    mlt_properties props = MLT_PRODUCER_PROPERTIES(producer);

    // 1. Explicit property override on producer (osv_mode)
    const char* prop_mode = mlt_properties_get(props, "osv_mode");
    if (prop_mode) {
        if (std::strcmp(prop_mode, "full") == 0 || std::strcmp(prop_mode, "render") == 0 || std::strcmp(prop_mode, "export") == 0) return true;
        if (std::strcmp(prop_mode, "proxy") == 0 || std::strcmp(prop_mode, "lrf") == 0 || std::strcmp(prop_mode, "preview") == 0) return false;
    }

    // 2. Explicit environment variable override (OSV_MODE)
    const char* env_mode = std::getenv("OSV_MODE");
    if (env_mode) {
        if (std::strcmp(env_mode, "full") == 0 || std::strcmp(env_mode, "render") == 0 || std::strcmp(env_mode, "export") == 0) return true;
        if (std::strcmp(env_mode, "proxy") == 0 || std::strcmp(env_mode, "lrf") == 0 || std::strcmp(env_mode, "preview") == 0) return false;
    }

    // 3. Process detection:
    // When Shotcut exports, it spawns 'melt' / 'melt-7' to encode the timeline.
    // Inside the Shotcut GUI itself, it runs the interactive editor preview.
    try {
        std::string exe = std::filesystem::read_symlink("/proc/self/exe").filename().string();
        if (exe.find("melt") != std::string::npos) {
            return true;
        }
    } catch (...) {}

    // 4. Consumer service check if directly connected
    mlt_service cons = mlt_service_consumer(MLT_PRODUCER_SERVICE(producer));
    if (cons) {
        mlt_properties cp = MLT_SERVICE_PROPERTIES(cons);
        const char* svc = mlt_properties_get(cp, "mlt_service");
        if (svc && (std::strcmp(svc, "avformat") == 0 || std::strcmp(svc, "avformat-novalidate") == 0)) {
            return true;
        }
    }

    return false;
}

static int producer_get_image(mlt_frame frame, uint8_t** image, mlt_image_format* format, int* width, int* height, int writable) {
    mlt_producer producer = (mlt_producer)mlt_frame_pop_service(frame);
    if (!producer || !producer->child) return 1;
    ProducerOsvContext* ctx = (ProducerOsvContext*)producer->child;

    mlt_position position = mlt_frame_get_position(frame);

    std::lock_guard<std::mutex> lock(ctx->mutex);

    // Gyro stabilization mode: 0 = Off, 1 = Horizon Lock (default), 2 = Full Lock
    mlt_properties props = MLT_PRODUCER_PROPERTIES(producer);
    int stab_mode = 1;
    const char* stab_str = mlt_properties_get(props, "osv.stabilization");
    if (stab_str) {
        if (std::strcmp(stab_str, "0") == 0 || std::strcmp(stab_str, "off") == 0 || std::strcmp(stab_str, "Off") == 0) {
            stab_mode = 0;
        } else if (std::strcmp(stab_str, "2") == 0 || std::strcmp(stab_str, "full") == 0 || std::strcmp(stab_str, "Full") == 0) {
            stab_mode = 2;
        } else {
            stab_mode = 1;
        }
    }

    // Fast-path: Companion .LRF proxy decoding during interactive editor playback
    // The camera's companion .LRF is 2048x1024 progressive H.264 side-by-side dual fisheye.
    // Left half (1024x1024) is Slave lens (Stream 0), Right half (1024x1024) is Master lens (Stream 1).
    bool export_mode = is_render_export(producer);
    bool use_lrf = ctx->has_lrf && ctx->producer_lrf && !export_mode;

    // Instantaneous cache hit when user pans/rotates/zooms the 360 viewport on the same frame
    if (position == ctx->cached_position && export_mode == ctx->cached_export_mode &&
        stab_mode == ctx->cached_stab_mode &&
        ctx->cached_w > 0 && ctx->cached_h > 0 &&
        ctx->cached_equirect.size() == (size_t)ctx->cached_w * ctx->cached_h * 4) {
        *width = ctx->cached_w;
        *height = ctx->cached_h;
        *format = mlt_image_rgba;
        int buf_size = ctx->cached_w * ctx->cached_h * 4;
        *image = (uint8_t*)mlt_pool_alloc(buf_size);
        std::memcpy(*image, ctx->cached_equirect.data(), buf_size);
        mlt_frame_set_image(frame, *image, buf_size, mlt_pool_release);
        return 0;
    }

    if (use_lrf) {
        if (!ctx->ensureStitcher()) {
            std::cerr << "[OSV-MLT] Failed to initialize Vulkan stitcher for LRF mode" << std::endl;
            return 1;
        }

        if (position != ctx->last_lrf_position + 1) {
            mlt_producer_seek(ctx->producer_lrf, position);
        }
        ctx->last_lrf_position = position;

        mlt_frame lrf_frame = nullptr;
        mlt_service_get_frame(MLT_PRODUCER_SERVICE(ctx->producer_lrf), &lrf_frame, 0);
        if (lrf_frame) {
            mlt_image_format fmt = mlt_image_rgba;
            uint8_t* lrf_img = nullptr;
            int lw = 2048;
            int lh = 1024;
            if (mlt_frame_get_image(lrf_frame, &lrf_img, &fmt, &lw, &lh, 0) == 0 && lrf_img) {
                int out_w = 2048;
                int out_h = 1024;
                *width = out_w;
                *height = out_h;
                *format = mlt_image_rgba;
                int buf_size = out_w * out_h * 4;
                *image = (uint8_t*)mlt_pool_alloc(buf_size);

                float gyro[4] = {1.0f, 0.0f, 0.0f, 0.0f};
                ctx->metadata.getStabilizationQuat(position, stab_mode, gyro);
                bool ok = ctx->stitcher->stitchSideBySide(
                    lrf_img,
                    lw, lh,
                    ctx->metadata.lens_rear,  // Master (Stream 1)
                    ctx->metadata.lens_front, // Slave (Stream 0)
                    gyro,
                    out_w, out_h,
                    *image
                );

                mlt_frame_close(lrf_frame);

                if (ok) {
                    ctx->cached_position = position;
                    ctx->cached_w = out_w;
                    ctx->cached_h = out_h;
                    ctx->cached_export_mode = export_mode;
                    ctx->cached_stab_mode = stab_mode;
                    ctx->cached_equirect.assign(*image, *image + buf_size);

                    mlt_frame_set_image(frame, *image, buf_size, mlt_pool_release);
                    return 0;
                } else {
                    std::memset(*image, 0, buf_size);
                    mlt_frame_set_image(frame, *image, buf_size, mlt_pool_release);
                    return 0;
                }
            }
            mlt_frame_close(lrf_frame);
        }
    }

    // Full 4K Vulkan compute stitching path (used for final render / export or when no LRF is available)
    if (!ctx->ensureFullProducers()) {
        return 1;
    }

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

    // In export mode, the stitched equirectangular frame must maintain 2:1 aspect ratio.
    // If the caller requested an explicit 2:1 resolution (e.g. equirectangular export project profile),
    // respect caller's requested dimensions.
    // Otherwise (e.g. reframing to 16:9 1080p or 4K), default to 5.7K (5760x2880) so downstream reframing
    // filters (e.g. bigsh0t_eq_to_rect) sample from high-resolution optical sensor data.
    int out_w = 5760;
    int out_h = 2880;
    if (*width > 0 && *height > 0 && *width == 2 * (*height)) {
        out_w = *width;
        out_h = *height;
    }
    const char* export_res = mlt_properties_get(props, "osv.export_res");
    if (!export_res) export_res = std::getenv("OSV_EXPORT_RES");
    if (export_res) {
        if (std::strcmp(export_res, "4k") == 0 || std::strcmp(export_res, "3840") == 0) {
            out_w = 3840;
            out_h = 1920;
        } else if (std::strcmp(export_res, "5.7k") == 0 || std::strcmp(export_res, "5760") == 0) {
            out_w = 5760;
            out_h = 2880;
        } else if (std::strcmp(export_res, "8k") == 0 || std::strcmp(export_res, "7680") == 0) {
            out_w = 7680;
            out_h = 3840;
        }
    }
    *width = out_w;
    *height = out_h;
    *format = mlt_image_rgba;

    int buf_size = out_w * out_h * 4;
    *image = (uint8_t*)mlt_pool_alloc(buf_size);

    if (img_master && img_slave && ctx->stitcher_initialized) {
        float gyro[4] = {1.0f, 0.0f, 0.0f, 0.0f};
        ctx->metadata.getStabilizationQuat(position, stab_mode, gyro);
        if (ctx->stitcher->stitchFrames(
            img_master, img_slave,
            w1, h1,
            ctx->metadata.lens_rear,  // Master (Lens 2)
            ctx->metadata.lens_front, // Slave (Lens 1)
            gyro,
            out_w, out_h,
            *image
        )) {
            ctx->cached_position = position;
            ctx->cached_w = out_w;
            ctx->cached_h = out_h;
            ctx->cached_export_mode = export_mode;
            ctx->cached_stab_mode = stab_mode;
            ctx->cached_equirect.assign(*image, *image + buf_size);
        }
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
    if (position != ctx->last_audio_position + 1) {
        mlt_producer_seek(ctx->producer_audio, position);
    }
    ctx->last_audio_position = position;

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
        std::lock_guard<std::mutex> lock(ctx->mutex);
        producer->child = nullptr;
        if (ctx->producer_master) mlt_producer_close(ctx->producer_master);
        if (ctx->producer_slave) mlt_producer_close(ctx->producer_slave);
        if (ctx->producer_audio) mlt_producer_close(ctx->producer_audio);
        if (ctx->producer_lrf) mlt_producer_close(ctx->producer_lrf);
        delete ctx;
    }
    producer->close = nullptr;
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

    // Parse OSV metadata and per-frame attitude telemetry
    if (filename && OsvMetadataParser::parseFromFile(filename, ctx->metadata)) {
        mlt_properties_set(props, "meta.camera.model", ctx->metadata.camera_model.c_str());
        mlt_properties_set(props, "meta.camera.serial", ctx->metadata.serial_number.c_str());
        mlt_properties_set_int(props, "meta.telemetry.samples", ctx->metadata.telemetry_frames.size());
    }

    // Default stabilization: 1 = Horizon Lock (0 = Off, 1 = Horizon Lock, 2 = Full Lock)
    mlt_properties_set_int(props, "osv.stabilization", 1);
    mlt_properties_set(props, "meta.stabilization.mode", "Horizon Lock (1)");

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

    ctx->profile = profile;
    ctx->osv_path = filename ? filename : "";

    if (ctx->has_lrf) {
        ctx->producer_lrf = mlt_factory_producer(profile, "avformat", ctx->lrf_path.c_str());
        mlt_properties_set(props, "meta.lrf_path", ctx->lrf_path.c_str());
        std::cout << "[OSV-MLT] Detected companion LRF proxy for editing: " << ctx->lrf_path << std::endl;
    }

    // Stream 2: Dedicated audio producer
    ctx->producer_audio = mlt_factory_producer(profile, "avformat", filename);
    if (ctx->producer_audio) {
        mlt_properties_set_int(MLT_PRODUCER_PROPERTIES(ctx->producer_audio), "audio_index", 0);
        mlt_properties_set_int(MLT_PRODUCER_PROPERTIES(ctx->producer_audio), "astream", 2);
    }

    // Set duration and timeline in/out bounds (from LRF if available, otherwise from master stream)
    mlt_position length = 0;
    if (ctx->producer_lrf) {
        length = mlt_producer_get_length(ctx->producer_lrf);
    } else {
        ctx->ensureFullProducers();
        if (ctx->producer_master) {
            length = mlt_producer_get_length(ctx->producer_master);
        }
    }
    mlt_properties_set_position(props, "in", 0);
    mlt_properties_set_position(props, "out", length > 0 ? length - 1 : 0);
    mlt_properties_set_position(props, "length", length);

    // Optional auto-reframing filter attachment if requested via reframe=1 property
    if (mlt_properties_get_int(props, "reframe") == 1) {
        mlt_filter reframe_filter = mlt_factory_filter(profile, "frei0r.bigsh0t_eq_to_rect", nullptr);
        if (reframe_filter) {
            mlt_properties_set_double(MLT_FILTER_PROPERTIES(reframe_filter), "fov", 90.0); // 90 degrees
            mlt_service_attach(MLT_PRODUCER_SERVICE(producer), reframe_filter);
        }
    }

    return producer;
}
