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
    std::mutex audio_mutex;

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
    void* last_active_producer = nullptr;
    void* last_audio_producer = nullptr;

    static std::string resolveShaderPath() {
        const char* env_path = std::getenv("OSV_SHADER_PATH");
        if (env_path && std::filesystem::exists(env_path)) {
            return env_path;
        }
        const char* mlt_data = std::getenv("MLT_DATA");
        if (mlt_data) {
            std::filesystem::path p = std::filesystem::path(mlt_data).parent_path().parent_path() / "shaders" / "osv_stitch.spv";
            if (std::filesystem::exists(p)) return p.string();
        }
        try {
            std::filesystem::path exe_dir = std::filesystem::read_symlink("/proc/self/exe").parent_path();
            std::vector<std::filesystem::path> exe_candidates = {
                exe_dir / "shaders" / "osv_stitch.spv",
                exe_dir / ".." / "shaders" / "osv_stitch.spv"
            };
            for (const auto& p : exe_candidates) {
                if (std::filesystem::exists(p)) return p.string();
            }
        } catch (...) {}
        std::vector<std::string> candidates = {
            "shaders/osv_stitch.spv",
            "../shaders/osv_stitch.spv"
        };
        for (const auto& path : candidates) {
            if (std::filesystem::exists(path)) return path;
        }
        return "shaders/osv_stitch.spv";
    }

    bool ensureStitcher() {
        if (!stitcher) {
            stitcher = std::make_unique<OsvVulkanStitcher>();
            std::string shader_path = resolveShaderPath();
            stitcher_initialized = stitcher->initialize(shader_path);
        }
        return stitcher_initialized;
    }

    bool ensureFullProducers() {
        if (!producer_master) {
            producer_master = mlt_factory_producer(profile, "avformat", osv_path.c_str());
            if (producer_master) {
                mlt_properties mprops = MLT_PRODUCER_PROPERTIES(producer_master);
                mlt_properties_set_int(mprops, "video_index", 1);
                mlt_properties_set_int(mprops, "audio_index", -1);
                mlt_producer_probe(producer_master);
            }
        }
        if (!producer_slave) {
            producer_slave = mlt_factory_producer(profile, "avformat", osv_path.c_str());
            if (producer_slave) {
                mlt_properties sprops = MLT_PRODUCER_PROPERTIES(producer_slave);
                mlt_properties_set_int(sprops, "video_index", 0);
                mlt_properties_set_int(sprops, "audio_index", -1);
                mlt_producer_probe(producer_slave);
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

    mlt_position position = mlt_properties_get(MLT_FRAME_PROPERTIES(frame), "original_position")
        ? mlt_frame_original_position(frame)
        : mlt_frame_get_position(frame);

    std::lock_guard<std::mutex> lock(ctx->mutex);

    void* current_prod = mlt_properties_get_data(MLT_FRAME_PROPERTIES(frame), "_producer", NULL);
    if (!current_prod) current_prod = producer;
    bool cut_changed = (current_prod != ctx->last_active_producer);
    ctx->last_active_producer = current_prod;

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

    // Determine preview vs export mode and target stitch dimensions
    bool export_mode = is_render_export(producer);
    bool use_lrf = ctx->has_lrf && ctx->producer_lrf && !export_mode;

    int out_w = (*width > 0) ? *width : (export_mode ? 5760 : 2048);
    int out_h = (*height > 0) ? *height : (export_mode ? 2880 : 1024);

    if (export_mode && (*width <= 0 || *height <= 0)) {
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
    }

    // Instantaneous cache hit when user pans/rotates/zooms the 360 viewport on the same frame
    if (position == ctx->cached_position && export_mode == ctx->cached_export_mode &&
        stab_mode == ctx->cached_stab_mode &&
        ctx->cached_w == out_w && ctx->cached_h == out_h &&
        ctx->cached_equirect.size() == (size_t)out_w * out_h * 4) {
        *width = out_w;
        *height = out_h;
        *format = mlt_image_rgba;
        int buf_size = out_w * out_h * 4;
        *image = (uint8_t*)mlt_pool_alloc(buf_size);
        std::memcpy(*image, ctx->cached_equirect.data(), buf_size);
        mlt_frame_set_image(frame, *image, buf_size, mlt_pool_release);
        return 0;
    }

    // Scaled frame index for gyro telemetry lookup matching container fps
    double profile_fps = mlt_producer_get_fps(producer);
    double source_fps = ctx->metadata.fps > 0.0 ? ctx->metadata.fps : 25.0;
    int64_t source_frame = (profile_fps > 0.0)
        ? static_cast<int64_t>(position / profile_fps * source_fps + 0.5)
        : position;

    // Fast-path: Companion .LRF proxy decoding during interactive editor playback
    // The camera's companion .LRF is 2048x1024 progressive H.264 side-by-side dual fisheye.
    // Left half (1024x1024) is Slave lens (Stream 0), Right half (1024x1024) is Master lens (Stream 1).
    if (use_lrf) {
        if (!ctx->ensureStitcher()) {
            std::cerr << "[OSV-MLT] Failed to initialize Vulkan stitcher for LRF mode" << std::endl;
            return 1;
        }

        if (cut_changed || ctx->last_lrf_position < 0 || position != ctx->last_lrf_position + 1) {
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
                *width = out_w;
                *height = out_h;
                *format = mlt_image_rgba;
                int buf_size = out_w * out_h * 4;
                *image = (uint8_t*)mlt_pool_alloc(buf_size);

                float gyro[4] = {1.0f, 0.0f, 0.0f, 0.0f};
                ctx->metadata.getStabilizationQuat(source_frame, stab_mode, gyro);
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

    // Full Vulkan compute stitching path (used for final render / export or when no LRF is available)
    if (!ctx->ensureFullProducers()) {
        return 1;
    }

    // Avoid seeking on sequential playback within the same cut to prevent GOP decoder flush
    if (cut_changed || ctx->last_position < 0 || position != ctx->last_position + 1) {
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

    *width = out_w;
    *height = out_h;
    *format = mlt_image_rgba;

    int buf_size = out_w * out_h * 4;
    *image = (uint8_t*)mlt_pool_alloc(buf_size);

    if (img_master && img_slave && ctx->stitcher_initialized) {
        float gyro[4] = {1.0f, 0.0f, 0.0f, 0.0f};
        ctx->metadata.getStabilizationQuat(source_frame, stab_mode, gyro);
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

    mlt_position position = mlt_properties_get(MLT_FRAME_PROPERTIES(frame), "original_position")
        ? mlt_frame_original_position(frame)
        : mlt_frame_get_position(frame);

    std::lock_guard<std::mutex> lock(ctx->audio_mutex);

    void* current_prod = mlt_properties_get_data(MLT_FRAME_PROPERTIES(frame), "_producer", NULL);
    if (!current_prod) current_prod = producer;
    bool cut_changed = (current_prod != ctx->last_audio_producer);
    ctx->last_audio_producer = current_prod;

    if (cut_changed || ctx->last_audio_position < 0 || position != ctx->last_audio_position + 1) {
        mlt_producer_seek(ctx->producer_audio, position);
    }
    ctx->last_audio_position = position;

    mlt_frame audio_frame = nullptr;
    int err = mlt_service_get_frame(MLT_PRODUCER_SERVICE(ctx->producer_audio), &audio_frame, 0);
    if (err != 0 || !audio_frame) return 1;

    void* audio_buf = nullptr;
    int ret = mlt_frame_get_audio(audio_frame, &audio_buf, format, frequency, channels, samples);
    if (ret == 0 && audio_buf && *samples > 0) {
        int total_bytes = mlt_audio_format_size(*format, *samples, *channels);
        *buffer = mlt_pool_alloc(total_bytes);
        if (*buffer) {
            std::memcpy(*buffer, audio_buf, total_bytes);
            mlt_frame_set_audio(frame, *buffer, *format, total_bytes, mlt_pool_release);
            const char* layout = mlt_properties_get(MLT_FRAME_PROPERTIES(audio_frame), "channel_layout");
            mlt_properties_set(MLT_FRAME_PROPERTIES(frame), "channel_layout", layout ? layout : "stereo");
        } else {
            ret = 1;
        }
    } else {
        ret = 1;
    }
    mlt_frame_close(audio_frame);
    return ret;
}

static int producer_get_frame(mlt_producer producer, mlt_frame_ptr frame, int index) {
    *frame = mlt_frame_init(MLT_PRODUCER_SERVICE(producer));
    if (*frame) {
        mlt_properties frame_props = MLT_FRAME_PROPERTIES(*frame);
        mlt_position p_frame = mlt_producer_frame(producer);
        mlt_position p_pos = mlt_producer_position(producer);
        mlt_properties_set_position(frame_props, "original_position", p_frame);
        mlt_frame_set_position(*frame, p_pos);
        mlt_properties_set_data(frame_props, "_producer", MLT_PRODUCER_SERVICE(producer), 0, NULL, NULL);

        // Populate geometry properties on frame so downstream filters receive correct format
        mlt_properties_set_int(frame_props, "width", 3840);
        mlt_properties_set_int(frame_props, "height", 1920);
        mlt_properties_set_double(frame_props, "aspect_ratio", 1.0);
        mlt_properties_set_int(frame_props, "progressive", 1);

        mlt_frame_push_service(*frame, producer);
        mlt_frame_push_get_image(*frame, producer_get_image);
        mlt_frame_push_audio(*frame, producer);
        mlt_frame_push_audio(*frame, (void*)producer_get_audio);
        mlt_producer_prepare_next(producer);
        return 0;
    }
    return 1;
}

static void producer_close(mlt_producer producer) {
    if (producer) {
        if (producer->child) {
            ProducerOsvContext* ctx = (ProducerOsvContext*)producer->child;
            std::lock_guard<std::mutex> lock(ctx->mutex);
            std::lock_guard<std::mutex> audio_lock(ctx->audio_mutex);
            producer->child = nullptr;
            if (ctx->producer_master) mlt_producer_close(ctx->producer_master);
            if (ctx->producer_slave) mlt_producer_close(ctx->producer_slave);
            if (ctx->producer_audio) mlt_producer_close(ctx->producer_audio);
            if (ctx->producer_lrf) mlt_producer_close(ctx->producer_lrf);
            delete ctx;
        }
        producer->close = nullptr;
        mlt_producer_close(producer);
    }
}

extern "C" mlt_producer producer_osv_init(mlt_profile profile, mlt_service_type type, const char* id, const char* filename) {
    mlt_producer producer = mlt_producer_new(profile);
    if (!producer) return nullptr;

    ProducerOsvContext* ctx = new ProducerOsvContext();
    producer->child = ctx;
    producer->get_frame = producer_get_frame;
    producer->close = (mlt_destructor)producer_close;
    producer->close_object = producer;

    mlt_properties props = MLT_PRODUCER_PROPERTIES(producer);
    mlt_properties_set(props, "resource", filename);
    mlt_properties_set(props, "mlt_service", "osv");
    mlt_properties_set(props, "shotcut:producer", "avformat");

    mlt_log_info(MLT_PRODUCER_SERVICE(producer), "[OSV-MLT] Loaded native OSV plugin (build 2026-11-10 direct-osv-v3)\n");
    std::cout << "[OSV-MLT] Loaded native OSV plugin (build 2026-11-10 direct-osv-v3)" << std::endl;

    ctx->profile = profile;
    ctx->osv_path = filename ? filename : "";

    // Parse OSV metadata, frame rate, and per-frame attitude telemetry
    if (filename && OsvMetadataParser::parseFromFile(filename, ctx->metadata)) {
        mlt_properties_set(props, "meta.camera.model", ctx->metadata.camera_model.c_str());
        mlt_properties_set(props, "meta.camera.serial", ctx->metadata.serial_number.c_str());
        mlt_properties_set_int(props, "meta.telemetry.samples", ctx->metadata.telemetry_frames.size());
    }

    double fps = ctx->metadata.fps > 0.0 ? ctx->metadata.fps : 29.97002997002997;
    int fps_num = ctx->metadata.fps_num > 0 ? ctx->metadata.fps_num : 30000;
    int fps_den = ctx->metadata.fps_den > 0 ? ctx->metadata.fps_den : 1001;

    // Critical seekability and stream properties for Shotcut UI & Filter dock
    mlt_properties_set_int(props, "seekable", 1);
    mlt_properties_set_int(props, "force_seekable", 1);
    mlt_properties_set_int(props, "video_index", 0);
    mlt_properties_set_int(props, "vstream", 0);
    mlt_properties_set_int(props, "audio_index", 1);
    mlt_properties_set_int(props, "astream", 0);
    mlt_properties_set_int(props, "mute_on_pause", 0);

    // Inject Equirectangular aspect ratio (SAR = 1.0 for square pixels) and video properties
    mlt_properties_set_int(props, "width", 3840);
    mlt_properties_set_int(props, "height", 1920);
    mlt_properties_set_double(props, "aspect_ratio", 1.0);
    mlt_properties_set_int(props, "progressive", 1);
    mlt_properties_set_double(props, "fps", fps);

    mlt_properties_set_int(props, "meta.media.width", 3840);
    mlt_properties_set_int(props, "meta.media.height", 1920);
    mlt_properties_set_double(props, "meta.media.aspect_ratio", 1.0);
    mlt_properties_set_int(props, "meta.media.sample_aspect_num", 1);
    mlt_properties_set_int(props, "meta.media.sample_aspect_den", 1);
    mlt_properties_set_int(props, "meta.media.progressive", 1);
    mlt_properties_set_double(props, "meta.media.frame_rate_num", fps_num);
    mlt_properties_set_double(props, "meta.media.frame_rate_den", fps_den);
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
    mlt_properties_set_double(props, "meta.media.0.codec.aspect_ratio", 1.0);
    mlt_properties_set_double(props, "meta.media.0.codec.frame_rate", fps);
    mlt_properties_set_double(props, "meta.media.0.stream.frame_rate", fps);
    mlt_properties_set(props, "meta.media.0.codec.pix_fmt", "yuv420p");
    mlt_properties_set_int(props, "meta.media.0.codec.colorspace", 709);

    // Stream 1: Audio
    mlt_properties_set(props, "meta.media.1.stream.type", "audio");
    mlt_properties_set(props, "meta.media.1.codec.name", "aac");
    mlt_properties_set(props, "meta.media.1.codec.long_name", "AAC (Advanced Audio Coding)");
    mlt_properties_set_int(props, "meta.media.1.codec.channels", 2);
    mlt_properties_set_int(props, "meta.media.1.codec.sample_rate", 48000);
    mlt_properties_set(props, "meta.media.1.codec.sample_fmt", "fltp");

    // Default stabilization: 1 = Horizon Lock (0 = Off, 1 = Horizon Lock, 2 = Full Lock)
    mlt_properties_set_int(props, "osv.stabilization", 1);
    mlt_properties_set(props, "meta.stabilization.mode", "Horizon Lock (1)");

    // Direct native OSV dual-HEVC hardware decoding.
    // LRF proxy is bypassed by default to avoid VAAPI decoder deadlocks, audio/video stream sync drift,
    // and timeline preview freezing during interactive scrubbing and trimming.
    // Set OSV_USE_LRF=1 in environment to opt-in to companion LRF proxy mode.
    bool enable_lrf = (std::getenv("OSV_USE_LRF") != nullptr && std::string(std::getenv("OSV_USE_LRF")) == "1");
    if (enable_lrf && filename) {
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
        if (ctx->producer_lrf) {
            mlt_properties_set_int(MLT_PRODUCER_PROPERTIES(ctx->producer_lrf), "audio_index", -1);
            mlt_producer_probe(ctx->producer_lrf);
        }
        mlt_properties_set(props, "meta.lrf_path", ctx->lrf_path.c_str());
        std::cout << "[OSV-MLT] Detected companion LRF proxy for editing: " << ctx->lrf_path << std::endl;
        mlt_log_info(MLT_PRODUCER_SERVICE(producer), "[OSV-MLT] Using companion LRF proxy: %s\n", ctx->lrf_path.c_str());
    } else {
        std::cout << "[OSV-MLT] Direct native OSV dual-stream mode active (LRF proxy bypassed)" << std::endl;
        mlt_log_info(MLT_PRODUCER_SERVICE(producer), "[OSV-MLT] Direct native OSV dual-stream mode active\n");
    }

    // Dedicated audio producer (prefer lightweight LRF container if available, otherwise OSV)
    const char* audio_source = (ctx->has_lrf && !ctx->lrf_path.empty()) ? ctx->lrf_path.c_str() : filename;
    int audio_stream_idx = (ctx->has_lrf && !ctx->lrf_path.empty()) ? 1 : 2;

    ctx->producer_audio = mlt_factory_producer(profile, "avformat", audio_source);
    if (ctx->producer_audio) {
        mlt_properties aprops = MLT_PRODUCER_PROPERTIES(ctx->producer_audio);
        mlt_properties_set_int(aprops, "video_index", -1);
        mlt_properties_set_int(aprops, "audio_index", audio_stream_idx);
        mlt_properties_set_int(aprops, "astream", 0);
        mlt_producer_probe(ctx->producer_audio);
    }

    // Set duration and timeline in/out bounds (from LRF if available, otherwise from audio/master stream)
    mlt_position length = 0;
    if (ctx->producer_lrf) {
        length = mlt_producer_get_length(ctx->producer_lrf);
    } else if (ctx->producer_audio) {
        length = mlt_producer_get_length(ctx->producer_audio);
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
