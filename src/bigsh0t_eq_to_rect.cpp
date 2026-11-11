extern "C" {
#include <frei0r.h>
}
#include <chrono>
#include <cmath>
#include <cstdint>
#include <vector>
#include <cstring>
#include <algorithm>
#include <omp.h>
#include <cstdio>

static constexpr float PI = 3.14159265358979323846f;

struct Mat3f {
    float m[3][3];
};

static inline Mat3f mulMat3(const Mat3f& A, const Mat3f& B) {
    Mat3f C{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                C.m[i][j] += A.m[i][k] * B.m[k][j];
    return C;
}

static inline Mat3f buildRotation(float yaw_deg, float pitch_deg, float roll_deg) {
    float rx = roll_deg * (PI / 180.0f);
    float ry = pitch_deg * (PI / 180.0f);
    float rz = yaw_deg * (PI / 180.0f);
    Mat3f Rx = {{{1.0f, 0.0f, 0.0f}, {0.0f, std::cos(rx), -std::sin(rx)}, {0.0f, std::sin(rx), std::cos(rx)}}};
    Mat3f Ry = {{{std::cos(ry), 0.0f, std::sin(ry)}, {0.0f, 1.0f, 0.0f}, {-std::sin(ry), 0.0f, std::cos(ry)}}};
    Mat3f Rz = {{{std::cos(rz), -std::sin(rz), 0.0f}, {std::sin(rz), std::cos(rz), 0.0f}, {0.0f, 0.0f, 1.0f}}};
    // Bigsh0t convention: M = Rz(yaw) * Ry(pitch) * Rx(roll)
    return mulMat3(Rz, mulMat3(Ry, Rx));
}

static inline float fast_atan2f(float y, float x) {
    float abs_y = std::fabs(y) + 1e-10f;
    float r, angle;
    if (x < 0.0f) {
        r = (x + abs_y) / (abs_y - x);
        angle = 3.0f * (PI * 0.25f);
    } else {
        r = (x - abs_y) / (x + abs_y);
        angle = (PI * 0.25f);
    }
    angle += (0.1963f * r * r - 0.9817f) * r;
    return (y < 0.0f) ? -angle : angle;
}

struct CachedSample {
    int32_t offset;   // v0 * W + u0 (< 0 indicates black/out-of-bounds)
    uint16_t fx;      // 0..256
    uint16_t fy;      // 0..256
};

class EqToRectInstance {
public:
    uint32_t width = 0;
    uint32_t height = 0;

    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;
    double fov = 90.0;
    double fisheye = 0.0;
    double interpolation = 1.0;

    bool dirty = true;
    std::vector<CachedSample> sample_map;

    EqToRectInstance(uint32_t w, uint32_t h)
        : width(w), height(h) {
        sample_map.resize((size_t)w * h);
    }

    void updateMap() {
        if (!dirty) {
            return;
        }

        int W = static_cast<int>(width);
        int H = static_cast<int>(height);
        if (W < 2 || H < 2) return;

        float y_deg = static_cast<float>(yaw);
        float p_deg = static_cast<float>(pitch);
        float r_deg = static_cast<float>(roll);
        float fov_deg = static_cast<float>(fov);
        float fe = static_cast<float>(fisheye);
        if (fov_deg < 0.1f) {
            fov_deg = 90.0f;
        } else if (fov_deg > 179.0f && fe <= 1e-4f) {
            fov_deg = 179.0f;
        }

        Mat3f M = buildRotation(y_deg, p_deg, r_deg);
        float tan_half_fov = std::tan(0.5f * fov_deg * (PI / 180.0f));
        if (std::isnan(tan_half_fov) || tan_half_fov <= 1e-4f) {
            tan_half_fov = 1.0f;
        }
        float half_w = static_cast<float>(W >> 1);
        float half_h = static_cast<float>(H >> 1);
        float step_x = tan_half_fov / half_w;
        float top_y = -tan_half_fov * static_cast<float>(H) / static_cast<float>(W);
        float step_y = (-top_y) / half_h;

        float scale_u = half_w / PI;
        float scale_v = half_h / (0.5f * PI);
        float fW = static_cast<float>(W);
        float max_v = static_cast<float>(H - 1);

        if (fe <= 1e-4f) {
            // Fast Rectilinear projection pass (vectorized ray increments)
            #pragma omp parallel for schedule(static)
            for (int y = 0; y < H; ++y) {
                float vz = y * step_y + top_y;
                float wx0 = M.m[0][0] + M.m[0][2] * vz - M.m[0][1] * tan_half_fov;
                float wy0 = M.m[1][0] + M.m[1][2] * vz - M.m[1][1] * tan_half_fov;
                float wz0 = M.m[2][0] + M.m[2][2] * vz - M.m[2][1] * tan_half_fov;
                float dwx = M.m[0][1] * step_x;
                float dwy = M.m[1][1] * step_x;
                float dwz = M.m[2][1] * step_x;

                CachedSample* row = sample_map.data() + (size_t)y * W;
                #pragma omp simd
                for (int x = 0; x < W; ++x) {
                    float wx = wx0 + x * dwx;
                    float wy = wy0 + x * dwy;
                    float wz = wz0 + x * dwz;
                    float len_xy = std::sqrt(wx * wx + wy * wy);
                    float lon = fast_atan2f(wy, wx);
                    float lat = fast_atan2f(wz, (len_xy > 1e-6f) ? len_xy : 1e-6f);
                    float u = lon * scale_u + half_w;
                    float v = lat * scale_v + half_h;

                    if (std::isnan(u) || std::isnan(v)) {
                        row[x] = {-1, 0, 0};
                        continue;
                    }
                    while (u < 0.0f) u += fW;
                    while (u >= fW) u -= fW;
                    if (v < 0.0f) v = 0.0f;
                    else if (v > max_v) v = max_v;

                    int u0 = std::clamp(static_cast<int>(u), 0, W - 2);
                    int v0 = std::clamp(static_cast<int>(v), 0, H - 2);
                    int fx = static_cast<int>((u - u0) * 256.0f);
                    int fy = static_cast<int>((v - v0) * 256.0f);
                    row[x] = {v0 * W + u0, static_cast<uint16_t>(fx), static_cast<uint16_t>(fy)};
                }
            }
        } else {
            // Fisheye blended projection pass
            float diag = std::sqrt(static_cast<float>(W * W + H * H));
            float r_half_diag = 0.5f * diag;
            float fisheye_weight = fe / 100.0f;
            float rect_weight = 1.0f - fisheye_weight;
            float fov_rad_half = 0.5f * fov_deg * (PI / 180.0f);

            #pragma omp parallel for schedule(static)
            for (int y = 0; y < H; ++y) {
                float vz = y * step_y + top_y;
                float dy = static_cast<float>(y) - half_h;
                CachedSample* row = sample_map.data() + (size_t)y * W;

                for (int x = 0; x < W; ++x) {
                    float dx = static_cast<float>(x) - half_w;
                    float dist = std::sqrt(dx * dx + dy * dy);
                    float theta = fov_rad_half * dist / r_half_diag;
                    if (theta > PI) {
                        row[x] = {-1, 0, 0};
                        continue;
                    }
                    float phi = std::atan2(dy, dx);
                    float sin_theta = std::sin(theta);
                    float cos_theta = std::cos(theta);

                    float v_fish_x = cos_theta * fisheye_weight;
                    float v_fish_y = sin_theta * std::cos(phi) * fisheye_weight;
                    float v_fish_z = sin_theta * std::sin(phi) * fisheye_weight;

                    float vx = rect_weight + v_fish_x;
                    float vy = (x * step_x - tan_half_fov) * rect_weight + v_fish_y;
                    float vz_tot = vz * rect_weight + v_fish_z;

                    float wx = M.m[0][0] * vx + M.m[0][1] * vy + M.m[0][2] * vz_tot;
                    float wy = M.m[1][0] * vx + M.m[1][1] * vy + M.m[1][2] * vz_tot;
                    float wz = M.m[2][0] * vx + M.m[2][1] * vy + M.m[2][2] * vz_tot;

                    float len_xy = std::sqrt(wx * wx + wy * wy);
                    float lon = fast_atan2f(wy, wx);
                    float lat = fast_atan2f(wz, (len_xy > 1e-6f) ? len_xy : 1e-6f);
                    float u = lon * scale_u + half_w;
                    float v = lat * scale_v + half_h;

                    if (std::isnan(u) || std::isnan(v)) {
                        row[x] = {-1, 0, 0};
                        continue;
                    }
                    while (u < 0.0f) u += fW;
                    while (u >= fW) u -= fW;
                    if (v < 0.0f) v = 0.0f;
                    else if (v > max_v) v = max_v;

                    int u0 = std::clamp(static_cast<int>(u), 0, W - 2);
                    int v0 = std::clamp(static_cast<int>(v), 0, H - 2);
                    int fx = static_cast<int>((u - u0) * 256.0f);
                    int fy = static_cast<int>((v - v0) * 256.0f);
                    row[x] = {v0 * W + u0, static_cast<uint16_t>(fx), static_cast<uint16_t>(fy)};
                }
            }
        }

        dirty = false;
    }

    void process(const uint32_t* __restrict__ inframe, uint32_t* __restrict__ outframe) {
        if (!inframe || !outframe || width < 2 || height < 2) return;
        updateMap();

        int total = static_cast<int>(width * height);
        const CachedSample* __restrict__ m = sample_map.data();

        if (interpolation < 0.5) {
            // Nearest neighbor interpolation
            #pragma omp parallel for schedule(static)
            for (int i = 0; i < total; ++i) {
                int32_t off = m[i].offset;
                outframe[i] = (off >= 0 && off < total) ? inframe[off] : 0xFF000000u;
            }
        } else {
            // High quality fast bilinear interpolation
            int stride = static_cast<int>(width);
            int max_idx = total - stride - 2;
            #pragma omp parallel for schedule(static)
            for (int i = 0; i < total; ++i) {
                int32_t off = m[i].offset;
                if (__builtin_expect(off < 0 || off > max_idx, 0)) {
                    outframe[i] = (off >= 0 && off < total) ? inframe[off] : 0xFF000000u;
                    continue;
                }
                uint32_t fx = m[i].fx;
                uint32_t fy = m[i].fy;
                const uint8_t* p00 = reinterpret_cast<const uint8_t*>(inframe + off);
                const uint8_t* p10 = p00 + 4;
                const uint8_t* p01 = p00 + stride * 4;
                const uint8_t* p11 = p01 + 4;

                uint32_t ifx = 256 - fx;
                uint32_t ify = 256 - fy;
                uint32_t w00 = ifx * ify;
                uint32_t w10 = fx * ify;
                uint32_t w01 = ifx * fy;
                uint32_t w11 = fx * fy;

                uint32_t r = (p00[0] * w00 + p10[0] * w10 + p01[0] * w01 + p11[0] * w11 + 32768) >> 16;
                uint32_t g = (p00[1] * w00 + p10[1] * w10 + p01[1] * w01 + p11[1] * w11 + 32768) >> 16;
                uint32_t b = (p00[2] * w00 + p10[2] * w10 + p01[2] * w01 + p11[2] * w11 + 32768) >> 16;
                uint32_t a = (p00[3] * w00 + p10[3] * w10 + p01[3] * w01 + p11[3] * w11 + 32768) >> 16;

                outframe[i] = (a << 24) | (b << 16) | (g << 8) | r;
            }
        }
    }
};

extern "C" {

int f0r_init() {
    return 1;
}

void f0r_deinit() {
}

void f0r_get_plugin_info(f0r_plugin_info_t* info) {
    info->name = "bigsh0t_eq_to_rect";
    info->author = "Leo Sutic <leo@sutic.nu> (Fast OpenMP SIMD build)";
    info->plugin_type = F0R_PLUGIN_TYPE_FILTER;
    info->color_model = F0R_COLOR_MODEL_PACKED32;
    info->frei0r_version = FREI0R_MAJOR_VERSION;
    info->major_version = 2;
    info->minor_version = 7;
    info->num_params = 6;
    info->explanation = "Extracts a rectilinear image from an equirectangular.";
}

void f0r_get_param_info(f0r_param_info_t* info, int param_index) {
    switch (param_index) {
        case 0:
            info->name = "yaw";
            info->type = F0R_PARAM_DOUBLE;
            info->explanation = "Yaw angle in degrees";
            break;
        case 1:
            info->name = "pitch";
            info->type = F0R_PARAM_DOUBLE;
            info->explanation = "Pitch angle in degrees";
            break;
        case 2:
            info->name = "roll";
            info->type = F0R_PARAM_DOUBLE;
            info->explanation = "Roll angle in degrees";
            break;
        case 3:
            info->name = "fov";
            info->type = F0R_PARAM_DOUBLE;
            info->explanation = "Field of view in degrees";
            break;
        case 4:
            info->name = "fisheye";
            info->type = F0R_PARAM_DOUBLE;
            info->explanation = "Fisheye blend percentage (0..100)";
            break;
        case 5:
            info->name = "interpolation";
            info->type = F0R_PARAM_DOUBLE;
            info->explanation = "0 = Nearest neighbor, 1 = Bilinear";
            break;
        default:
            break;
    }
}

f0r_instance_t f0r_construct(unsigned int width, unsigned int height) {
    return new EqToRectInstance(width, height);
}

void f0r_destruct(f0r_instance_t instance) {
    delete static_cast<EqToRectInstance*>(instance);
}

void f0r_set_param_value(f0r_instance_t instance, f0r_param_t param, int param_index) {
    if (!instance || !param) return;
    auto* inst = static_cast<EqToRectInstance*>(instance);
    double val = *static_cast<const double*>(param);
    switch (param_index) {
        case 0:
            if (std::isnan(val)) val = 0.0;
            if (inst->yaw != val) { inst->yaw = val; inst->dirty = true; }
            break;
        case 1:
            if (std::isnan(val)) val = 0.0;
            if (inst->pitch != val) { inst->pitch = val; inst->dirty = true; }
            break;
        case 2:
            if (std::isnan(val)) val = 0.0;
            if (inst->roll != val) { inst->roll = val; inst->dirty = true; }
            break;
        case 3:
            // Guard against UI reset / 0 fov: retain current fov or default to 90.0
            if (std::isnan(val) || val < 0.1) {
                val = (inst->fov >= 0.1) ? inst->fov : 90.0;
            }
            if (inst->fov != val) { inst->fov = val; inst->dirty = true; }
            break;
        case 4:
            if (std::isnan(val) || val < 0.0) val = 0.0;
            if (val > 100.0) val = 100.0;
            if (inst->fisheye != val) { inst->fisheye = val; inst->dirty = true; }
            break;
        case 5:
            if (std::isnan(val) || val < 0.0) val = 1.0;
            if (inst->interpolation != val) { inst->interpolation = val; }
            break;
        default:
            break;
    }
}

void f0r_get_param_value(f0r_instance_t instance, f0r_param_t param, int param_index) {
    auto* inst = static_cast<EqToRectInstance*>(instance);
    auto* p = static_cast<double*>(param);
    switch (param_index) {
        case 0: *p = inst->yaw; break;
        case 1: *p = inst->pitch; break;
        case 2: *p = inst->roll; break;
        case 3: *p = inst->fov; break;
        case 4: *p = inst->fisheye; break;
        case 5: *p = inst->interpolation; break;
        default: break;
    }
}

void f0r_update(f0r_instance_t instance, double time, const uint32_t* inframe, uint32_t* outframe) {
    (void)time;
    static_cast<EqToRectInstance*>(instance)->process(inframe, outframe);
}

void f0r_update2(f0r_instance_t instance, double time, const uint32_t* inframe1, const uint32_t* inframe2, const uint32_t* inframe3, uint32_t* outframe) {
    (void)time;
    (void)inframe2;
    (void)inframe3;
    static_cast<EqToRectInstance*>(instance)->process(inframe1, outframe);
}

} // extern "C"
