#include "osv_metadata.h"
#include <iostream>
#include <fstream>
#include <cstring>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
}

namespace {

uint64_t readVarint(const uint8_t*& ptr, const uint8_t* end) {
    uint64_t result = 0;
    int shift = 0;
    while (ptr < end) {
        uint8_t byte = *ptr++;
        result |= static_cast<uint64_t>(byte & 0x7F) << shift;
        if ((byte & 0x80) == 0) break;
        shift += 7;
    }
    return result;
}

float readFloat(const uint8_t*& ptr, const uint8_t* end) {
    if (ptr + 4 > end) return 0.0f;
    float val;
    std::memcpy(&val, ptr, 4);
    ptr += 4;
    return val;
}

void parseLensMessage(const uint8_t* data, size_t size, LensCalibration& lens) {
    const uint8_t* ptr = data;
    const uint8_t* end = data + size;

    while (ptr < end) {
        uint64_t tag = readVarint(ptr, end);
        uint32_t field_num = tag >> 3;
        uint32_t wire_type = tag & 0x7;

        if (wire_type == 5) { // 32-bit float
            float val = readFloat(ptr, end);
            switch (field_num) {
                case 1: lens.fx = val; break;
                case 2: lens.fy = val; break;
                case 3: lens.cx = val; break;
                case 4: lens.cy = val; break;
                case 5: lens.k1 = val; break;
                case 6: lens.k2 = val; break;
                case 7: lens.k3 = val; break;
                case 8: lens.k4 = val; break;
                default: break;
            }
        } else if (wire_type == 2) { // length-delimited
            uint64_t len = readVarint(ptr, end);
            if (field_num == 28 && len >= 16) { // orientation quaternion submessage
                const uint8_t* sub_ptr = ptr;
                const uint8_t* sub_end = ptr + len;
                int q_idx = 0;
                while (sub_ptr < sub_end && q_idx < 4) {
                    uint64_t sub_tag = readVarint(sub_ptr, sub_end);
                    uint32_t sub_wire = sub_tag & 0x7;
                    if (sub_wire == 5) {
                        lens.quat[q_idx++] = readFloat(sub_ptr, sub_end);
                    } else if (sub_wire == 0) {
                        readVarint(sub_ptr, sub_end);
                    } else if (sub_wire == 2) {
                        uint64_t l = readVarint(sub_ptr, sub_end);
                        sub_ptr += l;
                    }
                }
            }
            ptr += len;
        } else if (wire_type == 0) {
            readVarint(ptr, end);
        } else if (wire_type == 1) {
            ptr += 8;
        } else {
            break;
        }
    }
}

} // namespace

bool OsvMetadataParser::parseFromBuffer(const uint8_t* data, size_t size, OsvMetadata& out_meta) {
    const uint8_t* ptr = data;
    const uint8_t* end = data + size;

    while (ptr < end) {
        uint64_t tag = readVarint(ptr, end);
        uint32_t field_num = tag >> 3;
        uint32_t wire_type = tag & 0x7;

        if (wire_type != 2) {
            if (wire_type == 0) readVarint(ptr, end);
            else if (wire_type == 1) ptr += 8;
            else if (wire_type == 5) ptr += 4;
            continue;
        }

        uint64_t len = readVarint(ptr, end);
        if (ptr + len > end) break;

        const uint8_t* msg_start = ptr;
        const uint8_t* msg_end = ptr + len;

        if (field_num == 1) {
            // Header message
            const uint8_t* h_ptr = msg_start;
            while (h_ptr < msg_end) {
                uint64_t h_tag = readVarint(h_ptr, msg_end);
                uint32_t h_field = h_tag >> 3;
                uint32_t h_wire = h_tag & 0x7;
                if (h_wire == 2) {
                    uint64_t s_len = readVarint(h_ptr, msg_end);
                    if (h_field == 1) {
                        // Submessage: proto name, serial, model
                        const uint8_t* sub_ptr = h_ptr;
                        const uint8_t* sub_end = h_ptr + s_len;
                        while (sub_ptr < sub_end) {
                            uint64_t t = readVarint(sub_ptr, sub_end);
                            uint32_t fn = t >> 3;
                            uint32_t wt = t & 0x7;
                            if (wt == 2) {
                                uint64_t sl = readVarint(sub_ptr, sub_end);
                                std::string str(reinterpret_cast<const char*>(sub_ptr), sl);
                                if (fn == 1) out_meta.proto_name = str;
                                else if (fn == 2) out_meta.firmware_version = str;
                                else if (fn == 5) out_meta.serial_number = str;
                                else if (fn == 10) out_meta.camera_model = str;
                                sub_ptr += sl;
                            } else if (wt == 0) readVarint(sub_ptr, sub_end);
                            else if (wt == 1) sub_ptr += 8;
                            else if (wt == 5) sub_ptr += 4;
                        }
                    }
                    h_ptr += s_len;
                } else if (h_wire == 0) {
                    uint64_t val = readVarint(h_ptr, msg_end);
                    if (h_field == 10) out_meta.sample_rate_hz = static_cast<uint32_t>(val);
                } else if (h_wire == 5) {
                    h_ptr += 4;
                } else if (h_wire == 1) {
                    h_ptr += 8;
                }
            }
        } else if (field_num == 2) {
            // Video / Calibration packet
            const uint8_t* v_ptr = msg_start;
            while (v_ptr < msg_end) {
                uint64_t v_tag = readVarint(v_ptr, msg_end);
                uint32_t v_field = v_tag >> 3;
                uint32_t v_wire = v_tag & 0x7;

                if (v_wire == 2) {
                    uint64_t v_len = readVarint(v_ptr, msg_end);
                    if (v_field == 6) { // Message containing lens 1 and lens 2 calibrations
                        const uint8_t* l_ptr = v_ptr;
                        const uint8_t* l_end = v_ptr + v_len;
                        while (l_ptr < l_end) {
                            uint64_t l_tag = readVarint(l_ptr, l_end);
                            uint32_t l_field = l_tag >> 3;
                            uint32_t l_wire = l_tag & 0x7;
                            if (l_wire == 2) {
                                uint64_t lens_len = readVarint(l_ptr, l_end);
                                if (l_field == 1) {
                                    parseLensMessage(l_ptr, lens_len, out_meta.lens_front);
                                } else if (l_field == 2) {
                                    parseLensMessage(l_ptr, lens_len, out_meta.lens_rear);
                                }
                                l_ptr += lens_len;
                            } else if (l_wire == 0) readVarint(l_ptr, l_end);
                            else if (l_wire == 5) l_ptr += 4;
                            else if (l_wire == 1) l_ptr += 8;
                        }
                    }
                    v_ptr += v_len;
                } else if (v_wire == 0) readVarint(v_ptr, msg_end);
                else if (v_wire == 5) v_ptr += 4;
                else if (v_wire == 1) v_ptr += 8;
            }
        }

        ptr += len;
    }

    return !out_meta.proto_name.empty() || out_meta.lens_front.fx > 100.0f;
}

bool OsvMetadataParser::parseFromFile(const std::string& osv_path, OsvMetadata& out_meta) {
    AVFormatContext* fmt_ctx = nullptr;
    if (avformat_open_input(&fmt_ctx, osv_path.c_str(), nullptr, nullptr) < 0) {
        std::cerr << "Failed to open video container: " << osv_path << std::endl;
        return false;
    }

    if (avformat_find_stream_info(fmt_ctx, nullptr) < 0) {
        avformat_close_input(&fmt_ctx);
        return false;
    }

    int djmd_stream_idx = -1;
    for (unsigned int i = 0; i < fmt_ctx->nb_streams; ++i) {
        AVStream* st = fmt_ctx->streams[i];
        if (st->codecpar->codec_tag == MKTAG('d', 'j', 'm', 'd')) {
            djmd_stream_idx = i;
            break;
        }
    }

    if (djmd_stream_idx < 0) {
        std::cerr << "No 'djmd' telemetry stream found in: " << osv_path << std::endl;
        avformat_close_input(&fmt_ctx);
        return false;
    }

    AVPacket* pkt = av_packet_alloc();
    bool parsed = false;

    // Read the first few packets containing the header and calibration
    while (av_read_frame(fmt_ctx, pkt) >= 0) {
        if (pkt->stream_index == djmd_stream_idx) {
            if (parseFromBuffer(pkt->data, pkt->size, out_meta)) {
                parsed = true;
                break;
            }
        }
        av_packet_unref(pkt);
    }

    av_packet_free(&pkt);
    avformat_close_input(&fmt_ctx);
    return parsed;
}
