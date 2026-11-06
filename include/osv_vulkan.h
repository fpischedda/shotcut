#ifndef OSV_VULKAN_H
#define OSV_VULKAN_H

#include "osv_metadata.h"
#include <vulkan/vulkan.h>
#include <vector>
#include <string>

struct PushConstants {
    float lens1_intrinsics[4]; // fx, fy, cx, cy
    float lens1_dist[4];       // k1, k2, k3, k4
    float lens1_quat[4];       // x, y, z, w
    float lens2_intrinsics[4];
    float lens2_dist[4];
    float lens2_quat[4];
    float gyro_quat[4];
    uint32_t outWidth;
    uint32_t outHeight;
    float maxFovDegrees;
    float blendOverlapDegrees;
};

class OsvVulkanStitcher {
public:
    OsvVulkanStitcher();
    ~OsvVulkanStitcher();

    bool initialize(const std::string& spv_shader_path);
    void cleanup();

    bool stitchFrames(
        const uint8_t* front_rgba,
        const uint8_t* rear_rgba,
        uint32_t sensor_w,
        uint32_t sensor_h,
        const LensCalibration& lens_front,
        const LensCalibration& lens_rear,
        const float* gyro_quat,
        uint32_t out_w,
        uint32_t out_h,
        uint8_t* out_equirect_rgba
    );

private:
    VkInstance instance_ = VK_NULL_HANDLE;
    VkPhysicalDevice physical_device_ = VK_NULL_HANDLE;
    VkDevice device_ = VK_NULL_HANDLE;
    VkQueue compute_queue_ = VK_NULL_HANDLE;
    uint32_t queue_family_index_ = 0;

    VkShaderModule shader_module_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout desc_layout_ = VK_NULL_HANDLE;
    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
    VkPipeline pipeline_ = VK_NULL_HANDLE;
    VkCommandPool command_pool_ = VK_NULL_HANDLE;

    bool initVulkanCore();
    bool createPipeline(const std::string& spv_path);
};

#endif // OSV_VULKAN_H
