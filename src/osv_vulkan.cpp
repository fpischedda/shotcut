#include "osv_vulkan.h"
#include <iostream>
#include <fstream>
#include <cstring>
#include <stdexcept>

namespace {

uint32_t findMemoryType(VkPhysicalDevice phys, uint32_t typeFilter, VkMemoryPropertyFlags properties) {
    VkPhysicalDeviceMemoryProperties memProperties;
    vkGetPhysicalDeviceMemoryProperties(phys, &memProperties);

    for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
        if ((typeFilter & (1 << i)) && (memProperties.memoryTypes[i].propertyFlags & properties) == properties) {
            return i;
        }
    }
    throw std::runtime_error("failed to find suitable memory type!");
}

std::vector<char> readFile(const std::string& filename) {
    std::ifstream file(filename, std::ios::ate | std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("failed to open file: " + filename);
    }
    size_t fileSize = (size_t)file.tellg();
    std::vector<char> buffer(fileSize);
    file.seekg(0);
    file.read(buffer.data(), fileSize);
    file.close();
    return buffer;
}

} // namespace

OsvVulkanStitcher::OsvVulkanStitcher() {}

OsvVulkanStitcher::~OsvVulkanStitcher() {
    cleanup();
}

bool OsvVulkanStitcher::initVulkanCore() {
    VkApplicationInfo appInfo{};
    appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    appInfo.pApplicationName = "OSV Native Vulkan Stitcher";
    appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
    appInfo.pEngineName = "OSV Engine";
    appInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
    appInfo.apiVersion = VK_API_VERSION_1_2;

    VkInstanceCreateInfo createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    createInfo.pApplicationInfo = &appInfo;

    if (vkCreateInstance(&createInfo, nullptr, &instance_) != VK_SUCCESS) {
        std::cerr << "Failed to create Vulkan instance" << std::endl;
        return false;
    }

    uint32_t deviceCount = 0;
    vkEnumeratePhysicalDevices(instance_, &deviceCount, nullptr);
    if (deviceCount == 0) {
        std::cerr << "Failed to find GPUs with Vulkan support" << std::endl;
        return false;
    }

    std::vector<VkPhysicalDevice> devices(deviceCount);
    vkEnumeratePhysicalDevices(instance_, &deviceCount, devices.data());
    physical_device_ = devices[0]; // Select primary GPU (Intel Arc)

    VkPhysicalDeviceProperties props;
    vkGetPhysicalDeviceProperties(physical_device_, &props);
    std::cout << "[OSV-Vulkan] Using GPU: " << props.deviceName << std::endl;

    uint32_t queueFamilyCount = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(physical_device_, &queueFamilyCount, nullptr);
    std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
    vkGetPhysicalDeviceQueueFamilyProperties(physical_device_, &queueFamilyCount, queueFamilies.data());

    bool foundQueue = false;
    for (uint32_t i = 0; i < queueFamilyCount; i++) {
        if (queueFamilies[i].queueFlags & VK_QUEUE_COMPUTE_BIT) {
            queue_family_index_ = i;
            foundQueue = true;
            break;
        }
    }

    if (!foundQueue) {
        std::cerr << "Failed to find compute queue family" << std::endl;
        return false;
    }

    float queuePriority = 1.0f;
    VkDeviceQueueCreateInfo queueCreateInfo{};
    queueCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
    queueCreateInfo.queueFamilyIndex = queue_family_index_;
    queueCreateInfo.queueCount = 1;
    queueCreateInfo.pQueuePriorities = &queuePriority;

    VkPhysicalDeviceFeatures deviceFeatures{};
    VkDeviceCreateInfo devCreateInfo{};
    devCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    devCreateInfo.pQueueCreateInfos = &queueCreateInfo;
    devCreateInfo.queueCreateInfoCount = 1;
    devCreateInfo.pEnabledFeatures = &deviceFeatures;

    if (vkCreateDevice(physical_device_, &devCreateInfo, nullptr, &device_) != VK_SUCCESS) {
        std::cerr << "Failed to create logical Vulkan device" << std::endl;
        return false;
    }

    vkGetDeviceQueue(device_, queue_family_index_, 0, &compute_queue_);

    VkCommandPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    poolInfo.queueFamilyIndex = queue_family_index_;
    poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;

    if (vkCreateCommandPool(device_, &poolInfo, nullptr, &command_pool_) != VK_SUCCESS) {
        std::cerr << "Failed to create compute command pool" << std::endl;
        return false;
    }

    return true;
}

bool OsvVulkanStitcher::createPipeline(const std::string& spv_path) {
    std::vector<char> code;
    try {
        code = readFile(spv_path);
    } catch (const std::exception& e) {
        std::cerr << "Shader error: " << e.what() << std::endl;
        return false;
    }

    VkShaderModuleCreateInfo createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    createInfo.codeSize = code.size();
    createInfo.pCode = reinterpret_cast<const uint32_t*>(code.data());

    if (vkCreateShaderModule(device_, &createInfo, nullptr, &shader_module_) != VK_SUCCESS) {
        std::cerr << "Failed to create compute shader module" << std::endl;
        return false;
    }

    VkDescriptorSetLayoutBinding bindings[3]{};
    // Binding 0: outEquirect storage image
    bindings[0].binding = 0;
    bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
    bindings[0].descriptorCount = 1;
    bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    // Binding 1: lensFront sampler
    bindings[1].binding = 1;
    bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    bindings[1].descriptorCount = 1;
    bindings[1].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    // Binding 2: lensRear sampler
    bindings[2].binding = 2;
    bindings[2].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    bindings[2].descriptorCount = 1;
    bindings[2].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    VkDescriptorSetLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutInfo.bindingCount = 3;
    layoutInfo.pBindings = bindings;

    if (vkCreateDescriptorSetLayout(device_, &layoutInfo, nullptr, &desc_layout_) != VK_SUCCESS) {
        std::cerr << "Failed to create descriptor set layout" << std::endl;
        return false;
    }

    VkPushConstantRange pushConstant{};
    pushConstant.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    pushConstant.offset = 0;
    pushConstant.size = sizeof(PushConstants);

    VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
    pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &desc_layout_;
    pipelineLayoutInfo.pushConstantRangeCount = 1;
    pipelineLayoutInfo.pPushConstantRanges = &pushConstant;

    if (vkCreatePipelineLayout(device_, &pipelineLayoutInfo, nullptr, &pipeline_layout_) != VK_SUCCESS) {
        std::cerr << "Failed to create pipeline layout" << std::endl;
        return false;
    }

    VkComputePipelineCreateInfo pipelineInfo{};
    pipelineInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    pipelineInfo.layout = pipeline_layout_;
    pipelineInfo.stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    pipelineInfo.stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    pipelineInfo.stage.module = shader_module_;
    pipelineInfo.stage.pName = "main";

    if (vkCreateComputePipelines(device_, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &pipeline_) != VK_SUCCESS) {
        std::cerr << "Failed to create compute pipeline" << std::endl;
        return false;
    }

    return true;
}

bool OsvVulkanStitcher::initialize(const std::string& spv_shader_path) {
    if (!initVulkanCore()) return false;
    if (!createPipeline(spv_shader_path)) return false;
    return true;
}

void OsvVulkanStitcher::cleanup() {
    if (device_ != VK_NULL_HANDLE) {
        vkDeviceWaitIdle(device_);
        if (pipeline_ != VK_NULL_HANDLE) vkDestroyPipeline(device_, pipeline_, nullptr);
        if (pipeline_layout_ != VK_NULL_HANDLE) vkDestroyPipelineLayout(device_, pipeline_layout_, nullptr);
        if (desc_layout_ != VK_NULL_HANDLE) vkDestroyDescriptorSetLayout(device_, desc_layout_, nullptr);
        if (shader_module_ != VK_NULL_HANDLE) vkDestroyShaderModule(device_, shader_module_, nullptr);
        if (command_pool_ != VK_NULL_HANDLE) vkDestroyCommandPool(device_, command_pool_, nullptr);
        vkDestroyDevice(device_, nullptr);
        device_ = VK_NULL_HANDLE;
    }
    if (instance_ != VK_NULL_HANDLE) {
        vkDestroyInstance(instance_, nullptr);
        instance_ = VK_NULL_HANDLE;
    }
}

bool OsvVulkanStitcher::stitchFrames(
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
) {
    if (!device_ || !pipeline_) return false;

    // Helper to create and bind buffer
    auto createBuffer = [&](VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags props, VkBuffer& buffer, VkDeviceMemory& memory) {
        VkBufferCreateInfo bufInfo{};
        bufInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bufInfo.size = size;
        bufInfo.usage = usage;
        bufInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        vkCreateBuffer(device_, &bufInfo, nullptr, &buffer);

        VkMemoryRequirements memReq;
        vkGetBufferMemoryRequirements(device_, buffer, &memReq);
        VkMemoryAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        allocInfo.allocationSize = memReq.size;
        allocInfo.memoryTypeIndex = findMemoryType(physical_device_, memReq.memoryTypeBits, props);
        vkAllocateMemory(device_, &allocInfo, nullptr, &memory);
        vkBindBufferMemory(device_, buffer, memory, 0);
    };

    // Helper to create 2D image
    auto createImage = [&](uint32_t w, uint32_t h, VkFormat fmt, VkImageUsageFlags usage, VkImage& img, VkDeviceMemory& mem) {
        VkImageCreateInfo imgInfo{};
        imgInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
        imgInfo.imageType = VK_IMAGE_TYPE_2D;
        imgInfo.extent.width = w;
        imgInfo.extent.height = h;
        imgInfo.extent.depth = 1;
        imgInfo.mipLevels = 1;
        imgInfo.arrayLayers = 1;
        imgInfo.format = fmt;
        imgInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
        imgInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        imgInfo.usage = usage;
        imgInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        imgInfo.samples = VK_SAMPLE_COUNT_1_BIT;
        vkCreateImage(device_, &imgInfo, nullptr, &img);

        VkMemoryRequirements memReq;
        vkGetImageMemoryRequirements(device_, img, &memReq);
        VkMemoryAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        allocInfo.allocationSize = memReq.size;
        allocInfo.memoryTypeIndex = findMemoryType(physical_device_, memReq.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        vkAllocateMemory(device_, &allocInfo, nullptr, &mem);
        vkBindImageMemory(device_, img, mem, 0);
    };

    auto createImageView = [&](VkImage img, VkFormat fmt) {
        VkImageViewCreateInfo vInfo{};
        vInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        vInfo.image = img;
        vInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        vInfo.format = fmt;
        vInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        vInfo.subresourceRange.baseMipLevel = 0;
        vInfo.subresourceRange.levelCount = 1;
        vInfo.subresourceRange.baseArrayLayer = 0;
        vInfo.subresourceRange.layerCount = 1;
        VkImageView view;
        vkCreateImageView(device_, &vInfo, nullptr, &view);
        return view;
    };

    VkDeviceSize sensorBytes = (VkDeviceSize)sensor_w * sensor_h * 4;
    VkDeviceSize outBytes = (VkDeviceSize)out_w * out_h * 4;

    // Staging buffers for upload and download
    VkBuffer stageInFront, stageInRear, stageOut;
    VkDeviceMemory stageInFrontMem, stageInRearMem, stageOutMem;
    createBuffer(sensorBytes, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stageInFront, stageInFrontMem);
    createBuffer(sensorBytes, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stageInRear, stageInRearMem);
    createBuffer(outBytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stageOut, stageOutMem);

    void* dataFront;
    vkMapMemory(device_, stageInFrontMem, 0, sensorBytes, 0, &dataFront);
    std::memcpy(dataFront, front_rgba, sensorBytes);
    vkUnmapMemory(device_, stageInFrontMem);

    void* dataRear;
    vkMapMemory(device_, stageInRearMem, 0, sensorBytes, 0, &dataRear);
    std::memcpy(dataRear, rear_rgba, sensorBytes);
    vkUnmapMemory(device_, stageInRearMem);

    // Device Images
    VkImage imgFront, imgRear, imgOut;
    VkDeviceMemory memFront, memRear, memOut;
    createImage(sensor_w, sensor_h, VK_FORMAT_R8G8B8A8_UNORM, VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, imgFront, memFront);
    createImage(sensor_w, sensor_h, VK_FORMAT_R8G8B8A8_UNORM, VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, imgRear, memRear);
    createImage(out_w, out_h, VK_FORMAT_R8G8B8A8_UNORM, VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT, imgOut, memOut);

    VkImageView viewFront = createImageView(imgFront, VK_FORMAT_R8G8B8A8_UNORM);
    VkImageView viewRear = createImageView(imgRear, VK_FORMAT_R8G8B8A8_UNORM);
    VkImageView viewOut = createImageView(imgOut, VK_FORMAT_R8G8B8A8_UNORM);

    VkSampler sampler;
    VkSamplerCreateInfo samplerInfo{};
    samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    samplerInfo.magFilter = VK_FILTER_LINEAR;
    samplerInfo.minFilter = VK_FILTER_LINEAR;
    samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    vkCreateSampler(device_, &samplerInfo, nullptr, &sampler);

    // Descriptor Pool & Set
    VkDescriptorPoolSize poolSizes[2]{};
    poolSizes[0].type = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
    poolSizes[0].descriptorCount = 1;
    poolSizes[1].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    poolSizes[1].descriptorCount = 2;

    VkDescriptorPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.maxSets = 1;
    poolInfo.poolSizeCount = 2;
    poolInfo.pPoolSizes = poolSizes;
    VkDescriptorPool descPool;
    vkCreateDescriptorPool(device_, &poolInfo, nullptr, &descPool);

    VkDescriptorSetAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = descPool;
    allocInfo.descriptorSetCount = 1;
    allocInfo.pSetLayouts = &desc_layout_;
    VkDescriptorSet descSet;
    vkAllocateDescriptorSets(device_, &allocInfo, &descSet);

    VkDescriptorImageInfo outInfo{};
    outInfo.imageView = viewOut;
    outInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;

    VkDescriptorImageInfo frontInfo{};
    frontInfo.sampler = sampler;
    frontInfo.imageView = viewFront;
    frontInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkDescriptorImageInfo rearInfo{};
    rearInfo.sampler = sampler;
    rearInfo.imageView = viewRear;
    rearInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkWriteDescriptorSet writes[3]{};
    writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[0].dstSet = descSet;
    writes[0].dstBinding = 0;
    writes[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
    writes[0].descriptorCount = 1;
    writes[0].pImageInfo = &outInfo;

    writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[1].dstSet = descSet;
    writes[1].dstBinding = 1;
    writes[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writes[1].descriptorCount = 1;
    writes[1].pImageInfo = &frontInfo;

    writes[2].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[2].dstSet = descSet;
    writes[2].dstBinding = 2;
    writes[2].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writes[2].descriptorCount = 1;
    writes[2].pImageInfo = &rearInfo;

    vkUpdateDescriptorSets(device_, 3, writes, 0, nullptr);

    // Command Buffer recording
    VkCommandBufferAllocateInfo cmdAlloc{};
    cmdAlloc.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    cmdAlloc.commandPool = command_pool_;
    cmdAlloc.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cmdAlloc.commandBufferCount = 1;
    VkCommandBuffer cmd;
    vkAllocateCommandBuffers(device_, &cmdAlloc, &cmd);

    VkCommandBufferBeginInfo beginInfo{};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    vkBeginCommandBuffer(cmd, &beginInfo);

    // Transition input images to TRANSFER_DST and copy from staging
    auto transition = [&](VkImage img, VkImageLayout oldL, VkImageLayout newL, VkAccessFlags srcA, VkAccessFlags dstA, VkPipelineStageFlags srcS, VkPipelineStageFlags dstS) {
        VkImageMemoryBarrier barrier{};
        barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        barrier.oldLayout = oldL;
        barrier.newLayout = newL;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.image = img;
        barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        barrier.subresourceRange.baseMipLevel = 0;
        barrier.subresourceRange.levelCount = 1;
        barrier.subresourceRange.baseArrayLayer = 0;
        barrier.subresourceRange.layerCount = 1;
        barrier.srcAccessMask = srcA;
        barrier.dstAccessMask = dstA;
        vkCmdPipelineBarrier(cmd, srcS, dstS, 0, 0, nullptr, 0, nullptr, 1, &barrier);
    };

    transition(imgFront, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 0, VK_ACCESS_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT);
    transition(imgRear, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 0, VK_ACCESS_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT);
    transition(imgOut, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL, 0, VK_ACCESS_SHADER_WRITE_BIT, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);

    VkBufferImageCopy region{};
    region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    region.imageSubresource.layerCount = 1;
    region.imageExtent = {sensor_w, sensor_h, 1};

    vkCmdCopyBufferToImage(cmd, stageInFront, imgFront, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);
    vkCmdCopyBufferToImage(cmd, stageInRear, imgRear, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);

    transition(imgFront, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
    transition(imgRear, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);

    // Bind Pipeline & Push Constants
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_layout_, 0, 1, &descSet, 0, nullptr);

    PushConstants pc{};
    pc.lens1_intrinsics[0] = lens_front.fx;
    pc.lens1_intrinsics[1] = lens_front.fy;
    pc.lens1_intrinsics[2] = lens_front.cx;
    pc.lens1_intrinsics[3] = lens_front.cy;
    std::memcpy(pc.lens1_dist, &lens_front.k1, 4 * sizeof(float));
    std::memcpy(pc.lens1_quat, lens_front.quat, 4 * sizeof(float));

    pc.lens2_intrinsics[0] = lens_rear.fx;
    pc.lens2_intrinsics[1] = lens_rear.fy;
    pc.lens2_intrinsics[2] = lens_rear.cx;
    pc.lens2_intrinsics[3] = lens_rear.cy;
    std::memcpy(pc.lens2_dist, &lens_rear.k1, 4 * sizeof(float));
    std::memcpy(pc.lens2_quat, lens_rear.quat, 4 * sizeof(float));

    if (gyro_quat) {
        std::memcpy(pc.gyro_quat, gyro_quat, 4 * sizeof(float));
    } else {
        pc.gyro_quat[0] = 0.0f; pc.gyro_quat[1] = 0.0f; pc.gyro_quat[2] = 0.0f; pc.gyro_quat[3] = 1.0f;
    }

    pc.outWidth = out_w;
    pc.outHeight = out_h;
    pc.maxFovDegrees = 195.0f;
    pc.blendOverlapDegrees = 15.0f;

    vkCmdPushConstants(cmd, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(PushConstants), &pc);

    // Dispatch compute groups (16x16 workgroup size)
    uint32_t groupX = (out_w + 15) / 16;
    uint32_t groupY = (out_h + 15) / 16;
    vkCmdDispatch(cmd, groupX, groupY, 1);

    // Transition output image to TRANSFER_SRC and copy back to staging
    transition(imgOut, VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_TRANSFER_READ_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT);

    VkBufferImageCopy outRegion{};
    outRegion.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    outRegion.imageSubresource.layerCount = 1;
    outRegion.imageExtent = {out_w, out_h, 1};
    vkCmdCopyImageToBuffer(cmd, imgOut, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, stageOut, 1, &outRegion);

    vkEndCommandBuffer(cmd);

    // Submit and wait
    VkSubmitInfo submitInfo{};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &cmd;

    VkFence fence;
    VkFenceCreateInfo fenceInfo{};
    fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    vkCreateFence(device_, &fenceInfo, nullptr, &fence);

    vkQueueSubmit(compute_queue_, 1, &submitInfo, fence);
    vkWaitForFences(device_, 1, &fence, VK_TRUE, UINT64_MAX);

    // Read back data
    void* mappedOut;
    vkMapMemory(device_, stageOutMem, 0, outBytes, 0, &mappedOut);
    std::memcpy(out_equirect_rgba, mappedOut, outBytes);
    vkUnmapMemory(device_, stageOutMem);

    // Cleanup resources
    vkDestroyFence(device_, fence, nullptr);
    vkFreeCommandBuffers(device_, command_pool_, 1, &cmd);
    vkDestroyDescriptorPool(device_, descPool, nullptr);
    vkDestroySampler(device_, sampler, nullptr);
    vkDestroyImageView(device_, viewFront, nullptr);
    vkDestroyImageView(device_, viewRear, nullptr);
    vkDestroyImageView(device_, viewOut, nullptr);
    vkDestroyImage(device_, imgFront, nullptr);
    vkDestroyImage(device_, imgRear, nullptr);
    vkDestroyImage(device_, imgOut, nullptr);
    vkFreeMemory(device_, memFront, nullptr);
    vkFreeMemory(device_, memRear, nullptr);
    vkFreeMemory(device_, memOut, nullptr);
    vkDestroyBuffer(device_, stageInFront, nullptr);
    vkDestroyBuffer(device_, stageInRear, nullptr);
    vkDestroyBuffer(device_, stageOut, nullptr);
    vkFreeMemory(device_, stageInFrontMem, nullptr);
    vkFreeMemory(device_, stageInRearMem, nullptr);
    vkFreeMemory(device_, stageOutMem, nullptr);

    return true;
}
