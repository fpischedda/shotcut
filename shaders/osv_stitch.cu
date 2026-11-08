#ifndef OSV_STITCH_CU
#define OSV_STITCH_CU

#include <cuda_runtime.h>
#include <math.h>

#define PI 3.14159265358979323846f

struct LensParams {
    float4 intrinsics; // fx, fy, cx, cy
    float4 dist;       // k1, k2, k3, k4
    float4 quat;       // w, x, y, z
};

struct StitchUniforms {
    LensParams lens_master;
    LensParams lens_slave;
    float4 gyro_quat; // w, x, y, z
    unsigned int outWidth;
    unsigned int outHeight;
    float maxFovDegrees;
    float blendOverlapDegrees;
};

__device__ inline float3 quatRotateWXYZ(float4 q, float3 v) {
    float3 qvec = make_float3(q.y, q.z, q.w); // x, y, z
    float3 uv = make_float3(
        qvec.y * v.z - qvec.z * v.y,
        qvec.z * v.x - qvec.x * v.z,
        qvec.x * v.y - qvec.y * v.x
    );
    float3 uuv = make_float3(
        qvec.y * uv.z - qvec.z * uv.y,
        qvec.z * uv.x - qvec.x * uv.z,
        qvec.x * uv.y - qvec.y * uv.x
    );
    return make_float3(
        v.x + ((uv.x * q.x) + uuv.x) * 2.0f,
        v.y + ((uv.y * q.x) + uuv.y) * 2.0f,
        v.z + ((uv.z * q.x) + uuv.z) * 2.0f
    );
}

__device__ inline float4 quatConjugateWXYZ(float4 q) {
    return make_float4(q.x, -q.y, -q.z, -q.w);
}

__device__ inline float2 projectKannalaBrandt(float3 dLens, float4 intrinsics, float4 dist, float2 texSize, float theta, float rho) {
    if (rho <= 1e-6f) {
        return make_float2(intrinsics.z / texSize.x, intrinsics.w / texSize.y);
    }
    float t2 = theta * theta;
    float t4 = t2 * t2;
    float t6 = t4 * t2;
    float t8 = t4 * t4;
    float rd = theta * (1.0f + dist.x * t2 + dist.y * t4 + dist.z * t6 + dist.w * t8);

    float px = intrinsics.z + intrinsics.x * rd * (dLens.x / rho);
    float py = intrinsics.w + intrinsics.y * rd * (dLens.y / rho);

    return make_float2(px / texSize.x, py / texSize.y);
}

__device__ inline float smoothstep(float edge0, float edge1, float x) {
    float t = fminf(fmaxf((x - edge0) / (edge1 - edge0), 0.0f), 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

extern "C" __global__ void osv_stitch_kernel(
    cudaSurfaceObject_t outEquirect,
    cudaTextureObject_t lensMaster,
    cudaTextureObject_t lensSlave,
    StitchUniforms params
) {
    unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
    unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x >= params.outWidth || y >= params.outHeight) return;

    float u = ((float)x + 0.5f) / (float)params.outWidth;
    float v = ((float)y + 0.5f) / (float)params.outHeight;

    float lon = (u - 0.5f) * 2.0f * PI - (PI * 0.5f);
    float lat = (0.5f - v) * PI;
    float cosLat = cosf(lat);

    float3 dBody = make_float3(
        sinf(lon) * cosLat,
        cosf(lon) * cosLat,
        sinf(lat)
    );

    if (fabsf(params.gyro_quat.x - 1.0f) > 1e-5f ||
        fabsf(params.gyro_quat.y) > 1e-5f ||
        fabsf(params.gyro_quat.z) > 1e-5f ||
        fabsf(params.gyro_quat.w) > 1e-5f) {
        float4 invGyro = quatConjugateWXYZ(params.gyro_quat);
        dBody = quatRotateWXYZ(invGyro, dBody);
    }

    float3 dMaster = quatRotateWXYZ(params.lens_master.quat, dBody);
    float rhoMaster = sqrtf(dMaster.x * dMaster.x + dMaster.y * dMaster.y);
    float thMaster = atan2f(rhoMaster, dMaster.z);

    float3 dSlave = quatRotateWXYZ(params.lens_slave.quat, dBody);
    float rhoSlave = sqrtf(dSlave.x * dSlave.x + dSlave.y * dSlave.y);
    float thSlave = atan2f(rhoSlave, dSlave.z);

    float maxTheta = (params.maxFovDegrees * 0.5f) * (PI / 180.0f);
    float blendOverlap = params.blendOverlapDegrees * (PI / 180.0f);
    float blendStart = maxTheta - blendOverlap;
    float blendEnd = maxTheta;

    float wMaster = (thMaster <= maxTheta) ? (1.0f - smoothstep(blendStart, blendEnd, thMaster)) : 0.0f;
    float wSlave  = (thSlave  <= maxTheta) ? (1.0f - smoothstep(blendStart, blendEnd, thSlave))  : 0.0f;

    float4 colorMaster = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
    float4 colorSlave  = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
    float2 sensorSize = make_float2(3840.0f, 3840.0f);

    if (wMaster > 0.0f) {
        float2 uvMaster = projectKannalaBrandt(dMaster, params.lens_master.intrinsics, params.lens_master.dist, sensorSize, thMaster, rhoMaster);
        if (uvMaster.x >= 0.0f && uvMaster.x <= 1.0f && uvMaster.y >= 0.0f && uvMaster.y <= 1.0f) {
            colorMaster = tex2D<float4>(lensMaster, uvMaster.x, uvMaster.y);
        } else {
            wMaster = 0.0f;
        }
    }

    if (wSlave > 0.0f) {
        float2 uvSlave = projectKannalaBrandt(dSlave, params.lens_slave.intrinsics, params.lens_slave.dist, sensorSize, thSlave, rhoSlave);
        if (uvSlave.x >= 0.0f && uvSlave.x <= 1.0f && uvSlave.y >= 0.0f && uvSlave.y <= 1.0f) {
            colorSlave = tex2D<float4>(lensSlave, uvSlave.x, uvSlave.y);
        } else {
            wSlave = 0.0f;
        }
    }

    float totalWeight = wMaster + wSlave;
    uchar4 pixelOut;
    if (totalWeight > 1e-4f) {
        pixelOut.x = (unsigned char)__saturatef((colorMaster.x * wMaster + colorSlave.x * wSlave) / totalWeight * 255.0f);
        pixelOut.y = (unsigned char)__saturatef((colorMaster.y * wMaster + colorSlave.y * wSlave) / totalWeight * 255.0f);
        pixelOut.z = (unsigned char)__saturatef((colorMaster.z * wMaster + colorSlave.z * wSlave) / totalWeight * 255.0f);
        pixelOut.w = 255;
    } else {
        pixelOut = make_uchar4(0, 0, 0, 255);
    }

    surf2Dwrite(pixelOut, outEquirect, x * sizeof(uchar4), y);
}

#endif // OSV_STITCH_CU
