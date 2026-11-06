#ifndef OSV_STITCH_CU
#define OSV_STITCH_CU

#include <cuda_runtime.h>
#include <math.h>

#define PI 3.14159265358979323846f

struct LensParams {
    float4 intrinsics; // fx, fy, cx, cy
    float4 dist;       // k1, k2, k3, k4
    float4 quat;       // x, y, z, w
};

struct StitchUniforms {
    LensParams lens1;
    LensParams lens2;
    float4 gyro_quat;
    unsigned int outWidth;
    unsigned int outHeight;
    float maxFovDegrees;
    float blendOverlapDegrees;
};

__device__ inline float3 rotateByQuat(float4 q, float3 v) {
    float3 qvec = make_float3(q.x, q.y, q.z);
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
        v.x + ((uv.x * q.w) + uuv.x) * 2.0f,
        v.y + ((uv.y * q.w) + uuv.y) * 2.0f,
        v.z + ((uv.z * q.w) + uuv.z) * 2.0f
    );
}

__device__ inline float4 quatConjugate(float4 q) {
    return make_float4(-q.x, -q.y, -q.z, q.w);
}

__device__ inline float2 projectFisheye(float3 ray, float4 intrinsics, float4 dist, float2 texSize) {
    float r_xy = sqrtf(ray.x * ray.x + ray.y * ray.y);
    float theta = atan2f(r_xy, ray.z);

    float t2 = theta * theta;
    float t4 = t2 * t2;
    float t6 = t4 * t2;
    float t8 = t4 * t4;
    float theta_d = theta * (1.0f + dist.x * t2 + dist.y * t4 + dist.z * t6 + dist.w * t8);

    float dir_x = (r_xy > 1e-6f) ? (ray.x / r_xy) : 0.0f;
    float dir_y = (r_xy > 1e-6f) ? (ray.y / r_xy) : 0.0f;

    float px = intrinsics.z + intrinsics.x * theta_d * dir_x;
    float py = intrinsics.w + intrinsics.y * theta_d * dir_y;

    return make_float2(px / texSize.x, py / texSize.y);
}

__device__ inline float smoothstep(float edge0, float edge1, float x) {
    float t = fminf(fmaxf((x - edge0) / (edge1 - edge0), 0.0f), 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

extern "C" __global__ void osv_stitch_kernel(
    cudaSurfaceObject_t outEquirect,
    cudaTextureObject_t lensFront,
    cudaTextureObject_t lensRear,
    StitchUniforms params
) {
    unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
    unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x >= params.outWidth || y >= params.outHeight) return;

    float u = (float)x / (float)params.outWidth;
    float v = (float)y / (float)params.outHeight;

    float lambda = (u - 0.5f) * 2.0f * PI;
    float phi = (0.5f - v) * PI;

    float3 ray = make_float3(
        cosf(phi) * sinf(lambda),
        sinf(phi),
        cosf(phi) * cosf(lambda)
    );

    float4 invGyro = quatConjugate(params.gyro_quat);
    ray = rotateByQuat(invGyro, ray);

    float4 invLens1 = quatConjugate(params.lens1.quat);
    float3 rayFront = rotateByQuat(invLens1, ray);
    float thetaFront = atan2f(sqrtf(rayFront.x * rayFront.x + rayFront.y * rayFront.y), rayFront.z);

    float4 invLens2 = quatConjugate(params.lens2.quat);
    float3 rayRear = rotateByQuat(invLens2, ray);
    float thetaRear = atan2f(sqrtf(rayRear.x * rayRear.x + rayRear.y * rayRear.y), rayRear.z);

    float maxTheta = (params.maxFovDegrees * 0.5f) * (PI / 180.0f);
    float blendHalf = (params.blendOverlapDegrees * 0.5f) * (PI / 180.0f);
    float blendStart = maxTheta - blendHalf;
    float blendEnd = maxTheta + blendHalf;

    float wFront = 1.0f - smoothstep(blendStart, blendEnd, thetaFront);
    float wRear  = 1.0f - smoothstep(blendStart, blendEnd, thetaRear);

    float4 colorFront = make_float4(0.0f, 0.0f, 0.0f, 1.0f);
    float4 colorRear  = make_float4(0.0f, 0.0f, 0.0f, 1.0f);
    float2 sensorSize = make_float2(3840.0f, 3840.0f);

    if (wFront > 0.0f) {
        float2 uvFront = projectFisheye(rayFront, params.lens1.intrinsics, params.lens1.dist, sensorSize);
        if (uvFront.x >= 0.0f && uvFront.x <= 1.0f && uvFront.y >= 0.0f && uvFront.y <= 1.0f) {
            colorFront = tex2D<float4>(lensFront, uvFront.x, uvFront.y);
        } else {
            wFront = 0.0f;
        }
    }

    if (wRear > 0.0f) {
        float2 uvRear = projectFisheye(rayRear, params.lens2.intrinsics, params.lens2.dist, sensorSize);
        if (uvRear.x >= 0.0f && uvRear.x <= 1.0f && uvRear.y >= 0.0f && uvRear.y <= 1.0f) {
            colorRear = tex2D<float4>(lensRear, uvRear.x, uvRear.y);
        } else {
            wRear = 0.0f;
        }
    }

    float totalWeight = wFront + wRear;
    uchar4 outPixel;
    if (totalWeight > 1e-4f) {
        float r = (colorFront.x * wFront + colorRear.x * wRear) / totalWeight;
        float g = (colorFront.y * wFront + colorRear.y * wRear) / totalWeight;
        float b = (colorFront.z * wFront + colorRear.z * wRear) / totalWeight;
        outPixel = make_uchar4(
            (unsigned char)fminf(fmaxf(r * 255.0f, 0.0f), 255.0f),
            (unsigned char)fminf(fmaxf(g * 255.0f, 0.0f), 255.0f),
            (unsigned char)fminf(fmaxf(b * 255.0f, 0.0f), 255.0f),
            255
        );
    } else {
        outPixel = make_uchar4(0, 0, 0, 255);
    }

    surf2Dwrite(outPixel, outEquirect, x * sizeof(uchar4), y);
}

#endif // OSV_STITCH_CU
