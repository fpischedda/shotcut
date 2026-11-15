#ifndef OSV_STITCH_CU
#define OSV_STITCH_CU

#include <cuda_runtime.h>
#include <math.h>

#define PI 3.14159265358979323846f

struct LensParams
{
    float4 intrinsics; // fx, fy, cx, cy
    float4 dist;       // k1, k2, k3, k4
    float4 quat;       // w, x, y, z
};

struct StitchUniforms
{
    LensParams lens_master;
    LensParams lens_slave;
    float4 gyro_quat; // w, x, y, z
    unsigned int packedDims;
    float stitchRadius;
    float maxFovDegrees;
    float blendOverlapDegrees;
};

__device__ inline float3 quatRotateWXYZ(float4 q, float3 v)
{
    float3 qvec = make_float3(q.y, q.z, q.w); // x, y, z
    float3 uv = make_float3(qvec.y * v.z - qvec.z * v.y,
                            qvec.z * v.x - qvec.x * v.z,
                            qvec.x * v.y - qvec.y * v.x);
    float3 uuv = make_float3(qvec.y * uv.z - qvec.z * uv.y,
                             qvec.z * uv.x - qvec.x * uv.z,
                             qvec.x * uv.y - qvec.y * uv.x);
    return make_float3(v.x + ((uv.x * q.x) + uuv.x) * 2.0f,
                       v.y + ((uv.y * q.x) + uuv.y) * 2.0f,
                       v.z + ((uv.z * q.x) + uuv.z) * 2.0f);
}

__device__ inline float4 quatConjugateWXYZ(float4 q)
{
    return make_float4(q.x, -q.y, -q.z, -q.w);
}

__device__ inline float2 projectKannalaBrandt(
    float3 dLens, float4 intrinsics, float4 dist, float2 texSize, float theta, float rho)
{
    if (rho <= 1e-6f) {
        return make_float2(intrinsics.z / texSize.x, intrinsics.w / texSize.y);
    }
    float th0 = 1.57079632679f; // 90.0 degrees (PI * 0.5)
    float rd;
    if (theta <= th0) {
        float t2 = theta * theta;
        float t4 = t2 * t2;
        float t6 = t4 * t2;
        float t8 = t4 * t4;
        rd = theta * (1.0f + dist.x * t2 + dist.y * t4 + dist.z * t6 + dist.w * t8);
    } else {
        float t2 = th0 * th0;
        float t4 = t2 * t2;
        float t6 = t4 * t2;
        float t8 = t4 * t4;
        float rd0 = th0 * (1.0f + dist.x * t2 + dist.y * t4 + dist.z * t6 + dist.w * t8);
        float drd0 = 1.0f + 3.0f * dist.x * t2 + 5.0f * dist.y * t4 + 7.0f * dist.z * t6 + 9.0f * dist.w * t8;
        rd = rd0 + drd0 * (theta - th0);
    }

    float px = intrinsics.z + intrinsics.x * rd * (dLens.x / rho);
    float py = intrinsics.w + intrinsics.y * rd * (dLens.y / rho);

    return make_float2(px / texSize.x, py / texSize.y);
}

__device__ inline float smoothstep(float edge0, float edge1, float x)
{
    float t = fminf(fmaxf((x - edge0) / (edge1 - edge0), 0.0f), 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

__device__ inline float3 offsetToLensMaster(float3 dirBody, float3 aMaster, float stitchRadius)
{
    if (stitchRadius > 0.05f && stitchRadius < 100.0f) {
        float3 p = make_float3(stitchRadius * dirBody.x - (0.5f * 0.024f) * aMaster.x,
                               stitchRadius * dirBody.y - (0.5f * 0.024f) * aMaster.y,
                               stitchRadius * dirBody.z - (0.5f * 0.024f) * aMaster.z);
        float len = sqrtf(p.x * p.x + p.y * p.y + p.z * p.z);
        return make_float3(p.x / len, p.y / len, p.z / len);
    }
    return dirBody;
}

__device__ inline float3 offsetToLensSlave(float3 dirBody, float3 aSlave, float stitchRadius)
{
    if (stitchRadius > 0.05f && stitchRadius < 100.0f) {
        float3 p = make_float3(stitchRadius * dirBody.x - (0.5f * 0.024f) * aSlave.x,
                               stitchRadius * dirBody.y - (0.5f * 0.024f) * aSlave.y,
                               stitchRadius * dirBody.z - (0.5f * 0.024f) * aSlave.z);
        float len = sqrtf(p.x * p.x + p.y * p.y + p.z * p.z);
        return make_float3(p.x / len, p.y / len, p.z / len);
    }
    return dirBody;
}

extern "C" __global__ void osv_stitch_kernel(cudaSurfaceObject_t outEquirect,
                                             cudaTextureObject_t lensMaster,
                                             cudaTextureObject_t lensSlave,
                                             cudaTextureObject_t seamLut,
                                             StitchUniforms params)
{
    unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
    unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;

    unsigned int outWidth = params.packedDims & 0xFFFF;
    unsigned int outHeight = (params.packedDims >> 16) & 0xFFFF;

    if (x >= outWidth || y >= outHeight)
        return;

    float u = ((float) x + 0.5f) / (float) outWidth;
    float v = ((float) y + 0.5f) / (float) outHeight;

    float lon = (u - 0.5f) * 2.0f * PI - (PI * 0.5f);
    float lat = (0.5f - v) * PI;
    float cosLat = cosf(lat);

    float3 dBody = make_float3(sinf(lon) * cosLat, cosf(lon) * cosLat, sinf(lat));

    if (fabsf(params.gyro_quat.x - 1.0f) > 1e-5f || fabsf(params.gyro_quat.y) > 1e-5f
        || fabsf(params.gyro_quat.z) > 1e-5f || fabsf(params.gyro_quat.w) > 1e-5f) {
        float4 invGyro = quatConjugateWXYZ(params.gyro_quat);
        dBody = quatRotateWXYZ(invGyro, dBody);
    }

    float3 aMaster = quatRotateWXYZ(quatConjugateWXYZ(params.lens_master.quat),
                                    make_float3(0.0f, 0.0f, 1.0f));
    float3 aSlave = quatRotateWXYZ(quatConjugateWXYZ(params.lens_slave.quat),
                                   make_float3(0.0f, 0.0f, 1.0f));
    float3 diff = make_float3(aMaster.x - aSlave.x, aMaster.y - aSlave.y, aMaster.z - aSlave.z);
    float nLen = sqrtf(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);
    float3 nBisector = make_float3(diff.x / nLen, diff.y / nLen, diff.z / nLen);
    float3 uSeam = make_float3(aMaster.y * aSlave.z - aMaster.z * aSlave.y,
                               aMaster.z * aSlave.x - aMaster.x * aSlave.z,
                               aMaster.x * aSlave.y - aMaster.y * aSlave.x);
    float uLen = sqrtf(uSeam.x * uSeam.x + uSeam.y * uSeam.y + uSeam.z * uSeam.z);
    uSeam = make_float3(uSeam.x / uLen, uSeam.y / uLen, uSeam.z / uLen);
    float3 vSeam = make_float3(nBisector.y * uSeam.z - nBisector.z * uSeam.y,
                               nBisector.z * uSeam.x - nBisector.x * uSeam.z,
                               nBisector.x * uSeam.y - nBisector.y * uSeam.x);
    float vLen = sqrtf(vSeam.x * vSeam.x + vSeam.y * vSeam.y + vSeam.z * vSeam.z);
    vSeam = make_float3(vSeam.x / vLen, vSeam.y / vLen, vSeam.z / vLen);

    float d_n = dBody.x * nBisector.x + dBody.y * nBisector.y + dBody.z * nBisector.z;
    float phiSeam = asinf(fminf(fmaxf(d_n, -1.0f), 1.0f));
    float alpha = atan2f(dBody.x * vSeam.x + dBody.y * vSeam.y + dBody.z * vSeam.z,
                         dBody.x * uSeam.x + dBody.y * uSeam.y + dBody.z * uSeam.z);

    // Apply physical baseline offset to project equirectangular rays to virtual sphere at stitchRadius
    float3 dBodyMaster = offsetToLensMaster(dBody, aMaster, params.stitchRadius);
    float3 dBodySlave = offsetToLensSlave(dBody, aSlave, params.stitchRadius);

    // Rotate body ray directly into Master lens frame (+z optical axis): d_lens = R * d_body
    float3 dMaster = quatRotateWXYZ(params.lens_master.quat, dBodyMaster);
    float rhoMaster = sqrtf(dMaster.x * dMaster.x + dMaster.y * dMaster.y);
    float thMaster = atan2f(rhoMaster, dMaster.z);

    // Rotate body ray directly into Slave lens frame (+z optical axis): d_lens = R * d_body
    float3 dSlave = quatRotateWXYZ(params.lens_slave.quat, dBodySlave);
    float rhoSlave = sqrtf(dSlave.x * dSlave.x + dSlave.y * dSlave.y);
    float thSlave = atan2f(rhoSlave, dSlave.z);

    float maxTheta = (params.maxFovDegrees * 0.5f) * (PI / 180.0f);
    float halfOverlap = (params.blendOverlapDegrees * 0.5f) * (PI / 180.0f);
    float2 sensorSize = make_float2(3840.0f, 3840.0f);

    float2 rawUvMaster = projectKannalaBrandt(dMaster,
                                              params.lens_master.intrinsics,
                                              params.lens_master.dist,
                                              sensorSize,
                                              thMaster,
                                              rhoMaster);
    float2 rawUvSlave = projectKannalaBrandt(dSlave,
                                             params.lens_slave.intrinsics,
                                             params.lens_slave.dist,
                                             sensorSize,
                                             thSlave,
                                             rhoSlave);
    bool validMaster = (thMaster <= maxTheta) && rawUvMaster.x >= 0.0f && rawUvMaster.x <= 1.0f
                       && rawUvMaster.y >= 0.0f && rawUvMaster.y <= 1.0f;
    bool validSlave = (thSlave <= maxTheta) && rawUvSlave.x >= 0.0f && rawUvSlave.x <= 1.0f
                      && rawUvSlave.y >= 0.0f && rawUvSlave.y <= 1.0f;

    // Sample continuous, globally optimal seam cut angle from dynamic-programming seam LUT.
    float normAlpha = (alpha + PI) / (2.0f * PI);
    float phiCut = tex2D<float>(seamLut, normAlpha, 0.5f);

    float deg2rad = PI / 180.0f;
    float blendHalf = 1.2f * deg2rad;

    float4 colorMaster = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
    float4 colorSlave = make_float4(0.0f, 0.0f, 0.0f, 0.0f);

    if (validMaster) {
        colorMaster = tex2D<float4>(lensMaster,
                                    fminf(fmaxf(rawUvMaster.x, 0.0f), 1.0f),
                                    fminf(fmaxf(rawUvMaster.y, 0.0f), 1.0f));
    }

    if (validSlave) {
        colorSlave = tex2D<float4>(lensSlave,
                                   fminf(fmaxf(rawUvSlave.x, 0.0f), 1.0f),
                                   fminf(fmaxf(rawUvSlave.y, 0.0f), 1.0f));
    }

    float4 finalCol;
    if (validMaster && validSlave) {
        float tWide = smoothstep(-halfOverlap, halfOverlap, phiSeam);
        float dx = colorMaster.x - colorSlave.x;
        float dy = colorMaster.y - colorSlave.y;
        float dz = colorMaster.z - colorSlave.z;
        float colorDiff = sqrtf(dx * dx + dy * dy + dz * dz);
        float exposureWeight = smoothstep(0.18f, 0.05f, colorDiff);

        float deltaX = fminf(fmaxf((colorMaster.x - colorSlave.x) * exposureWeight, -0.15f), 0.15f);
        float deltaY = fminf(fmaxf((colorMaster.y - colorSlave.y) * exposureWeight, -0.15f), 0.15f);
        float deltaZ = fminf(fmaxf((colorMaster.z - colorSlave.z) * exposureWeight, -0.15f), 0.15f);

        float balSlaveX = colorSlave.x + tWide * deltaX;
        float balSlaveY = colorSlave.y + tWide * deltaY;
        float balSlaveZ = colorSlave.z + tWide * deltaZ;

        float balMasterX = colorMaster.x - (1.0f - tWide) * deltaX;
        float balMasterY = colorMaster.y - (1.0f - tWide) * deltaY;
        float balMasterZ = colorMaster.z - (1.0f - tWide) * deltaZ;

        float mismatch = smoothstep(0.05f, 0.15f, colorDiff);
        float minHalf = 0.02f * deg2rad;
        float effectiveHalf = blendHalf + mismatch * (minHalf - blendHalf);
        float adaptiveBlend = smoothstep(phiCut - effectiveHalf, phiCut + effectiveHalf, phiSeam);

        finalCol.x = balSlaveX + adaptiveBlend * (balMasterX - balSlaveX);
        finalCol.y = balSlaveY + adaptiveBlend * (balMasterY - balSlaveY);
        finalCol.z = balSlaveZ + adaptiveBlend * (balMasterZ - balSlaveZ);
        finalCol.w = 1.0f;
    } else if (validMaster) {
        finalCol = colorMaster;
    } else if (validSlave) {
        finalCol = colorSlave;
    } else {
        finalCol = make_float4(0.0f, 0.0f, 0.0f, 1.0f);
    }

    uchar4 pixelOut;
    pixelOut.x = (unsigned char) __saturatef(finalCol.x) * 255.0f;
    pixelOut.y = (unsigned char) __saturatef(finalCol.y) * 255.0f;
    pixelOut.z = (unsigned char) __saturatef(finalCol.z) * 255.0f;
    pixelOut.w = 255;
    surf2Dwrite(pixelOut, outEquirect, x * sizeof(uchar4), y);
}

#endif // OSV_STITCH_CU
