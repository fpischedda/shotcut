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
    // Factory calibrated Kannala-Brandt distortion model:
    // Evaluate full 8th-order polynomial up to stationary peak (drd/dtheta = 0)
    // Master: ~1.63587 rad (93.73 deg), Slave: ~1.70008 rad (97.41 deg)
    float thPeak = (dist.w < -0.002f) ? 1.63587f : 1.70008f;
    float safeTheta = fminf(theta, thPeak);
    float t2 = safeTheta * safeTheta;
    float t4 = t2 * t2;
    float t6 = t4 * t2;
    float t8 = t4 * t4;
    float rd = safeTheta * (1.0f + dist.x * t2 + dist.y * t4 + dist.z * t6 + dist.w * t8);

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

__device__ inline float osv_smoothstep(float edge0, float edge1, float x)
{
    float t = fminf(fmaxf((x - edge0) / (edge1 - edge0), 0.0f), 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

// Elevation/ground-plane distance gradient:
// Models natural scene geometry (close ground at nadir, expanding towards horizon, distant sky)
// smoothly anchored around baseRadius for the primary subject.
__device__ inline float computeEffectiveRadius(float3 dirEquirect, float baseRadius)
{
    if (baseRadius <= 0.05f || baseRadius >= 100.0f) {
        return baseRadius;
    }
    float scale = baseRadius / 2.2f;
    float z = dirEquirect.z;
    if (z < 0.0f) {
        float rGround = fminf(fmaxf(1.4f / fmaxf(-z, 0.18f), 1.3f), 5.0f) * scale;
        float tHorizon = osv_smoothstep(-0.25f, 0.0f, z);
        float wNadir = 1.0f - osv_smoothstep(-0.75f, -0.35f, z);
        float r = baseRadius + tHorizon * (5.0f * scale - baseRadius);
        return r + wNadir * (rGround - r);
    } else {
        float tSky = osv_smoothstep(0.0f, 0.4f, z);
        float rHorizon = 5.0f * scale;
        float rSky = 8.0f * scale;
        return rHorizon + tSky * (rSky - rHorizon);
    }
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

    float3 dEquirect = make_float3(sinf(lon) * cosLat, cosf(lon) * cosLat, sinf(lat));
    float3 dBody = dEquirect;

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

    // Apply physical baseline offset with elevation-adaptive distance gradient
    float effRadius = computeEffectiveRadius(dEquirect, params.stitchRadius);
    float3 dBodyMaster = offsetToLensMaster(dBody, aMaster, effRadius);
    float3 dBodySlave = offsetToLensSlave(dBody, aSlave, effRadius);

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
        float2 texel = make_float2(3.0f / sensorSize.x, 3.0f / sensorSize.y);
        float4 mBox0 = tex2D<float4>(lensMaster, fminf(fmaxf(rawUvMaster.x - texel.x, 0.0f), 1.0f), fminf(fmaxf(rawUvMaster.y - texel.y, 0.0f), 1.0f));
        float4 mBox1 = tex2D<float4>(lensMaster, fminf(fmaxf(rawUvMaster.x + texel.x, 0.0f), 1.0f), fminf(fmaxf(rawUvMaster.y - texel.y, 0.0f), 1.0f));
        float4 mBox2 = tex2D<float4>(lensMaster, fminf(fmaxf(rawUvMaster.x - texel.x, 0.0f), 1.0f), fminf(fmaxf(rawUvMaster.y + texel.y, 0.0f), 1.0f));
        float4 mBox3 = tex2D<float4>(lensMaster, fminf(fmaxf(rawUvMaster.x + texel.x, 0.0f), 1.0f), fminf(fmaxf(rawUvMaster.y + texel.y, 0.0f), 1.0f));
        float3 mBox = make_float3(0.25f * (mBox0.x + mBox1.x + mBox2.x + mBox3.x),
                                  0.25f * (mBox0.y + mBox1.y + mBox2.y + mBox3.y),
                                  0.25f * (mBox0.z + mBox1.z + mBox2.z + mBox3.z));

        float4 sBox0 = tex2D<float4>(lensSlave, fminf(fmaxf(rawUvSlave.x - texel.x, 0.0f), 1.0f), fminf(fmaxf(rawUvSlave.y - texel.y, 0.0f), 1.0f));
        float4 sBox1 = tex2D<float4>(lensSlave, fminf(fmaxf(rawUvSlave.x + texel.x, 0.0f), 1.0f), fminf(fmaxf(rawUvSlave.y - texel.y, 0.0f), 1.0f));
        float4 sBox2 = tex2D<float4>(lensSlave, fminf(fmaxf(rawUvSlave.x - texel.x, 0.0f), 1.0f), fminf(fmaxf(rawUvSlave.y + texel.y, 0.0f), 1.0f));
        float4 sBox3 = tex2D<float4>(lensSlave, fminf(fmaxf(rawUvSlave.x + texel.x, 0.0f), 1.0f), fminf(fmaxf(rawUvSlave.y + texel.y, 0.0f), 1.0f));
        float3 sBox = make_float3(0.25f * (sBox0.x + sBox1.x + sBox2.x + sBox3.x),
                                  0.25f * (sBox0.y + sBox1.y + sBox2.y + sBox3.y),
                                  0.25f * (sBox0.z + sBox1.z + sBox2.z + sBox3.z));

        float sdx = mBox.x - sBox.x;
        float sdy = mBox.y - sBox.y;
        float sdz = mBox.z - sBox.z;
        float structDiff = sqrtf(sdx * sdx + sdy * sdy + sdz * sdz);

        float exposureWeight = osv_smoothstep(0.20f, 0.05f, structDiff);

        float deltaX = fminf(fmaxf((colorMaster.x - colorSlave.x) * exposureWeight, -0.15f), 0.15f);
        float deltaY = fminf(fmaxf((colorMaster.y - colorSlave.y) * exposureWeight, -0.15f), 0.15f);
        float deltaZ = fminf(fmaxf((colorMaster.z - colorSlave.z) * exposureWeight, -0.15f), 0.15f);

        float tWide = osv_smoothstep(-halfOverlap, halfOverlap, phiSeam);
        float balSlaveX = colorSlave.x + tWide * deltaX;
        float balSlaveY = colorSlave.y + tWide * deltaY;
        float balSlaveZ = colorSlave.z + tWide * deltaZ;

        float balMasterX = colorMaster.x - (1.0f - tWide) * deltaX;
        float balMasterY = colorMaster.y - (1.0f - tWide) * deltaY;
        float balMasterZ = colorMaster.z - (1.0f - tWide) * deltaZ;

        float mismatch = osv_smoothstep(0.06f, 0.22f, structDiff);
        float minHalf = 1.00f * deg2rad;
        float maxHalf = 2.00f * deg2rad;
        float effectiveHalf = maxHalf + mismatch * (minHalf - maxHalf);
        float adaptiveBlend = osv_smoothstep(phiCut - effectiveHalf, phiCut + effectiveHalf, phiSeam);

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
