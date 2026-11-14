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
    unsigned int outWidth;
    unsigned int outHeight;
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
    float safeTheta = fminf(theta, 1.635f);
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

extern "C" __global__ void osv_stitch_kernel(cudaSurfaceObject_t outEquirect,
                                             cudaTextureObject_t lensMaster,
                                             cudaTextureObject_t lensSlave,
                                             StitchUniforms params)
{
    unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
    unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x >= params.outWidth || y >= params.outHeight)
        return;

    float u = ((float) x + 0.5f) / (float) params.outWidth;
    float v = ((float) y + 0.5f) / (float) params.outHeight;

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
    float uLen = sqrtf(nBisector.y * nBisector.y + nBisector.x * nBisector.x);
    float3 uSeam = make_float3(nBisector.y / uLen, -nBisector.x / uLen, 0.0f);

    float d_n = dBody.x * nBisector.x + dBody.y * nBisector.y + dBody.z * nBisector.z;
    float d_u = dBody.x * uSeam.x + dBody.y * uSeam.y + dBody.z * uSeam.z;
    float phiSeam = (d_u < 0.0f) ? atan2f(d_n, -d_u) : atan2f(d_n, d_u);

    // Rotate body ray directly into Master lens frame (+z optical axis): d_lens = R * d_body
    float3 dMaster = quatRotateWXYZ(params.lens_master.quat, dBody);
    float rhoMaster = sqrtf(dMaster.x * dMaster.x + dMaster.y * dMaster.y);
    float thMaster = atan2f(rhoMaster, dMaster.z);

    // Rotate body ray directly into Slave lens frame (+z optical axis): d_lens = R * d_body
    float3 dSlave = quatRotateWXYZ(params.lens_slave.quat, dBody);
    float rhoSlave = sqrtf(dSlave.x * dSlave.x + dSlave.y * dSlave.y);
    float thSlave = atan2f(rhoSlave, dSlave.z);

    float maxTheta = fminf((params.maxFovDegrees * 0.5f) * (PI / 180.0f), 1.635f);
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

    float phiCut = 0.0f;
    if (fabsf(phiSeam) < halfOverlap && validMaster && validSlave) {
        float3 sVec = (d_u < 0.0f) ? make_float3(-uSeam.x, -uSeam.y, -uSeam.z) : uSeam;
        float3 vSeam = make_float3(nBisector.y * sVec.z - nBisector.z * sVec.y,
                                   nBisector.z * sVec.x - nBisector.x * sVec.z,
                                   nBisector.x * sVec.y - nBisector.y * sVec.x);
        if (vSeam.z < 0.0f) {
            vSeam = make_float3(-vSeam.x, -vSeam.y, -vSeam.z);
        }
        float3 onSeam = make_float3(dBody.x - d_n * nBisector.x,
                                    dBody.y - d_n * nBisector.y,
                                    dBody.z - d_n * nBisector.z);
        float onLen = sqrtf(onSeam.x * onSeam.x + onSeam.y * onSeam.y + onSeam.z * onSeam.z);
        float3 dOnSeam = make_float3(onSeam.x / onLen, onSeam.y / onLen, onSeam.z / onLen);
        float alpha = atan2f(dOnSeam.x * vSeam.x + dOnSeam.y * vSeam.y + dOnSeam.z * vSeam.z,
                             dOnSeam.x * sVec.x + dOnSeam.y * sVec.y + dOnSeam.z * sVec.z);
        float deg2rad = PI / 180.0f;
        float stepAlpha = 8.0f * deg2rad;
        float aIdx = floorf(alpha / stepAlpha);
        float aFracVal = (alpha / stepAlpha) - aIdx;
        float aFrac = aFracVal * aFracVal * (3.0f - 2.0f * aFracVal);
        float maxSearch = 3.3f * deg2rad;
        float stepPhi = maxSearch / 6.0f;

        auto sampleTexClamped = [](cudaTextureObject_t tex, float2 uv) -> float3 {
            float4 c = tex2D<float4>(tex,
                                     fminf(fmaxf(uv.x, 0.0f), 1.0f),
                                     fminf(fmaxf(uv.y, 0.0f), 1.0f));
            return make_float3(c.x, c.y, c.z);
        };
        auto colorLen = [](float3 a, float3 b) -> float {
            float dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
            return sqrtf(dx * dx + dy * dy + dz * dz);
        };

        float knotPhi[2];
        for (int k = 0; k < 2; ++k) {
            float aK = (aIdx + (float) k) * stepAlpha;
            float bestCost = 1e9f;
            int bestIdx = 6;
            float costs[13];
            for (int i = -6; i <= 6; ++i) {
                float candPhi = (float) i * stepPhi;
                float diffAccum = 0.0f;
                float validCount = 0.0f;
                for (int j = -3; j <= 3; ++j) {
                    float aSub = aK + (float) j * 1.2f * deg2rad;
                    float ca = cosf(aSub), sa = sinf(aSub);
                    float3 dSub = make_float3(ca * sVec.x + sa * vSeam.x,
                                              ca * sVec.y + sa * vSeam.y,
                                              ca * sVec.z + sa * vSeam.z);
                    float cp = cosf(candPhi), sp = sinf(candPhi);
                    float3 dt = make_float3(cp * dSub.x + sp * nBisector.x,
                                            cp * dSub.y + sp * nBisector.y,
                                            cp * dSub.z + sp * nBisector.z);
                    float dtLen = sqrtf(dt.x * dt.x + dt.y * dt.y + dt.z * dt.z);
                    float3 dTest = make_float3(dt.x / dtLen, dt.y / dtLen, dt.z / dtLen);
                    float3 dm = quatRotateWXYZ(params.lens_master.quat, dTest);
                    float3 ds = quatRotateWXYZ(params.lens_slave.quat, dTest);
                    float rm = sqrtf(dm.x * dm.x + dm.y * dm.y);
                    float rs = sqrtf(ds.x * ds.x + ds.y * ds.y);
                    float tm = atan2f(rm, dm.z);
                    float ts = atan2f(rs, ds.z);
                    if (tm <= maxTheta && ts <= maxTheta) {
                        float2 uvm = projectKannalaBrandt(dm,
                                                          params.lens_master.intrinsics,
                                                          params.lens_master.dist,
                                                          sensorSize,
                                                          tm,
                                                          rm);
                        float2 uvs = projectKannalaBrandt(ds,
                                                          params.lens_slave.intrinsics,
                                                          params.lens_slave.dist,
                                                          sensorSize,
                                                          ts,
                                                          rs);
                        if (uvm.x >= 0.0f && uvm.x <= 1.0f && uvm.y >= 0.0f && uvm.y <= 1.0f
                            && uvs.x >= 0.0f && uvs.x <= 1.0f && uvs.y >= 0.0f && uvs.y <= 1.0f) {
                            float4 cm = tex2D<float4>(lensMaster, uvm.x, uvm.y);
                            float4 cs = tex2D<float4>(lensSlave, uvs.x, uvs.y);
                            float dx = cm.x - cs.x, dy = cm.y - cs.y, dz = cm.z - cs.z;
                            diffAccum += sqrtf(dx * dx + dy * dy + dz * dz);
                            validCount += 1.0f;
                        }
                    }
                }
                if (validCount > 0.0f) {
                    float avgDiff = diffAccum / validCount;
                    float normPhi = candPhi / maxSearch;
                    costs[i + 6] = avgDiff + 0.004f * normPhi * normPhi;
                } else {
                    costs[i + 6] = 1e6f;
                }
            }
            if (costs[6] > 0.06f && costs[6] < 1e5f) {
                float fgMaster = 0.0f, fgSlave = 0.0f;
                auto evalDir = [&](float3 dSub, float angDeg) -> float3 {
                    float r = angDeg * deg2rad;
                    float cp = cosf(r), sp = sinf(r);
                    float3 v = make_float3(cp * dSub.x + sp * nBisector.x,
                                           cp * dSub.y + sp * nBisector.y,
                                           cp * dSub.z + sp * nBisector.z);
                    float l = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
                    return make_float3(v.x / l, v.y / l, v.z / l);
                };
                for (int j = -2; j <= 2; ++j) {
                    float aSub = aK + (float) j * 1.2f * deg2rad;
                    float ca = cosf(aSub), sa = sinf(aSub);
                    float3 dSub = make_float3(ca * sVec.x + sa * vSeam.x,
                                              ca * sVec.y + sa * vSeam.y,
                                              ca * sVec.z + sa * vSeam.z);
                    float3 dm0 = quatRotateWXYZ(params.lens_master.quat, dSub);
                    float3 dm1 = quatRotateWXYZ(params.lens_master.quat, evalDir(dSub, 1.5f));
                    float3 dm2 = quatRotateWXYZ(params.lens_master.quat, evalDir(dSub, 3.0f));
                    float3 ds0 = quatRotateWXYZ(params.lens_slave.quat, dSub);
                    float3 ds1 = quatRotateWXYZ(params.lens_slave.quat, evalDir(dSub, -1.5f));
                    float3 ds2 = quatRotateWXYZ(params.lens_slave.quat, evalDir(dSub, -3.0f));
                    auto projM = [&](float3 d) {
                        float r = sqrtf(d.x * d.x + d.y * d.y);
                        return sampleTexClamped(lensMaster,
                                                projectKannalaBrandt(d,
                                                                     params.lens_master.intrinsics,
                                                                     params.lens_master.dist,
                                                                     sensorSize,
                                                                     atan2f(r, d.z),
                                                                     r));
                    };
                    auto projS = [&](float3 d) {
                        float r = sqrtf(d.x * d.x + d.y * d.y);
                        return sampleTexClamped(lensSlave,
                                                projectKannalaBrandt(d,
                                                                     params.lens_slave.intrinsics,
                                                                     params.lens_slave.dist,
                                                                     sensorSize,
                                                                     atan2f(r, d.z),
                                                                     r));
                    };
                    float3 cM0 = projM(dm0), cM1 = projM(dm1), cM2 = projM(dm2);
                    float3 cS0 = projS(ds0), cS1 = projS(ds1), cS2 = projS(ds2);
                    fgMaster += colorLen(cM0, cM1) + colorLen(cM1, cM2) + colorLen(cM0, cM2);
                    fgSlave += colorLen(cS0, cS1) + colorLen(cS1, cS2) + colorLen(cS0, cS2);
                }
                float dirBias = fminf(fmaxf((fgMaster - fgSlave) * 0.08f, -0.35f), 0.35f);
                for (int idx = 0; idx < 13; ++idx) {
                    if (costs[idx] < 1e5f) {
                        float normPhi = (float) (idx - 6) / 6.0f;
                        costs[idx] += dirBias * normPhi;
                        if (fgSlave > fgMaster + 0.15f && idx < 6) {
                            costs[idx] += 0.5f;
                        } else if (fgMaster > fgSlave + 0.15f && idx > 6) {
                            costs[idx] += 0.5f;
                        }
                    }
                }
            }
            for (int idx = 0; idx < 13; ++idx) {
                if (costs[idx] < bestCost) {
                    bestCost = costs[idx];
                    bestIdx = idx;
                }
            }
            float subStep = 0.0f;
            if (bestIdx > 0 && bestIdx < 12 && costs[bestIdx - 1] < 1e5f
                && costs[bestIdx + 1] < 1e5f) {
                float denom = costs[bestIdx - 1] - 2.0f * costs[bestIdx] + costs[bestIdx + 1];
                if (denom > 1e-4f) {
                    subStep = fminf(fmaxf(0.5f * (costs[bestIdx - 1] - costs[bestIdx + 1]) / denom,
                                          -0.5f),
                                    0.5f);
                }
            }
            knotPhi[k] = ((float) (bestIdx - 6) + subStep) * stepPhi;
        }
        phiCut = knotPhi[0] + aFrac * (knotPhi[1] - knotPhi[0]);
    }

    float blendHalf = 1.2f * (PI / 180.0f);
    float blendFactor = smoothstep(phiCut - blendHalf, phiCut + blendHalf, phiSeam);

    float wMaster = validMaster ? blendFactor : 0.0f;
    float wSlave = validSlave ? (1.0f - blendFactor) : 0.0f;

    float4 colorMaster = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
    float4 colorSlave = make_float4(0.0f, 0.0f, 0.0f, 0.0f);

    if (wMaster > 0.0f) {
        colorMaster = tex2D<float4>(lensMaster,
                                    fminf(fmaxf(rawUvMaster.x, 0.0f), 1.0f),
                                    fminf(fmaxf(rawUvMaster.y, 0.0f), 1.0f));
    }

    if (wSlave > 0.0f) {
        colorSlave = tex2D<float4>(lensSlave,
                                   fminf(fmaxf(rawUvSlave.x, 0.0f), 1.0f),
                                   fminf(fmaxf(rawUvSlave.y, 0.0f), 1.0f));
    }

    if (wMaster > 0.0f && wSlave > 0.0f) {
        float dx = colorMaster.x - colorSlave.x;
        float dy = colorMaster.y - colorSlave.y;
        float dz = colorMaster.z - colorSlave.z;
        float colorDiff = sqrtf(dx * dx + dy * dy + dz * dz);
        float mismatch = smoothstep(0.06f, 0.25f, colorDiff);
        float minHalf = 0.6f * (PI / 180.0f);
        float effectiveHalf = blendHalf + mismatch * (minHalf - blendHalf);
        float adaptiveBlend = smoothstep(phiCut - effectiveHalf, phiCut + effectiveHalf, phiSeam);
        wMaster = adaptiveBlend;
        wSlave = 1.0f - adaptiveBlend;
    }

    float totalWeight = wMaster + wSlave;
    uchar4 pixelOut;
    if (totalWeight > 1e-4f) {
        pixelOut.x = (unsigned char) __saturatef((colorMaster.x * wMaster + colorSlave.x * wSlave)
                                                 / totalWeight * 255.0f);
        pixelOut.y = (unsigned char) __saturatef((colorMaster.y * wMaster + colorSlave.y * wSlave)
                                                 / totalWeight * 255.0f);
        pixelOut.z = (unsigned char) __saturatef((colorMaster.z * wMaster + colorSlave.z * wSlave)
                                                 / totalWeight * 255.0f);
        pixelOut.w = 255;
    } else {
        pixelOut = make_uchar4(0, 0, 0, 255);
    }

    surf2Dwrite(pixelOut, outEquirect, x * sizeof(uchar4), y);
}

#endif // OSV_STITCH_CU
