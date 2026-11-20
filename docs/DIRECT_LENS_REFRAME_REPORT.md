# Direct Dual-Fisheye to Rectilinear Projection — Implementation & Investigation Report

**Branch:** `feat/direct-lens-reframe` (paired with `shotcut-upstream-pr` branch `feat/osv-support`)

---

## 1. What Was Implemented

1. **Single-Pass Direct Vulkan Compute Shader (`shaders/direct_reframe.comp`)**
   - Inverse ray-tracing from output rectilinear pixels (`yaw`, `pitch`, `roll`, `fov`) directly to the two Kannala–Brandt fisheye sensors (`lensMaster`, `lensSlave`), avoiding the intermediate equirectangular stitch + second resample.
   - Single-lens fast path when `abs(phiSeam) > halfOverlap` (skips second sensor fetch and seam blending).
   - **Jacobian-Aware EWA Resampling** (`sampleLensEWA`): uses 1-pixel neighbor finite differences (`uvDx`, `uvDy`) in texel space and applies a 3×3 Gaussian-weighted elliptical kernel when `maxFootprint > 1.25` texels.
   - **Monotonic Lens Limits**: enforces `92.0°` (`1.6057 rad`) for Master and `95.4°` (`1.6650 rad`) for Slave so clamped/streaked outer rings (`drd/dtheta <= 0`) are never sampled.

2. **Vulkan Pipeline & Seam Optimization (`include/osv_vulkan.h`, `src/osv_vulkan.cpp`)**
   - Added `OsvVulkanStitcher::initializeReframe(...)`, `reframeFrames(...)`, `reframeSideBySide(...)`, and `executeReframePass(...)`.
   - **Frustum-to-Overlap Culling**: samples a 5×5 ray grid across the rectilinear viewport and skips `computeSeamLut(...)` when the viewport lies inside a single hemisphere.
   - Fixed calibration reference resolution (`ubo.sensorWidth = 3840.0f`, `ubo.sensorHeight = 3840.0f`) so normalized UVs remain valid for both `3840x3840` `.OSV` and `1024x1024` `.LRF` proxy frames.
   - Upgraded `seamLut` to `VK_FORMAT_R32G32B32A32_SFLOAT` (`phiCut, dPhi, dAlpha, flowConf`) with 2D zero-mean SAD block-matching flow in `computeSeamLut(...)`.

3. **MLT Plugin & Shotcut Integration**
   - `include/dual_lens_types.h`: added `DualLensFramePayload` carrying `DualLensCalibration`, `gyro_quat[4]`, and the un-rescaled `raw_sbs_rgba` buffer (immune to MLT `loader` normalizing `resize` letterboxing).
   - `mlt-plugin/producer_osv.cpp`: added `raw_dual_mode` triggered by `osv.request_raw_dual=1` on `mlt_frame` or `osv.output_mode=raw_dual`.
   - `mlt-plugin/filter_dual_lens_reframe.cpp` & `mlt-plugin/mlt_osv.cpp`: registered `dual_lens_reframe` MLT filter with keyframeable `yaw`, `pitch`, `roll`, `fov`, `fisheye`, and `stitch_radius`.
   - `src/reframe_main.cpp` & `CMakeLists.txt`: added standalone `osv_reframe` CLI target.
   - `shotcut-upstream-pr/src/qml/filters/dual_lens_reframe/`: added Shotcut QML filter UI (`"360: Dual-Lens Direct to Rectilinear"`).
   - `run_shotcut.sh`: exports `OSV_REFRAME_SHADER_PATH` and prefers `../shotcut-upstream-pr/build/src/shotcut`.

---

## 2. Diagnostic Findings & Adjustments Tested

1. **`.OSV` `djmd` Metadata Inspection**
   - Dumped `djmd` protobuf packets from `CAM_20261113130137_0020_D.OSV`: contains header/camera info (`field 1`), static factory lens calibration (`field 2 -> f6`), and per-frame camera/AE + 1000 Hz IMU telemetry (`field 3`). **No depth or object-distance metadata** is present.

2. **Distant-Object Diagnostic (`R = 50 m`, Master-only vs. Slave-only)**
   - Rendering `Yaw = -175.729°, Pitch = 1.423°, FOV = 68.4°` at `R = 50 m` (zero baseline parallax) still showed a horizontal + vertical offset between Master and Slave at `theta ~ 90°`, confirming a residual 2D calibration offset at the lens periphery in addition to near-field parallax.

3. **Adjustments Evaluated**
   - **Epipolar 1D `stitchRadius(alpha)`**: could not correct the 2D (vertical + horizontal) peripheral offset and caused line warping when `R` varied rapidly across `alpha`.
   - **Per-knot 2D Flow Warp**: narrow `2.5°` warp support caused Jacobian reversal (S-bends in floor cracks, ripples in buildings); widening to `±halfOverlap` (`14.5°`) removed the S-bend, but varying flow across sky/building/rail/floor still bent straight lines like the skatepark rail.
   - **Global 3-DOF Rigid Rotation (`rigidOmega`)**: fitting a single rigid rotation across all 128 knots was skewed by the skater's arm/hand at ~0.3 m (which occupies ~50% of the seam ring), tilting the distant building and creating a hard vertical cut.
   - **Current Configuration (Zero-Warp + Balanced `0.35°–1.00°` Seam)**: preserves exact Kannala–Brandt geometry (straight rail, ledge, and floor lines) while avoiding hard knife-cuts and minimizing ghosting.
   - **Preview Resolution (`OSV_PREVIEW_MODE=full` vs `auto`)**: `auto` uses the `1024x1024`-per-eye `.LRF` proxy, which has low peripheral pixel density at `88°–92°`. `OSV_PREVIEW_MODE=full` (and exports) use full `3840x3840` streams and eliminate peripheral blur.

---

## 3. Running on Another Host

```bash
git checkout feat/direct-lens-reframe
cmake -B build && cmake --build build -j$(nproc)

# If using the uninstalled shotcut-upstream-pr/build binary for export:
ln -sf "$(command -v melt-7)" ../shotcut-upstream-pr/build/src/melt-7

# Launch Shotcut (use OSV_PREVIEW_MODE=full for sharp 4K rim detail)
OSV_PREVIEW_MODE=full ./run_shotcut.sh /path/to/clip.OSV
```

Standalone CLI & MLT tests:
```bash
./build/osv_reframe /path/to/clip.OSV -180 2 0 75 reframed.ppm
./bin/melt-osv -silent -profile atsc_1080p_25 /path/to/clip.OSV out=1 -attach dual_lens_reframe yaw=-180 pitch=2 fov=90 -consumer avformat:out.mp4 vcodec=libx264 real_time=0 </dev/null
```

---

## 4. Next Steps

1. **Two-Band / Laplacian Pyramid Blending in Overlap Zone**: separate low-frequency color/exposure from high-frequency edges so the seam transition can stay sharp on textures while blending brightness/vignetting smoothly across `±halfOverlap` without any visible step on the ledge or floor.
2. **Background-Only Rigid Calibration**: if pursuing global rigid alignment, filter out high-disparity/near-field knots (e.g. `varM` / `bestSad` + RANSAC over `dPhi, dAlpha`) so close objects like hands/selfie sticks do not skew the distant horizon alignment.
3. **Fisheye / Panini Blend Support**: implement `rf.fisheyeAmount > 0` in `shaders/direct_reframe.comp` to match `bigsh0t_eq_to_rect` when the Shotcut `Fisheye` slider is non-zero.
