# Investigation: Seam Blending, Lens Curvature, and Stream Transition

## 1. Context & Objectives
This document summarizes the investigation and state of the native OSV Vulkan stitcher and MLT producer (`plugins/libmltosv.so`) as of **v4.4-true-poly-monotonic (build 2026-11-16)**.
The objective is to eliminate seam artifacts (ghosting vs. knife-edge amputation on moving subjects) and geometric distortion (sharp kinks in straight ground lines approaching stream boundaries) when stitching dual-fisheye streams from DJI Osmo 360 (`.OSV` and companion `.LRF` files).

---

## 2. Key Findings & Diagnoses

### A. Ground Line Kinks & Artificial $90^\circ$ Tangent Cutoff
- **Symptom**: Straight lines on the ground (concrete joints, shadows) approaching the middle of the preview took a sudden sharp turn/kink.
- **Root Cause**:
  1. In `projectKannalaBrandt` (Vulkan GLSL & CUDA) and `kannalaBrandtRd` (CPU), the projection previously cut off the 8th-order Kannala-Brandt polynomial at $\theta_0 = 90.0^\circ$ ($\pi/2$) and switched to a linear tangent extrapolation:
     $$rd(\theta) = rd_0 + \left.\frac{drd}{d\theta}\right|_{\theta_0} \times (\theta - \theta_0)$$
  2. The mutual lens overlap exists between $82.5^\circ$ and $97.5^\circ$. Setting $\theta_0 = 90^\circ$ placed the cutoff in the middle of active scene geometry.
  3. Below $90^\circ$, the polynomial has optical fisheye barrel distortion curvature ($\frac{d^2 rd}{d\theta^2} < 0$). At $90^\circ$, the linear tangent forced $\frac{d^2 rd}{d\theta^2} = 0$, abruptly halting peripheral compression. By $96^\circ$, the tangent diverged from the true calibrated profile by over 27 pixels!
  4. The factory lens calibration is monotonic well past $90^\circ$:
     - **Master Lens (Stream 1, Rear)**: $\left.\frac{drd}{d\theta}\right| = 0$ at $\theta_{\text{peak}} = 1.63587\text{ rad}$ ($93.73^\circ$).
     - **Slave Lens (Stream 0, Front)**: $\left.\frac{drd}{d\theta}\right| = 0$ at $\theta_{\text{peak}} = 1.70008\text{ rad}$ ($97.41^\circ$).
- **Solution (v4.4)**:
  - Replaced the $90^\circ$ cutoff with the lens's physical stationary peak ($\theta_{\text{peak}}$).
  - Because $\left.\frac{drd}{d\theta}\right|_{\theta_{\text{peak}}} = 0$, clamping $\theta$ to $\theta_{\text{peak}}$ is naturally $C^1$ smooth and eliminates the slope discontinuity, preserving straight lines across the full overlap.

### B. Elevation Gradient Parallax Distortion
- **Symptom**: Lines on the ground warped non-linearly with vertical distance, and sheared across the seam.
- **Root Cause**: An elevation gradient (`computeEffectiveRadius`) was dynamically changing the focal sphere radius $R$ based on ray latitude $z$. Because $R$ varied along a straight line on the ground, the baseline ray deflection ($\pm 12\text{ mm} / R$) varied non-linearly. Across the seam, Master and Slave shifted by $24\text{ mm}$ in opposite directions, creating a sharp directional turn.
- **Solution (v4.4)**:
  - Removed `computeEffectiveRadius`.
  - Allowed clean selection between uniform virtual sphere projection (`stitchRadius = 2.2m`) or pure angular central projection (`stitchRadius = 0.0m`).
  - When `stitchRadius = 0.0m`, rays from Master and Slave originate from the common camera center, mathematically guaranteeing that all 3D straight lines remain perfectly straight in rectilinear perspective projection (`bigsh0t_eq_to_rect`).

### C. Seam Routing & Blend Transition
- **DP Seam LUT (`computeSeamLut`)**:
  - Dynamically routes a 128-knot seam path through the overlap band on companion `.LRF` proxy frames.
  - Cost function evaluates luminance differences, edge alignment, and circular continuity ($|\Delta c| \le 2$).
  - Search band is constrained to $\pm 6.0^\circ$ within valid optical overlap, steering around moving foreground subjects (e.g. skater).
- **Multi-Band Photometric & Soft Blending**:
  - High-frequency structural disparity is estimated using a 4-tap box filter.
  - Exposure equalization is gated by `exposureWeight = smoothstep(0.20, 0.05, structDiff)` to prevent milky halo ghosts.
  - Transition half-width adapts between $1.0^\circ$ and $2.0^\circ$ (`mix(radians(2.00), radians(1.00), mismatch)`), preventing razor knife-edge amputation of limbs while avoiding double-exposure ghosting.

---

## 3. Quickstart & Testing Commands

### Build from Source
```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

### Run Shotcut with Native Plugin
The repository includes a launcher script that sets all necessary MLT, Frei0r, and Vulkan shader paths:
```bash
./run_shotcut.sh /path/to/CAM_XXXXXXXXXXXXXX_XXXX_D.OSV
```

To test with pure angular central projection (zero baseline displacement, perfectly straight lines):
```bash
OSV_STITCH_RADIUS=0.0 ./run_shotcut.sh /path/to/CAM_XXXXXXXXXXXXXX_XXXX_D.OSV
```

### Standalone Command-Line Stitcher
Extract and stitch a single full-resolution (3840x1920) frame at a given timestamp:
```bash
./build/osv_stitch /path/to/video.OSV <seek_seconds> <output.ppm>

# Example:
OSV_STITCH_RADIUS=0.0 ./build/osv_stitch CAM_20261114162424_0023_D.OSV 1.0 /tmp/frame_r0.ppm
OSV_STITCH_RADIUS=2.2 ./build/osv_stitch CAM_20261114162424_0023_D.OSV 1.0 /tmp/frame_r2.ppm
```

### Test with Melt & Bigsh0t Rectilinear Filter
```bash
MLT_REPOSITORY=build/plugins MLT_DATA=build/share/mlt-7 FREI0R_PATH=build/plugins/frei0r \
melt osv:/path/to/video.OSV \
  -filter frei0r.bigsh0t_eq_to_rect yaw=0.25 pitch=0.35 fov=0.25 \
  -consumer avformat:/tmp/preview.mp4 vcodec=libx264 crf=18
```

---

## 4. Key Files
- `mlt-plugin/producer_osv.cpp`: MLT producer plugin with companion LRF proxy support and audio track alignment.
- `src/osv_vulkan.cpp`: Vulkan compute pipeline, DP seam solver, and CPU Kannala-Brandt projection.
- `shaders/osv_stitch.comp`: Universal Vulkan compute shader for 360 equirectangular stitching.
- `shaders/osv_stitch.cu`: CUDA equivalent kernel.
- `run_shotcut.sh`: Environment wrapper for launching Shotcut.

---

## 5. Next Steps for Investigation
1. **User Verification**: Validate test images on the target machine with varying camera angles and moving subjects.
2. **Default Radius Evaluation**: Decide whether `stitchRadius = 0.0` should become the hard default for Shotcut preview or remain configurable via producer properties / environment variables.
3. **Fine-Tuning Seam Routing on Fast Crossing Motion**: Test additional scenes where subjects rapidly cross between front and rear lenses under non-uniform illumination.
