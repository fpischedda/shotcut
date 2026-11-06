# System Architecture and Implementation Plan

## 1. System Overview

To provide seamless playback and editing of DJI Osmo 360 files in Shotcut without requiring offline pre-conversion, the system implements a native **MLT Producer Service** backed by a **Vulkan / CUDA Compute Shader Stitching Engine**.

```
                +------------------------------------------------------+
                |             DJI Osmo 360 Video (.OSV)                |
                +------------------------------------------------------+
                                     |
               +---------------------+---------------------+
               |                     |                     |
         [Stream 0:0]          [Stream 0:1]          [Stream 0:3]
         Front Fisheye         Rear Fisheye          Timed Telemetry
          (3840x3840)           (3840x3840)         (dvtm_oq101.proto)
               |                     |                     |
               v                     v                     v
         [avformat #0]         [avformat #1]      [OsvMetadataParser]
               |                     |                     |
               |                     |             fx, fy, cx, cy, k, q
               v                     v                     v
         +------------------------------------------------------+
         |            mlt_producer "osv" Service                |
         +------------------------------------------------------+
                                     |
                                     v
         +------------------------------------------------------+
         |       GPU Stitcher (Vulkan 1.4 / CUDA Compute)       |
         |  - Kannala-Brandt Unwarping                          |
         |  - Relative Lens Orientation Alignment               |
         |  - Inverse Gyro Horizon Stabilization                |
         |  - Overlap Seam Feathering (195° FOV)                |
         +------------------------------------------------------+
                                     |
                                     v
         +------------------------------------------------------+
         |       Equirectangular 360° Canvas (3840x1920)        |
         +------------------------------------------------------+
                                     |
                                     v
         +------------------------------------------------------+
         |           Shotcut bigsh0t Filter Suite               |
         |  - 360: Equirectangular to Rectilinear               |
         |  - Keyframeable Yaw, Pitch, Roll, FOV                |
         +------------------------------------------------------+
```

---

## 2. Mathematical Model for Dual Fisheye Stitching

### A. Spherical Coordinate Mapping
For each pixel $(x, y)$ in the target equirectangular output $(W \times H)$:
1. Normalized coordinates: $u = (x + 0.5) / W, \quad v = (y + 0.5) / H$
2. Spherical angles:
   $$\lambda = (u - 0.5) \cdot 2\pi \quad (\text{longitude}, [-\pi, \pi])$$
   $$\phi = (0.5 - v) \cdot \pi \quad (\text{latitude}, [-\pi/2, \pi/2])$$
3. 3D unit ray on the unit sphere:
   $$\mathbf{P} = \begin{pmatrix} \cos(\phi) \sin(\lambda) \\ \sin(\phi) \\ \cos(\phi) \cos(\lambda) \end{pmatrix}$$

### B. Gyro Stabilization & Lens Ray Projection
1. Rotate ray by the conjugate (inverse) of the frame's gyro orientation quaternion:
   $$\mathbf{P}' = \mathbf{q}_{\text{gyro}}^* \cdot \mathbf{P} \cdot \mathbf{q}_{\text{gyro}}$$
2. For each lens $i \in \{1, 2\}$, transform the ray into the lens sensor coordinate space:
   $$\mathbf{P}_i = \mathbf{q}_{\text{lens}, i}^* \cdot \mathbf{P}' \cdot \mathbf{q}_{\text{lens}, i}$$
3. Angle with the optical axis ($Z$-axis):
   $$\theta_i = \arctan\left(\frac{\sqrt{P_{i, x}^2 + P_{i, y}^2}}{P_{i, z}}\right)$$

### C. Kannala-Brandt Fisheye Projection
Using the factory polynomial coefficients $k_1, k_2, k_3, k_4$:
$$\theta_{d, i} = \theta_i \left(1 + k_{1, i} \theta_i^2 + k_{2, i} \theta_i^4 + k_{3, i} \theta_i^6 + k_{4, i} \theta_i^8\right)$$
$$\mathbf{p}_{\text{pixel}, i} = \begin{pmatrix} c_{x, i} + f_{x, i} \cdot \theta_{d, i} \cdot \frac{P_{i, x}}{\|\mathbf{P}_{i, xy}\|} \\ c_{y, i} + f_{y, i} \cdot \theta_{d, i} \cdot \frac{P_{i, y}}{\|\mathbf{P}_{i, xy}\|} \end{pmatrix}$$

### D. Seam Feathering & Blending
Given a lens field of view $\theta_{\text{max}} = 195^\circ / 2 = 97.5^\circ$ and overlap blend window $\delta = 15^\circ$:
$$w_i = 1.0 - \text{smoothstep}(\theta_{\text{max}} - \delta, \theta_{\text{max}} + \delta, \theta_i)$$
$$C_{\text{final}} = \frac{w_1 C_1 + w_2 C_2}{w_1 + w_2}$$

---

## 3. Implementation Roadmap

- **Phase 1 (Complete)**: Telemetry demuxer, Protobuf calibration extractor, Vulkan 1.4 SPIR-V compute shader, CUDA compute kernel, and standalone verification CLI (`osv_stitch`).
- **Phase 2 (Complete)**: Native MLT producer plugin (`libmltosv.so`), synchronized dual-stream `avformat` sub-producers, and `loader.dict` automatic file routing.
- **Phase 3 (Next Steps)**:
  - Per-frame dynamic gyro orientation vector extraction from binary telemetry packets (feeding `PushConstants.gyro_quat` on every frame).
  - QML preset buttons inside Shotcut for 16:9, 9:16 vertical, Tiny Planet, and Selfie perspectives.
  - Zero-wait proxy switcher using companion `.LRF` files during high-speed timeline scrubbing.
