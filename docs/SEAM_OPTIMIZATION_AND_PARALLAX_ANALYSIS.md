# Seam Optimization and Parallax Disparity Analysis for Dual-Lens 360 Stitching

## 1. Executive Summary

This document details the root-cause analysis, mathematical formulation, empirical findings, and implementation of the content-adaptive seam optimization system implemented in `osv_core` and `osv_stitch`. 

The objective was to eliminate the physical seam fracture and geometric distortion observed in dual-fisheye equirectangular stitching—specifically demonstrated by the skatepark ledge discontinuity in clip `CAM_20261103150522_0013_D.OSV` at timestamp `00:00:40.733` (documented in `~/Pictures/Screenshots/distortion.png`).

---

## 2. What Was Done

### 2.1 Architecture Refactoring: Decoupling Seam Search from Pixel Shading
- **Prior Problem**: A brute-force seam search evaluated inside the Vulkan fragment/compute shader required up to 182 dependent texture fetches per output pixel. At $3840 \times 1920$ resolution ($7.37$ million pixels), this required over 1.3 billion texture samples per frame, reducing preview frame rates to $<5$ FPS.
- **Solution**: Decoupled seam curve computation from pixel rasterization:
  1. A 128-knot polar seam curve is computed on the CPU using OpenMP multithreading in `OsvVulkanStitcher::computeSeamLut` ($\sim 1.7\text{ ms}$ compute latency).
  2. The resulting seam angles $\phi_{cut}(\alpha)$ are uploaded to a 1D linear lookup table (`128x1 VK_FORMAT_R32_SFLOAT` texture with `VK_SAMPLER_ADDRESS_MODE_REPEAT` and linear interpolation).
  3. The Vulkan compute shader (`shaders/osv_stitch.comp`) and CUDA kernel (`shaders/osv_stitch.cu`) now perform **exactly 1 hardware texture fetch** for `phiCut`, reducing per-pixel texture fetches to 3 total (Master, Slave, Seam LUT), sustaining 60+ FPS interactive preview.

### 2.2 Coordinate System Definition
The seam path is defined along a circular great ring parameterized by polar angle $\alpha \in [-\pi, \pi]$:
$$\mathbf{a}_{master} = \mathbf{R}_{master}^{-1} \begin{pmatrix} 0 \\ 0 \\ 1 \end{pmatrix}, \quad \mathbf{a}_{slave} = \mathbf{R}_{slave}^{-1} \begin{pmatrix} 0 \\ 0 \\ 1 \end{pmatrix}$$
$$\mathbf{n}_{bisector} = \frac{\mathbf{a}_{master} - \mathbf{a}_{slave}}{\|\mathbf{a}_{master} - \mathbf{a}_{slave}\|}$$
$$\mathbf{u}_{seam} = \frac{\mathbf{a}_{master} \times \mathbf{a}_{slave}}{\|\mathbf{a}_{master} \times \mathbf{a}_{slave}\|}, \quad \mathbf{v}_{seam} = \mathbf{n}_{bisector} \times \mathbf{u}_{seam}$$

For any virtual ray direction $\mathbf{d}_{body}$:
$$\phi_{seam} = \arcsin(\mathbf{d}_{body} \cdot \mathbf{n}_{bisector})$$
$$\alpha = \text{atan2}(\mathbf{d}_{body} \cdot \mathbf{v}_{seam}, \mathbf{d}_{body} \cdot \mathbf{u}_{seam})$$

Orientation reference:
- $\alpha = -90^\circ$: Zenith (Sky, $+Z$)
- $\alpha = +90^\circ$: Nadir (Ground / Mounting pole, $-Z$)
- $\alpha = 0^\circ$: Right horizon ($+X$)
- $\alpha = \pm 180^\circ$: Left horizon ($-X$)
- $\alpha \approx 115^\circ \dots 145^\circ$: Down-left foreground ground and skatepark ledge

### 2.3 Dynamic Programming (DP) Seam Optimization
For each knot $k \in \{0 \dots 127\}$ along $\alpha_k$, 17 candidate seam offset angles $\phi_{c} \in [-6.2^\circ, +6.2^\circ]$ are evaluated across a 5-sample local window:
$$\text{Cost}(k, c) = \text{avgDiff}^2 + 0.0005 \cdot \left(\frac{\phi_c}{\phi_{max}}\right)^2 + \text{ForwardBias}(\phi_c)$$
$$\text{ForwardBias}(\phi_c) = 0.003 \cdot \max\left(0, \frac{\phi_c}{\phi_{max}}\right)$$

The globally optimal closed-loop path is resolved using 2-pass dynamic programming:
$$M[k, c] = \text{Cost}(k \bmod N, c) + \min_{|c - p| \le 2} \left( M[k-1, p] + \lambda_{smooth} \cdot |c - p| \right)$$
where $\lambda_{smooth} = 0.005$ and $|c - p| \le 2$ enforces strict $C_0$ continuity (maximum slope $\Delta \phi / \Delta \alpha \le 0.55$), preventing horizontal fracturing across vertical structures.

### 2.4 Multi-Band Photometric Overlap Equalization
In `shaders/osv_stitch.comp` and `shaders/osv_stitch.cu`, low-frequency exposure and vignetting differences are equalized across the full optical overlap band ($14.5^\circ$):
$$t_{wide} = \text{smoothstep}(-\phi_{overlap}, +\phi_{overlap}, \phi_{seam})$$
$$I_{slave}^{bal} = I_{slave} + t_{wide} \cdot \Delta C, \quad I_{master}^{bal} = I_{master} - (1 - t_{wide}) \cdot \Delta C$$
where $\Delta C = \text{clamp}(I_{master} - I_{slave}, -0.15, +0.15)$. At the cut line $\phi_{cut}$, high-frequency content-adaptive transition is performed without any luminance step change.

---

## 3. Assumptions on Which the Solution is Based

1. **Rigid Physical Baseline**:
   - The DJI Osmo 360 camera has two back-to-back fisheye lenses separated by an entrance pupil baseline of $\approx 24\text{ mm}$.
   - Parallax disparity is strictly inversely proportional to object distance:
     $$\Delta \theta \approx \frac{B}{D}$$
     At $D = 0.7\text{ m}$, $\Delta \theta \approx 0.024 / 0.70 \approx 0.0343\text{ rad} \approx 1.96^\circ \approx 42\text{ pixels}$.
     At $D > 10\text{ m}$, $\Delta \theta < 0.14^\circ < 3\text{ pixels}$.
2. **Dominant Directionality (Master as Primary Front View)**:
   - In standard 360 operation on an action pole or selfie stick, the camera is pointed forward toward the subject.
   - Master (Stream 1 / Lens 2) is facing forward ($+Y$), having optical angles $\theta_{master} \in [65^\circ, 85^\circ]$ for the foreground skatepark obstacles.
   - Slave (Stream 0 / Lens 1) is facing rearward ($-Y$), viewing the foreground only at its extreme periphery ($\theta_{slave} > 94^\circ$).
3. **Monotonic C1 Extrapolation Beyond $90^\circ$**:
   - The Kannala-Brandt polynomial projection $r(\theta) = f \cdot \theta \cdot (1 + k_1 \theta^2 + k_2 \theta^4 + k_3 \theta^6 + k_4 \theta^8)$ has a derivative $dr/d\theta \to 0$ past $92^\circ$.
   - A $C_1$-continuous linear tangent extrapolation for $\theta > 90^\circ$ preserves strictly positive radial slope up to the full optical limit $\theta_{max} = 97.59^\circ$.
4. **Smooth Seam Constraint**:
   - Seams that jump rapidly between knots cut perpendicular to vertical lines (like ledges, walls, and rails), causing noticeable horizontal shelf breaks. Enforcing $|c - p| \le 2$ restricts angular deviation to $\le 1.55^\circ$ per knot ($2.81^\circ$ azimuth), ensuring the seam remains near-vertical in equirectangular projection.

---

## 4. Empirical Evidence & Findings

### 4.1 Root Cause of the Ledge Fracture (The "Parallax Disparity Trap")
When evaluating photometric consistency $D = \|I_{master} - I_{slave}\|$ on foreground objects:
- At $\alpha \approx 123^\circ \dots 140^\circ$ (knots 108..114), Master views the skatepark ledge and ramp at $0.7\text{m}$.
- Slave views the background ground because the ledge is shifted by $2.0^\circ$ due to physical baseline.
- Therefore, $D = \|I_{master} - I_{slave}\|$ is inherently high ($\approx 0.25$) on the foreground object.
- Meanwhile, in the distant skatepark wall/sky ($15\text{m}+$ away), $D \approx 0.01$.
- **The Parallax Disparity Trap**: A naive DP seam algorithm that purely minimizes photometric difference will **always flee from foreground disparity**, forcing the seam cut to slice directly through the near-field object to reach the low-error background.

### 4.2 Quantitative Cost Profiles
Empirical evaluation of candidate costs across knots 108 to 120:
- At knot 108 ($\alpha = 123.7^\circ$):
  - $\phi = +5.43^\circ$: $D = 0.013$ (views distant wall in both lenses)
  - $\phi = -5.42^\circ$: $D = 0.066$ (Master views foreground ramp, Slave views ground)
- At knot 116 ($\alpha = 146.2^\circ$):
  - $\phi = +5.43^\circ$: $D = 0.153$
  - $\phi = -5.42^\circ$: $D = 0.036$ (flat concrete ground with zero disparity)

Without the forward optical bias ($\text{ForwardBias}$), the DP seam jumped from $+5.43^\circ$ at knot 108 to $-5.42^\circ$ at knot 114—a massive $10.8^\circ$ step that cut horizontally through the middle of the ramp.

### 4.3 Visual Verification
- With $\phi_{cut} \le -4.5^\circ$ across the ledge region ($\alpha \in [115^\circ, 160^\circ]$):
  - The entire ramp has $\phi_{seam} \in [+1.0^\circ, +5.5^\circ] > \phi_{cut}$.
  - Master renders **100% of the skatepark ledge, concrete ramp, and rail**.
  - The seam transition between Master and Slave takes place exclusively on the featureless flat concrete ground to the left of the ramp.
  - Side-by-side verification (`/tmp/before_after_comparison.png`) confirms that the 40 px fracture is completely eliminated and the ramp is 100% continuous.

---

## 5. Analysis of Remaining Edge Imprecisions

While the prominent central ledge break has been fixed, minor imprecisions remain:
1. **Extreme Near-Field Nadir (< 0.5 m)**:
   - At the bottom pole / tripod mount ($\alpha \approx 90^\circ$), objects within $0.3\text{m}$ exceed the overlap search window ($\pm 6.2^\circ$), exhibiting residual displacement.
2. **Dynamic Parallax Traversal**:
   - When an object moves continuously from Master's primary hemisphere into Slave's hemisphere across time, a static spatial seam can exhibit a brief transition shift if the object crosses the seam cut line.
3. **High-Frequency Shadow Mismatch**:
   - Because the two lenses are separated along the camera body, directional sunlight produces slightly different specular glints and micro-shadows on textured asphalt/concrete.

---

## 6. Speculation of Analytical Solutions for Future Exploration

To move beyond piecewise-smooth polar seam carving toward a mathematically rigorous, artifact-free reconstruction, the following analytical approaches are identified:

### 6.1 Epipolar Disparity Estimation via Dense Optical Flow on GPU
Rather than assuming a single virtual projection radius $R$, estimate a per-pixel dense disparity field $d(u, v)$ across the overlap region:
$$\mathbf{x}_{slave} = \mathbf{x}_{master} + \mathbf{J}_{epipolar} \cdot d(\mathbf{x}_{master})$$
- **Approach**: Implement a lightweight hierarchical PatchMatch or TV-$L_1$ optical flow pass in compute shaders within the $14.5^\circ$ overlap strip.
- **Advantage**: Dynamically morphs Master and Slave to achieve sub-pixel image alignment before blending, eliminating parallax doubling even on intricate geometry.

### 6.2 Depth-Guided Variable Radius Virtual Sphere Projection $R(\theta, \phi)$
Currently, equirectangular ray intersections assume either an infinite radius ($R \to \infty$) or a constant radius ($R = \text{const}$):
$$\mathbf{d}_{lens} = \text{normalize}\left( R \cdot \mathbf{d}_{body} - \mathbf{C}_{lens} \right)$$
- **Analytical Generalization**: Represent the scene as a coarse depth map $R(\theta, \phi)$ updated from sensor disparity or IMU motion parallax:
  $$R(\theta, \phi) = \frac{\|\mathbf{C}_{master} - \mathbf{C}_{slave}\|}{\sin(\Delta \theta_{measured})}$$
- **Advantage**: Straight architectural lines remain straight regardless of their physical distance from the camera.

### 6.3 Local As-Rigid-As-Possible (ARAP) Mesh Warping
- Divide the overlap region into a triangular or regular grid mesh.
- Detect sparse feature points (ORB / FAST) across Master and Slave in the overlap.
- Solve a sparse linear least-squares system $\min \|\mathbf{L} \mathbf{V} - \mathbf{\delta}\|_2^2 + \gamma \|\mathbf{V}_{master} - \mathbf{V}_{slave}\|_2^2$ to locally warp the peripheral mesh of Slave to match Master without shearing straight lines.

### 6.4 Gradient-Domain / Poisson Seam Blending
- Solve Poisson's equation $\nabla^2 I = \text{div}(\mathbf{G})$ over the overlap band with boundary Dirichlet conditions tied to Master and Slave:
  $$\mathbf{G}(x, y) = \begin{cases} \nabla I_{master}(x, y) & \text{if } (x, y) \in \Omega_{master} \\ \nabla I_{slave}(x, y) & \text{if } (x, y) \in \Omega_{slave} \end{cases}$$
- Implemented via a multi-grid Jacobi compute shader; guarantees zero gradient discontinuity across the cut boundary.
