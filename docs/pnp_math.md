# Target poses from one photo (PnP) for the SO-101 metrology

Design note for `so101.analysis.pose` (with `so101.analysis.apriltag` and `so101.analysis.charuco` for the
image points). The math uses `$…$` / `$$…$$` (renders in GitHub, VS Code's Markdown preview, Obsidian, Typora).

## 1. Problem

Every photo contains two planar targets with known geometry: the 20 mm AprilTag on the gripper and the ChArUco
board on the iPad. For each one, find the rigid transform that maps the target's own frame into the camera frame,

$$
T_{\text{cam},x} = \begin{bmatrix} R & t \\ 0^\top & 1 \end{bmatrix} \in SE(3), \qquad x \in \{\text{tag},\ \text{ipad}\},
$$

from the target's corners seen in the image (Perspective-n-Point, PnP). The measurement is then the tag
**relative to the iPad**, $T_{\text{ipad},\text{tag}}$, in which the camera's own motion cancels (§6).

Frames: camera $x$ right, $y$ down, $z$ along the optical axis. Tag frame (OpenCV ArUco convention): origin at the
tag centre, $x$ right, $y$ up as printed, $z$ out of the tag. iPad frame: origin at the board's top-left square
corner, in mm. Rotations are passed around as rotation vectors $r$ ($R = \exp([r]_\times)$, `cv2.Rodrigues`).

## 2. Camera model

Pinhole projection plus Brown–Conrady distortion (OpenCV's 5-coefficient model, $k_1, k_2, p_1, p_2, k_3$).
A point $X$ in target coordinates goes to the camera frame, onto the normalized image plane, through the
distortion, and onto the sensor:

$$
X_c = R X + t, \qquad (x, y) = \left(\tfrac{X_{c,1}}{X_{c,3}},\ \tfrac{X_{c,2}}{X_{c,3}}\right), \qquad r^2 = x^2 + y^2,
$$

$$
\begin{aligned}
x_d &= x\,(1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + 2 p_1 x y + p_2 (r^2 + 2x^2), \\
y_d &= y\,(1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + p_1 (r^2 + 2y^2) + 2 p_2 x y,
\end{aligned}
\qquad
\begin{bmatrix} u \\ v \end{bmatrix} = \begin{bmatrix} f_x x_d + c_x \\ f_y y_d + c_y \end{bmatrix}.
$$

Write this as $u = \pi(X;\ r, t)$. The intrinsics $K = (f_x, f_y, c_x, c_y)$ and the distortion come from the iPad
calibration (`config/intrinsics_a7iv.json`: $f_x = 7007.4$, $f_y = 7006.2$, $c_x = 3527.5$, $c_y = 2319.3$ px,
0.166 px RMS over 41 views) and are **held fixed** in every PnP. They are the same for every photo of a run, so an
error in them biases the absolute pose but not its change between photos. They are valid only at the taped
focus, and only above $y \approx 3700$ px (the calibration views didn't cover the bottom ~21 % of the frame);
detections with a corner below that line are flagged.

## 3. Object and image points

**Tag** (`tag_object_points`). The four outer corners of the black border, side $L$ (nominal 20 mm), in OpenCV's
order (top-left, top-right, bottom-right, bottom-left), which `SOLVEPNP_IPPE_SQUARE` requires:

$$
X_1 = \left(-\tfrac{L}{2},\ \tfrac{L}{2},\ 0\right),\quad
X_2 = \left(\tfrac{L}{2},\ \tfrac{L}{2},\ 0\right),\quad
X_3 = \left(\tfrac{L}{2},\ -\tfrac{L}{2},\ 0\right),\quad
X_4 = \left(-\tfrac{L}{2},\ -\tfrac{L}{2},\ 0\right).
$$

Image points $u_{1..4}$: OpenCV's ArUco detector finds and decodes the tag; the corners are then re-measured by
the edge-line refinement (`apriltag.refine_corners`): about 40 sub-pixel edge crossings per side on the linear
(gamma 1) RAW image, a total-least-squares line per side, and adjacent lines intersected. That removes OpenCV's
~0.17 px inward bias, which would otherwise read as a ~0.2 % scale error, i.e. depth.

**iPad**. The ChArUco inner corners (up to 96), with object points on the board plane in mm:
corner $(i, j)$ sits at $\big(i\,s,\ j\,s,\ 0\big)$ with square size $s = (\text{square px}) \times (\text{display pitch})$.
The pitch is Apple's nominal 264 ppi (96.2 µm), so the iPad frame's absolute scale is nominal. Image points come
from OpenCV's `CharucoDetector` (checkerboard saddle points, ~0.05 px); `matchImagePoints` pairs them with their
object points by corner ID.

## 4. Solving: closed-form start, then least squares

The pose minimizes the reprojection error over the $n$ correspondences:

$$
\boxed{\;(r^*, t^*) = \arg\min_{r,\,t}\ \sum_{k=1}^{n} \big\lVert \pi(X_k;\ r, t) - u_k \big\rVert^2\;}
$$

`pose.solve` does it in three steps:

1. **Closed-form candidates (IPPE).** For a planar target, the map from the board plane to the image is a
   homography. IPPE (Infinitesimal Plane-based Pose Estimation, Collins & Bartoli 2014) recovers the pose from
   that homography and its local derivative in closed form. For a plane it returns **two** candidates, which
   differ by a "tilt flip" (the plane tilted towards or away from the camera by about the same angle). The
   tag uses `SOLVEPNP_IPPE_SQUARE` (the 4-corner special case); the iPad uses `SOLVEPNP_IPPE`. Both come from
   `cv2.solvePnPGeneric`, which returns all candidates.
2. **Refinement.** Each candidate is polished with Levenberg–Marquardt on the full model including distortion
   (`cv2.solvePnPRefineLM`, up to 100 iterations, tolerance $10^{-12}$).
3. **Choice.** The candidate with the lower RMS reprojection error wins. The ratio
   $a = \text{RMS}_{\text{best}} / \text{RMS}_{\text{second}}$ is kept as `ambiguity`: near 0 the flip is clearly
   resolved; near 1 both tilts fit equally well and the pose is unreliable.

For the iPad (dozens of points spread over a large board) the flip is never ambiguous. For a small tag seen
nearly face-on, the two candidates can come close, which is why both are refined and compared.

## 5. Uncertainty

Assume independent, isotropic corner noise of $\sigma$ px (0.05 px used; the floor test measured an effective
~0.025–0.03 px). With $J = \partial \pi / \partial (r, t) \in \mathbb{R}^{2n \times 6}$ at the solution
(from `cv2.projectPoints`), the first-order covariance of the pose is

$$
\Sigma_{r,t} = \sigma^2 \left(J^\top J\right)^{-1}.
$$

This is computed at the actual pose, so the tag's tilt is accounted for. It is also strongly anisotropic. At
distance $Z$ with focal length $f$, one pixel across the image is $Z/f$ in space ($\approx 93$ µm at 0.65 m), so
lateral position is good to about $\sigma Z / f$. Depth comes from the target's apparent size: a target of size
$L$ spans $fL/Z$ px, and moving its corners by $\sigma$ changes the inferred distance by about

$$
\sigma_Z \approx \frac{Z}{L} \cdot \frac{\sigma Z}{f},
$$

about $Z/L \approx 30$ times worse than lateral for the 20 mm tag. That ratio is why depth is the weak axis
(~100 µm per photo against 5–15 µm laterally) and why the full $6 \times 6$ covariance is kept rather than
per-axis variances: the ellipsoid lies along the line of sight and rotates with any change of frame. Tilt and
depth also trade against each other, so their errors are correlated.

`tests/test_pose.py` checks this prediction against a Monte Carlo of 400 noisy solves.

## 6. Relative pose: tag in the iPad frame

$$
\boxed{\;T_{\text{ipad},\text{tag}} = T_{\text{cam},\text{ipad}}^{-1}\; T_{\text{cam},\text{tag}}
\;=\; \begin{bmatrix} R_i^\top R_t & R_i^\top (t_t - t_i) \\ 0^\top & 1 \end{bmatrix}\;}
$$

**Why camera motion cancels.** If the camera moves by a rigid $M$, every camera-frame pose becomes
$M\,T_{\text{cam},x}$, and

$$
(M T_{\text{cam},\text{ipad}})^{-1} (M T_{\text{cam},\text{tag}}) = T_{\text{cam},\text{ipad}}^{-1} M^{-1} M\, T_{\text{cam},\text{tag}} = T_{\text{ipad},\text{tag}} .
$$

So tripod creep, table bumps and the camera settling between shots drop out exactly, as long as the iPad
itself stays put.

**Uncertainty** (`pose.relative`). The two detections are treated as independent, so with
$x = (r_t, t_t, r_i, t_i) \in \mathbb{R}^{12}$ and $C = \mathrm{diag}(\Sigma_{\text{tag}}, \Sigma_{\text{ipad}})$,

$$
\Sigma_{\text{rel}} = G\,C\,G^\top, \qquad G = \frac{\partial\,(\delta r,\ t_{\text{rel}})}{\partial x} \in \mathbb{R}^{6 \times 12},
$$

with $G$ by central differences ($\varepsilon = 10^{-6}$). The rotation is expressed as a small rotation
$\delta r$ in the relative pose's own axes, so its $\sigma$ reads directly as angles. In practice the iPad term is
tiny (many corners, large board) and the tag's depth dominates.

## 7. What scale errors do

The pose is in the units of the object points. If the tag's true size is $(1+\varepsilon_t)L$ and the iPad's
true pitch is $(1+\varepsilon_i)$ times nominal, each target's translation scales by its own factor:
$t_t \to (1+\varepsilon_t)\,t_t$, $t_i \to (1+\varepsilon_i)\,t_i$ (rotations are unaffected). The tag's position
relative to the iPad is then off by about $\varepsilon_t\,t_t - \varepsilon_i\,t_i$, mostly along the line of
sight, since both targets sit ~0.65 m away: about 0.65 mm per 0.1 % of mismatch. The error is nearly constant
across the workspace, so it cancels in differences between photos (repeatability, drift) and is absorbed by
the fitted iPad←base and tag←gripper transforms in the kinematic calibration. It matters only for absolute
positions, which is why none are quoted. A caliper reading of the printed 100 mm scale bar put the print scale
within about 0.05 %; the iPad pitch hasn't been measured.

## 8. Behaviour worth knowing

- **Tilt flip.** A single small tag nearly face-on can have two poses with almost equal reprojection error;
  check `ambiguity` before trusting a tag pose.
- **The reprojection RMS of the tag says little.** Four corners give 8 equations for 6 unknowns, so the tag's
  RMS is small almost regardless (~0.1 px). Precision is better judged from repeated photos or the predicted
  covariance.
- **Corner bias becomes depth bias.** Anything that moves all four corners inward or outward together (gamma,
  OpenCV's SUBPIX, blur on gamma-encoded images) changes the apparent size and reads as depth. Hence the
  linear-image edge refinement in §3.
- **Small rotations.** `cv2.Rodrigues` rounds rotations below ~$10^{-5}$ rad to zero, which breaks
  finite-difference Jacobians; `rotation_delta` uses SciPy instead.

## 9. Why our own code

LeRobot has no camera or fiducial pose code, so there was nothing to copy. The heavy lifting is OpenCV
(`solvePnPGeneric` with IPPE, `solvePnPRefineLM`, `projectPoints`, the ArUco and ChArUco detectors). What's ours
is the glue: the edge-line corner refinement, trying both IPPE candidates, the covariance and its propagation
into the relative pose, and the flags (ambiguity, uncalibrated image region).

## 10. Verification (`tests/test_pose.py`)

- **Exact tag pose:** project the tag at a known pose with no noise; the solve recovers it ($10^{-6}$ mm), and
  the flipped candidate fits clearly worse (ambiguity $< 10^{-3}$).
- **Tag covariance vs Monte Carlo:** 400 solves with 0.05 px corner noise; the observed spread matches
  $\sigma^2 (J^\top J)^{-1}$ within 20 %, and depth comes out more than 5× worse than lateral.
- **Camera motion cancels:** moving the camera rigidly leaves $T_{\text{ipad},\text{tag}}$ unchanged.
- **Relative covariance vs Monte Carlo:** the propagated $\Sigma_{\text{rel}}$ matches the spread of 400 noisy
  relative poses within 20 %.
- **iPad pose:** recovered within 0.05 mm, without and with 0.05 px corner noise.

Edge-line refinement is tested separately (`tests/test_apriltag.py`) on synthetic tags at several sizes and tilts
(refined corners match the truth; a gamma-encoded image biases edges inward). On a real photo, see `docs/figures/tag_detection_20261030-113927.png`.
