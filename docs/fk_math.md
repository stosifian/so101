# Parameterized forward kinematics for the SO-101 (step 8a)

Design note for `so101.kinematics`. The math uses `$…$` / `$$…$$` (renders in GitHub, VS Code's Markdown
preview, Obsidian, Typora).

## 1. Notation

- Joint angles $q = (q_1, \dots, q_5)$ in radians: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex,
  wrist_roll, obtained from servo ticks by the step 7 mapping
  $q_i = s_i\,(\text{ticks}_i - z_i)\,\tfrac{2\pi}{4096}$ (sign $s_i$, zero $z_i$; `config/calibration/follower_mapping.yaml`).
- The gripper joint $q_6$ moves only the moving jaw. The tool point (`gripperframe`) sits on the fixed jaw, so
  $q_6$ is **not** part of the tool chain (it matters for collision geometry only).
- Homogeneous transform of rotation $R \in SO(3)$ and translation $p \in \mathbb{R}^3$:
$$
T = \begin{bmatrix} R & p \\ 0^\top & 1 \end{bmatrix} \in SE(3).
$$
- Rotation about the local $z$-axis (every SO-101 hinge has `axis="0 0 1"` in its own body frame):
$$
R_z(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta & 0 \\ \sin\theta & \cos\theta & 0 \\ 0 & 0 & 1 \end{bmatrix},
\qquad
\mathrm{Rot}_z(\theta) = \begin{bmatrix} R_z(\theta) & 0 \\ 0^\top & 1 \end{bmatrix}.
$$

## 2. Nominal chain (exactly what the MuJoCo model encodes)

Each body $i$ is placed relative to its parent by a fixed transform $T_i^{\text{fix}}$ (the MJCF `pos`, `quat`),
and then rotated by its joint:

$$
T_{i-1,i}(q_i) = T_i^{\text{fix}}\;\mathrm{Rot}_z(q_i),
\qquad
T_i^{\text{fix}} = \begin{bmatrix} R(\mathbf{u}_i) & p_i \\ 0^\top & 1 \end{bmatrix},
$$

where $R(\mathbf{u})$ is the rotation of the unit quaternion $\mathbf{u} = (w, x, y, z)$ (MuJoCo order, normalized):

$$
R(\mathbf{u}) =
\begin{bmatrix}
1 - 2(y^2 + z^2) & 2(xy - wz) & 2(xz + wy) \\
2(xy + wz) & 1 - 2(x^2 + z^2) & 2(yz - wx) \\
2(xz - wy) & 2(yz + wx) & 1 - 2(x^2 + y^2)
\end{bmatrix}.
$$

The tool pose in the base frame is the product along the chain, times the fixed tool site:

$$
\boxed{\;T_{\text{tool}}(q) = \prod_{i=1}^{5} \Big( T_i^{\text{fix}}\,\mathrm{Rot}_z(q_i) \Big)\; T_{\text{site}}\;}
$$

Nominal values (vendored model `models/so101_menagerie/so101.xml`, metres; quaternions before normalization):

| $i$ | body (joint) | $p_i$ | $\mathbf{u}_i = (w,x,y,z)$ |
|---|---|---|---|
| 1 | shoulder (pan) | $(0.0388353,\ 0,\ 0.0624)$ | $(0,\ 0,\ -1,\ 0)$ |
| 2 | upper_arm (lift) | $(-0.0303992,\ -0.0182778,\ -0.0542)$ | $(1,\ -1,\ -1,\ -1)$ |
| 3 | lower_arm (elbow) | $(-0.11257,\ -0.028,\ 0)$ | $(1,\ 0,\ 0,\ 1)$ |
| 4 | wrist (wrist_flex) | $(-0.1349,\ 0.0052,\ 0)$ | $(1,\ 0,\ 0,\ -1)$ |
| 5 | gripper (wrist_roll) | $(0,\ -0.0611,\ 0.0181)$ | $(0.0172,\ -0.0172,\ 0.7069,\ 0.7069)$ |
| site | gripperframe | $(0.012,\ -0.000218,\ -0.098127)$ | $(1,\ 0,\ 1,\ 0)$ |

The base frame is the model's `base` body. The table is the plane $z = -2.4\,\text{mm}$ (underside of the base
plate, which sits flat on the table).

## 3. Parameterized chain (what Phase 2 fits)

Small geometric errors are modelled as twists applied with the $SE(3)$ exponential, local to each link
(the "local product-of-exponentials" form used in kinematic calibration):

$$
\boxed{\;
T_{i-1,i}(q_i;\theta) = T_i^{\text{fix}}\; e^{[\hat\xi_i]}\; \mathrm{Rot}_z(q_i + \delta_i),
\qquad
T_{\text{tool}}(q;\theta) = \prod_{i=1}^{5} T_{i-1,i}(q_i;\theta)\; T_{\text{site}}\; e^{[\hat\xi_t]}
\;}
$$

Parameters $\theta = \big(\delta_{1..5},\ \xi_{1..5},\ \xi_t\big)$, all **zero nominally** (then the parameterized FK equals the model):

- $\delta_i \in \mathbb{R}$: joint zero offset. Absorbs the residual error of the step 7 (by-eye) mapping.
- $\xi_i = (\omega_i, v_i) \in \mathbb{R}^6$: rotation $\omega_i$ and translation $v_i$ correction of link $i$'s fixed placement.
- $\xi_t \in \mathbb{R}^6$: tool (or camera-target) transform correction.

### The $SE(3)$ exponential

For $\xi = (\omega, v)$ with $\phi = \lVert \omega \rVert$, and $[\omega]_\times$ the skew-symmetric matrix of $\omega$:

$$
[\hat\xi] = \begin{bmatrix} [\omega]_\times & v \\ 0^\top & 0 \end{bmatrix},
\qquad
e^{[\hat\xi]} = \begin{bmatrix} e^{[\omega]_\times} & J(\omega)\,v \\ 0^\top & 1 \end{bmatrix},
$$

with Rodrigues' formula for the rotation and the left Jacobian of $SO(3)$ for the translation:

$$
e^{[\omega]_\times} = I + \frac{\sin\phi}{\phi}[\omega]_\times + \frac{1 - \cos\phi}{\phi^2}[\omega]_\times^2,
\qquad
J(\omega) = I + \frac{1 - \cos\phi}{\phi^2}[\omega]_\times + \frac{\phi - \sin\phi}{\phi^3}[\omega]_\times^2 .
$$

(For $\phi \to 0$: $e^{[\omega]_\times} \to I + [\omega]_\times$ and $J \to I$; the code uses Taylor terms below a small threshold.)

### Identifiability

Not all $5 + 30 + 6 = 41$ parameters can be identified from tool measurements. Examples:
- a translation of link $i$ **along its own joint axis** is indistinguishable from the same translation of link $i{+}1$;
- a rotation correction **about** the joint axis ($\omega_{i,z}$) duplicates the zero offset $\delta_i$.

Phase 2 selects a minimal identifiable subset (rank of the parameter Jacobian over the measured poses) and fixes
the rest at zero. The code exposes all of them.

## 4. Why this form and not DH or a space-frame PoE

**Denavit–Hartenberg** would need the frames re-derived from the CAD, and the SO-101 has three **parallel**
axes (lift, elbow, wrist_flex). With parallel axes, DH-based calibration is ill-conditioned: a tiny axis
misalignment produces large jumps in the DH parameters (Hayati's problem). Ruled out.

**Space-frame product of exponentials** is a valid alternative:

$$
T(q) = e^{[\hat S_1] q_1}\, e^{[\hat S_2] q_2} \cdots e^{[\hat S_5] q_5}\, M,
\qquad S_i = (\omega_i,\ v_i),\quad v_i = -\,\omega_i \times r_i ,
$$

with $S_i$ joint $i$'s screw axis in the **base frame at the home pose** ($q = 0$), $r_i$ a point on it, and
$M = T_{\text{tool}}(0)$. Each revolute screw has 4 independent numbers ($\lVert\omega_i\rVert = 1$,
$\omega_i \cdot v_i = 0$), so calibration has $5 \cdot 4 + 6 = 26$ parameters: close to minimal, continuous,
no singularities for parallel axes (Okamura & Park, 1996). That is its main strength.

Why the local form is the main implementation anyway:

| | local PoE (chosen) | space-frame PoE |
|---|---|---|
| joint zero offsets | explicit $\delta_i$, fed straight back into the step 7 tick mapping | not separately identifiable: absorbed into $M$ and later screws |
| meaning of a correction | attached to a physical link ("upper arm placement off 0.3 mm / 0.2°") | expressed in the base frame |
| updating the MuJoCo collision model | body by body, directly | needs conversion back to body frames |
| verification vs MuJoCo | one-to-one with the MJCF bodies | via the conversion below |
| parameter count | 41, over-parameterized: Phase 2 must select an identifiable subset | ~26, near-minimal by construction |
| parallel axes | fine | fine |

**They describe the same kinematics.** The space screws and home pose follow from the local chain at $q = 0$:

$$
S_i = \mathrm{Ad}_{T_{0,i}(0)}\,\hat{e}_z,\qquad \hat{e}_z = (0,0,1,\ 0,0,0),
\qquad
\mathrm{Ad}_T = \begin{bmatrix} R & 0 \\ [p]_\times R & R \end{bmatrix},
\qquad M = T_{\text{tool}}(0),
$$

where $T_{0,i}(0)$ is joint $i$'s frame at the home pose. That gives $\omega_i = R\,e_z$ and $v_i = p \times \omega_i$
(i.e. $-\omega_i \times p$). The code exports $S_{1..5}$ and $M$ and tests that both formulations give the same pose
(`so101.kinematics.fk.space_screws`, `fk_space`).

## 5. Jacobians

**Geometric Jacobian** (for IK, step 9). All joints are revolute. With $z_i$ the joint axis expressed in the base frame and
$o_i$ a point on it (both from the partial products up to joint $i$), and $p$ the tool position:

$$
J(q) = \begin{bmatrix} z_1 \times (p - o_1) & \cdots & z_5 \times (p - o_5) \\ z_1 & \cdots & z_5 \end{bmatrix}
\in \mathbb{R}^{6 \times 5},
\qquad
\begin{bmatrix} \dot p \\ \omega_{\text{tool}} \end{bmatrix} = J(q)\,\dot q .
$$

With 5 joints the arm cannot reach every 6-DoF pose. IK (damped least squares, step 9) weights position over orientation.

**Parameter Jacobian** (for the Phase 2 fit): $\partial T_{\text{tool}} / \partial \theta$, by central differences
at first, analytic later if speed matters.

## 6. Verification plan (step 8a)

With $\theta = 0$, for the three validation poses (run `20261027-103553_model_check_follower`) and 1000 random $q$
within the joint ranges:

$$
\lVert p_{\text{ours}} - p_{\text{MuJoCo}} \rVert < 0.1\ \text{mm},
\qquad
\angle\!\left(R_{\text{ours}}^\top R_{\text{MuJoCo}}\right) < 0.01^\circ .
$$

Plus: perturbing $\delta_i$ alone must equal shifting $q_i$; the geometric Jacobian must match finite differences.

## 7. Table-touch validation (step 8c)

Touch the gripper tip to $N \ge 4$ table points; each recording gives $p_k = T_{\text{tool}}(q_k)\,[0,0,0,1]^\top$.
If FK and mapping were perfect, every $p_{k,z} = -2.4$ mm (+ the tip's contact offset). The residuals
$r_k = p_{k,z} - z_{\text{table}}$ measure the by-eye calibration error at the tool, and a least-squares plane
through the $p_k$ shows whether the touched table is tilted relative to the base (base not flat, or table not flat).
