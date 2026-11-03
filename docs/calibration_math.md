# Kinematic calibration and calibrated commanding for the SO-101

Design note for `so101.analysis.kincal` (the fit), `so101.kinematics.calibrated` (loading a fitted model) and
`so101.motion.calibrated_ik` (commanding with it). Builds on `fk_math.md` (the parameterized chain),
`ik_math.md` (the solver) and `pnp_math.md` (the camera measurements). The math uses `$…$` / `$$…$$`.

## 1. Problem

The nominal model (the MuJoCo/CAD geometry plus the by-eye tick→angle mapping) puts the gripper ~8 mm from where
it is told to go, mostly low. Two questions:

1. **Knowing where it is:** given the encoder readings, predict where the gripper actually is. This is a
   model-fitting problem: find the parameters that make forward kinematics agree with the camera.
2. **Putting it where you want:** given a target, choose servo goals so the gripper actually lands there. Three
   ways of doing this were compared on the arm:

| Method | Goal sent to the servos |
|---|---|
| `nominal` | IK on the nominal geometry; goal = the IK angles |
| `cal` | IK on the calibrated geometry, then feedforward for gravity bending and servo droop |
| `cal_corr` | as `cal`, then one correction from the servos' own encoder reading after settling |

Everything is measured at the tag's centre and expressed relative to the iPad (the fixed reference in every photo).

## 2. Measurement model

For photo $i$ with encoder angles $q_i$, the predicted pose of the tag in the camera frame is

$$
\boxed{\;\hat T_{\text{cam},\text{tag},i} = R_i \; X \; F(\tilde q_i;\ \theta) \; Y\;}
$$

| Symbol | What it is | Fitted? |
|---|---|---|
| $R_i = T_{\text{cam},\text{ipad},i}$ | the iPad's pose, **measured** by PnP in the same photo | no (measured) |
| $X = T_{\text{ipad},\text{base}}$ | where the arm's base sits in the iPad frame (one per run) | yes |
| $F(\cdot;\theta)$ | parameterized FK to the gripper body (`fk_math.md` §3) | through $\theta$ |
| $Y = T_{\text{gripper},\text{tag}}$ | where the tag is stuck on the gripper | yes |
| $\tilde q_i$ | the "effective" joint angles: encoder angles plus modelled gravity bending (§3) | through $c$ |

Because $R_i$ is measured per photo, camera motion cancels exactly (`pnp_math.md` §6). $X$ is fitted per run so a
small bump of the iPad or base between sessions is absorbed rather than read as arm error.

## 3. Parameters

**Geometry** $\theta$ (all zero = nominal):

- Joint zero offsets $\delta$ on shoulder_lift and elbow_flex: $q \to q + \delta$.
- Link placement twists $\xi_{1..4} \in \mathbb{R}^6$ (small rotation + translation of each joint's frame, applied
  with the $SE(3)$ exponential; `fk_math.md` §3).

**Gravity compliance** $c \in \mathbb{R}^3$ (rad per N·m) on the three pitch joints (shoulder_lift, elbow_flex,
wrist_flex). The servos read their own shaft angle, but the link and gearbox past the encoder bend under load,
so the real joint angle is the reading plus a deflection proportional to the static gravity torque:

$$
\tilde q_j = q_j + c_j\,\tau_j(q), \qquad j \in \{\text{lift},\ \text{elbow},\ \text{wrist\_flex}\},
$$

with $\tau(q)$ the gravity torque on each joint at that pose from the MuJoCo model's link masses
(`kincal.gravity_torques`, i.e. MuJoCo's `qfrc_bias` at zero velocity).

**What one fixed camera and one tag can't separate**, so they are left out of the fit:

- the pan zero offset and the first link's twist: both are just a rotation or shift of $X$;
- the wrist_flex and wrist_roll offsets and the tool twist: with wrist_roll mostly held fixed, they fold into $Y$.

`kincal.identifiability` checks the rest: the SVD of the residual Jacobian with respect to $\theta$, after removing
what $X$ and $Y$ can absorb. Singular values near zero flag combinations the poses don't determine.

### Model variants

Six variants were fitted and compared; each is saved separately in `config/kinematics/follower_<variant>.json`:

| Variant | Fits |
|---|---|
| `nominal` | $X$, $Y$ only (nominal geometry) |
| `offsets` | + lift/elbow zero offsets |
| `links` | + link twists $\xi_{1..4}$ |
| `full` | + wrist offsets and a tag-size scale |
| **`stiff_lin`** | `links` + linear compliance $c$ (lift, elbow, wrist) |
| `stiff_thr` | `links` + shoulder compliance only above a 0.75 N·m knee |

`stiff_lin` was best on held-out and independent test poses and is the one used for commanding. Note that even
`nominal` has its $X$ and $Y$ fitted to the camera: without them there is no way to express a target in the
arm's frame at all.

## 4. Objective

The residuals are the tag's four **corner reprojection errors in pixels**, not PnP positions. For shot $i$ and
corner $k$, with tag corners $P_k$ (`pnp_math.md` §3) and observed refined corners $u_{ik}$:

$$
r_{ik} = \pi\!\left(P_k;\ \hat T_{\text{cam},\text{tag},i}\right) - u_{ik} \in \mathbb{R}^2 .
$$

Fitting in pixels weights each photo by what it actually measured: the weak depth of a single tag gets the
small weight it deserves, with no need to model the PnP covariance.

Weak Gaussian priors keep directions the data can't see at zero instead of drifting:
$\sigma = 3^\circ$ on offsets, $2^\circ$ / 2 mm on twists, $0.05$ rad/N·m on compliances. The full problem is

$$
\min_{X,\,Y,\,\theta,\,c}\ \sum_{i,k} \lVert r_{ik} \rVert^2 \;+\; \sum_m \left(\frac{\theta_m}{\sigma_m}\right)^2 ,
$$

solved with Levenberg–Marquardt (`scipy.optimize.least_squares`, `method="lm"`, Jacobian-scaled steps).

**Initialization.** $X$ and $Y$ start from the hand-eye fit (`so101.analysis.handeye`): the same corner-pixel
problem with nominal geometry, with $Y$'s rotation found by trying a grid of axis-aligned candidates and keeping
the most consistent. $\theta$ and $c$ start at zero.

## 5. Validation

The fit's own residual flatters any model, so accuracy is judged on data the fit never saw:

- **Cross-validation by pose:** 221 shots of 92 distinct poses, split into 5 folds **by pose** (repeat shots of
  one pose stay together, so a pose never appears on both sides). Each fold is predicted by a model fitted on
  the other four.
- **Independent test:** 17 poses from a separate run, never used in any fit.
- **Error:** predicted tag centre minus the PnP-measured one, rotated into the camera's axes so lateral and depth
  separate (depth carries ~0.1 mm of PnP noise per shot).

| | Lateral RMS | 3D RMS |
|---|---|---|
| `nominal`, test | 2.02 mm | 2.18 mm |
| `stiff_lin`, cross-validation | 0.74 mm | 1.07 mm |
| `stiff_lin`, test | 0.79 mm | 1.14 mm |

These are *knowing where it is* numbers, and smaller than nominal's ~8 mm commanding error because even `nominal`
has $X$ and $Y$ fitted here.

**Fitted values** (`stiff_lin`): zero offsets lift −0.18°, elbow +0.26°; compliances lift −2.1°/N·m,
elbow −0.3°/N·m, wrist_flex 13°/N·m. The wrist value is suspect as a physical number: the wrist torque barely
varies (0.05–0.13 N·m), so it can't be told apart from a geometric error that follows the wrist's tilt.

### Re-anchoring

If the iPad or the base moves, the arm parameters stay valid but $X$ doesn't. `kincal.refit_transform` refits
**only** $X$ on a fresh set of poses, with $\theta$, $c$ and $Y$ held fixed, and writes it to
`config/kinematics/anchors/<run>.json` (the variant file is never modified). Anchor on poses where the model fits
well: a first anchor on poorly fitted poses carried ~2.6 mm of offset into every command.

## 6. Commanding: from a target to servo goals

A target is a tag-centre position $p^*$ relative to the iPad (mm). For each method:

### `nominal`

1. Into the base frame: $p^*_{\text{base}} = X^{-1} p^*$ (with the nominal variant's $X$).
2. Position-only IK (`ik_math.md`, damped least squares) for the tag point, i.e. tool transform $Y$, nominal
   geometry, wrist_roll held at its value in a seed pose that saw the tag.
3. Goal ticks = the IK angles converted through the tick mapping. No compensation.

### `cal`

1. Same as `nominal` steps 1–2, but with `stiff_lin`'s $X$, $Y$ and $\theta$. This gives $q_{\text{IK}}$, the
   joint angles the arm must **actually** have.
2. **Undo the gravity bend.** The encoders should read $q_{\text{enc}}$ such that, after bending,
   the arm is at $q_{\text{IK}}$:

$$
q_{\text{enc}} + c \odot \tau(q_{\text{enc}}) = q_{\text{IK}} ,
$$

   solved by fixed-point iteration (5 rounds, starting from $q_{\text{enc}} = q_{\text{IK}}$):
   $q_{\text{enc}} \leftarrow q_{\text{IK}} - c \odot \tau(q_{\text{enc}})$. It converges quickly because the
   deflections are small (≲ 1°).
3. **Pre-compensate servo droop.** A position-controlled servo holding a load settles short of its goal, because
   its proportional controller needs an error to produce torque. Fitted from 266 arrivals in earlier runs
   (`scripts/camera/servo_droop_fit.py`, `config/kinematics/servo_droop.json`), per pitch joint:

$$
\text{reading} - \text{goal} \approx a_j + b_j\,\tau_j \quad \text{(ticks)},
\qquad
\text{goal}_j = \text{reading}^*_j - \left(a_j + b_j\,\tau_j\right),
$$

   with $\text{reading}^* = q_{\text{enc}}$ in ticks. Fitted values:

| Joint | $a$ (ticks) | $b$ (ticks per N·m) | Residual sd (ticks) |
|---|---|---|---|
| shoulder_lift | +1.2 | −12.5 | 5.2 |
| elbow_flex | −13.2 | −51.3 | 6.4 |
| wrist_flex | −6.9 | −77.4 | 2.8 |

   The residuals are mostly the servo deadband (±1–2 ticks) plus approach direction, which this model can't
   predict.

This droop is **visible to the encoder** (the reading differs from the goal). It is a different effect from the
compliance $c$, which is **invisible** to the encoder (the reading is right, the link bends past it).

### `cal_corr`

Same goals as `cal`, plus the desired encoder reading $\text{reading}^*$ stored on the waypoint
(`desired_ticks`). After the arm settles (3 s), `capture_tour.py`:

1. reads the encoders and computes the offset $\Delta = \text{reading}^* - \text{reading}$ per joint;
2. if every $|\Delta_j| \le 30$ ticks, moves the goal by $\Delta$ (a smooth minimum-jerk nudge at 50 ticks/s),
   otherwise skips the correction and logs it;
3. settles again, then takes the photos.

One iteration only, and using nothing but the servos' own readings: no camera feedback. It removes whatever the
droop model got wrong for this particular arrival (approach direction, deadband, friction), so it targets the
landing **scatter** rather than the average error.

## 7. Results (13 new targets, each commanded all three ways)

| Method | 3D RMS | Lateral RMS | Worst | Common offset (x / y / z) | Lateral scatter |
|---|---|---|---|---|---|
| `nominal` | 8.18 mm | 8.07 mm | 10.95 mm | +0.8 / +7.5 / −0.8 | 2.91 mm |
| `cal` | 2.10 mm | 1.84 mm | 4.42 mm | −0.7 / −0.5 / −0.3 | 1.65 mm |
| `cal_corr` | **1.88 mm** | **1.53 mm** | **2.99 mm** | −0.3 / +1.3 / −0.7 | **0.75 mm** |

Run `20261031-135444`, anchored on `20261031-103300`; mm at the tag, camera axes (y down). Calibrated commanding
beat nominal on all 13 targets. Nominal's error is mostly a common ~7.5 mm sag (y is down in the camera frame);
the calibrated model removes it, and the encoder correction halves the remaining scatter.

## 8. Behaviour worth knowing

- **Commanding is worse than knowing.** The model predicts the tag to ~0.8 mm lateral from encoder readings, but
  commands land at ~1.5 mm, because the servos don't land exactly where told (deadband, droop residuals).
- **Far reach is worse.** At far-forward reach, where the shoulder carries the most torque (above ~0.75 N·m),
  prediction errors grow to 2.5–5 mm. The linear compliance underfits there.
- **Residuals are mostly depth.** The weak axis of a single tag; the fit can't fully separate depth noise from
  real arm error along the line of sight.
- **Tag, not tool tip.** Everything here is at the tag. The tool tip needs its own offset (pivot calibration).
- **Compliance vs geometry.** With gravity torques that vary little across poses, a compliance term and a
  pose-dependent geometric error look the same. Loading the arm with known payloads would separate them.

## 9. Code map

| What | Where |
|---|---|
| Measurement model, variants, fit, k-fold, identifiability, re-anchor | `src/so101/analysis/kincal.py` |
| Hand-eye initialization | `src/so101/analysis/handeye.py` |
| Fitting and saving variants | `scripts/camera/kinematic_fit.py` |
| Loading a variant (+ anchor) | `src/so101/kinematics/calibrated.py` |
| Re-anchoring | `scripts/camera/reanchor.py` |
| Servo droop fit | `scripts/camera/servo_droop_fit.py` → `config/kinematics/servo_droop.json` |
| Target → goals (IK, bend, droop) | `src/so101/motion/calibrated_ik.py` |
| Encoder correction on the arm | `scripts/motion/capture_tour.py` (`desired_ticks`, `--max-correct-ticks`) |
| Planning the command test | `scripts/sim/plan_step10b_tour.py` |
| Landing-error analysis | `scripts/camera/command_accuracy.py` |
