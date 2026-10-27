# Inverse kinematics for the SO-101 (step 9a)

Design note for `so101.kinematics.ik`. Builds on `docs/fk_math.md`. Math uses `$…$` / `$$…$$` (renders in
GitHub, VS Code's Markdown preview, Obsidian, Typora).

## 1. Problem

Given a target for the tool point, find joint angles $q \in \mathbb{R}^5$ (shoulder_pan … wrist_roll, radians)
that put it there, within the joint limits and close to where the arm is now.

- **Tool point (TCP):** the centre of the fixed jaw's tip sphere (`tcp="fixed_jaw_tip"`), the point the step 8c
  table touches measured. Its frame has $z$ pointing out along the fingers. Configurable: Phase 2 swaps in
  the fitted target frame.
- **Targets:**
  - position $p^* \in \mathbb{R}^3$ (always);
  - optionally an **approach direction** $a^* \in S^2$: the world direction the fingers should point;
  - optionally **held joints**, e.g. wrist_roll fixed at a value.
- **Why not a full 6-DoF pose:** with 5 joints the arm cannot reach every position *and* orientation.
  Position (3) + finger direction (2 independent constraints) = 5 matches the joint count. Roll about the
  finger axis follows from the solution, or is fixed by holding wrist_roll.

## 2. Forward model and errors

From the calibrated FK ($T_{\text{tool}}(q) = [R(q) \mid p(q)]$, `docs/fk_math.md`), the finger direction is
$a(q) = R(q)\,\hat z$ (the TCP frame's $z$-axis).

Position error and direction error:

$$
e_p = p^* - p(q),
\qquad
e_o = a(q) \times a^* ,
\qquad
\lVert e_o \rVert = \sin\angle(a, a^*).
$$

$e_o$ points along the axis about which the fingers would have to rotate to line up with $a^*$; its length
is the sine of the misalignment. Stacked, with a weight $w$ (metres per radian) on the orientation rows:

$$
e = \begin{bmatrix} e_p \\ w\, e_o \end{bmatrix} \in \mathbb{R}^6,
\qquad w = 0.05\ \text{m/rad}\ \ (1^\circ \approx 0.9\ \text{mm}).
$$

Without an approach target only $e_p$ is used (3 rows).

## 3. Jacobian

The geometric Jacobian (`fk.geometric_jacobian`) relates joint rates to tool-point velocity and angular
velocity. For revolute joint $i$ with axis $z_i$ through point $o_i$ (base frame):

$$
J = \begin{bmatrix} J_v \\ J_\omega \end{bmatrix},
\qquad
J_{v,i} = z_i \times (p - o_i),
\qquad
J_{\omega,i} = z_i .
$$

The finger direction changes as $\dot a = \omega \times a$, so to first order the rows $w J_\omega$ act on
the direction error. Rotation about $a$ itself does not change $a$; in the damped solve below that direction
simply gets no push. The stacked Jacobian is

$$
\tilde J = \begin{bmatrix} J_v \\ w\, J_\omega \end{bmatrix},
$$

restricted to the **active** columns (held joints removed).

## 4. Damped least-squares step

The linearised problem is $\tilde J\,\Delta q \approx e$. The damped least-squares solution
(Nakamura & Hanafusa 1986; Wampler 1986) is

$$
\boxed{\;\Delta q = \tilde J^\top \left(\tilde J \tilde J^\top + \lambda^2 I\right)^{-1} e\;}
$$

which minimises $\lVert \tilde J \Delta q - e \rVert^2 + \lambda^2 \lVert \Delta q \rVert^2$.

- $\lambda = 0$: the plain least-squares (pseudoinverse) step, the smallest $\Delta q$ that would remove the error
  if the arm were linear. Near singular poses (arm stretched, axes aligned) $\tilde J \tilde J^\top$ is nearly
  singular and this step blows up.
- $\lambda > 0$: trades a little accuracy per step for a bounded, safe step. In terms of the singular values
  $\sigma_k$ of $\tilde J$, each direction's gain is $\sigma_k / (\sigma_k^2 + \lambda^2)$ instead of
  $1/\sigma_k$: large for well-conditioned directions, damped to 0 as $\sigma_k \to 0$.

## 5. Iteration with adaptive damping (Levenberg–Marquardt style)

Starting from $q_0$ (the current pose), repeat:

1. Compute $e(q)$, cost $c = \lVert e \rVert^2$. **Stop (success)** if $\lVert e_p \rVert < 0.5$ mm and
   $\angle(a, a^*) < 1^\circ$.
2. Compute $\Delta q$ (section 4) with the current $\lambda$.
3. **Step cap:** if $\lVert \Delta q \rVert > 0.2$ rad, scale it down to 0.2 rad.
4. **Limits:** $q' = \mathrm{clip}(q + \Delta q,\ \ell,\ u)$ (section 6).
5. **Accept or retry:**
   - if $c(q') < c(q)$: accept $q \leftarrow q'$, and make the next step bolder: $\lambda \leftarrow \max(\lambda/2,\ 10^{-6})$;
   - else reject, and make the next step more cautious: $\lambda \leftarrow 4\lambda$.
     If $\lambda > 10^3$, **stop: stalled** (unreachable, or blocked by limits).
6. After 300 iterations without success, **stop: max_iter**.

Starting value $\lambda = 10^{-3}$. The result reports success, the remaining position and direction error,
the iteration count and the reason.

## 6. Joint limits

Limits come from the software limits in ticks (`config/arm_follower.yaml`, `limits_ticks`), converted with
the step 7 mapping (`ik.limits_rad`, sign-aware). Every iterate is clipped to

$$
\ell_i = \min(\ell_i^{\text{soft}},\ q_{0,i}),
\qquad
u_i = \max(u_i^{\text{soft}},\ q_{0,i}),
$$

i.e. the **outside-start rule** (same as SafeArm): a joint that starts outside its software limit (the elbow at
rest) may stay there or move back in, never further out.

## 7. Behaviour worth knowing

- **Stays near the current pose.** Small, improving steps from $q_0$ find the nearby solution (no elbow flip to
  the other branch). In a Cartesian move each waypoint is warm-started from the previous solution, so joints
  change smoothly: one or two iterations, < 5 ms per waypoint.
- **Unreachable targets are reported, not forced.** Example (pinned by a test): fingers pointing straight down
  at 8 cm above the table needs wrist_flex beyond its soft limit ($+72.7^\circ$ = swept max $77^\circ$ minus the
  50-tick margin). The solver stalls with wrist_flex at the limit and reports it; tilting the approach $25^\circ$
  off vertical is reachable.
- **Not a planner.** IK answers "which joint angles", not "is the way there safe". Paths are checked separately:
  joint limits, the table collision check (`kinematics.collision`), and SafeArm's runtime checks.

## 8. Why our own IK

LeRobot's SO-101 IK uses the placo library on its own URDF (`RobotKinematics`). We need IK on **our** calibrated
FK and tick mapping, so that model and real arm agree (to ~1.5–2 mm at table level after step 8c) and Phase 2's
fitted parameters apply directly. Damped least squares on `fk.py` is the standard choice.

## 9. Verification (tests/test_ik.py)

| Test | Checks |
|---|---|
| tip vs MuJoCo | `fixed_jaw_tip` FK equals MuJoCo's tip-sphere position |
| tip frame | its $z$ points out along the fingers |
| position round trip | 200 random poses within limits: solve back from a nearby start; ≤ 2% fail, limits respected |
| position + approach round trip | 100 random poses, targets from their own position and finger direction: < 0.5 mm, < 1° |
| unreachable | a target 1 m away is reported as a failure, not forced |
| held joint | wrist_roll stays at its held value |
| outside start | a joint starting past its limit never moves further out |
| wrist limit | fingers-straight-down near the table fails at the wrist_flex limit; 25° tilt succeeds |
| speed | warm-started waypoints < 5 ms |
