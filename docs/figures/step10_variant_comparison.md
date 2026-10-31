Variants (relative to the ipad; fitted on 20261030-190841_capture_follower, 20261030-162649_capture_follower, 20261030-154331_capture_follower, 20261030-170704_capture_follower; test 20261030-202915_capture_follower):

| variant | n θ | fit px | CV lat | CV 3D | CV 3D max | CV inner 3D (|τ|≤0.75) | CV far 3D | test lat | test 3D | test max | test sub-mm lat |
|---|---|---|---|---|---|---|---|---|---|---|---|
| nominal | 0 | 12.5 | 1.57 | 1.98 | 6.09 | 1.75 | 3.80 | 2.02 | 2.18 | 3.56 | 4/17 |
| offsets | 2 | 11.3 | 1.43 | 2.06 | 6.60 | 1.77 | 4.19 | 1.81 | 2.80 | 3.98 | 3/17 |
| links | 26 | 6.9 | 0.89 | 1.66 | 5.41 | 1.47 | 3.19 | 1.01 | 1.34 | 2.23 | 10/17 |
| full | 29 | 6.9 | 0.89 | 1.67 | 5.44 | 1.48 | 3.21 | 1.02 | 1.35 | 2.26 | 10/17 |
| stiff_lin | 29 | 6.2 | 0.74 | 1.07 | 4.94 | 0.85 | 2.48 | 0.79 | 1.14 | 2.32 | 13/17 |
| stiff_thr | 27 | 6.7 | 0.91 | 1.79 | 5.19 | 1.62 | 3.19 | 0.92 | 1.28 | 2.39 | 12/17 |

- **nominal**: CAD geometry; only the iPad<-base and gripper<-tag transforms fitted
- **offsets**: + zero offsets of shoulder_lift, elbow_flex
- **links**: + zero offsets (lift, elbow) + placement corrections of links 1-4 (6 each; priors 2°, 2 mm)
- **full**: links + wrist_flex and wrist_roll offsets + tag scale
- **stiff_lin**: links + linear gravity compliance on lift, elbow, wrist (Δθ = c·τ, model torques)
- **stiff_thr**: links + shoulder_lift compliance above 0.75 N·m only (threshold chosen from these results)
