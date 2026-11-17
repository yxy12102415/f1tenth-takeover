# 381 2.0 vehicle checkpoint — 2026-11-17

The current C++ dynamic MPC completed the original open 381 2.0 route around
a **stationary** shell-covered opponent. The operator confirmed no contact.
Return oscillation remains; continuous moving-opponent overtaking is not qualified.

| Latest physical trial | Result |
|---|---|
| Vehicle record | `20261117_114358-closed-frenet-ground-attempt` |
| Route completion | Yes; final endpoint distance 9.07 cm |
| Contact | None, operator confirmed |
| Return segment peak cross-track error | 18.31 cm, map X 3.0–4.5 m |
| Return segment wheel-speed estimate | 1.06–1.26 m/s |
| Steering | Reached ±0.269 rad; return tuning still needed |
| Maximum controller compute time | 31.30 ms in this run |
| Selected local-path offset | −0.399 m; one initial selection |

[Trajectory, local plan and original reference](../results/381_20261117/trajectory_overview.png)
· [Return detail](../results/381_20261117/return_detail.png)
· [Steering and tracking error](../results/381_20261117/steering_error.png)
· [Recorded result](../results/381_20261117/recorded_result.json)
· [Measured trajectory CSV](../results/381_20261117/actual_trajectory.csv).
Localization is estimated rather than external ground truth.

## Current stack and limits

GPU particle-filter localization → LiDAR occupied-support memory → C++ lead
template/route association → Frenet local avoidance → C++ PID/PI or joint
dynamic MPC → independent body/obstacle/map checks → actuator command guard.

The source snapshot is
[`ground_381_2_0_dynamic_avoidance_20261115_v2`](../real_vehicle/candidate_cpp_gpu_20260909/ground_381_2_0_dynamic_avoidance_20261115_v2).
It includes the subsequent November 16–17 fixes despite its directory date.

- Running command floor: 0.6 m/s; maximum: 2.0 m/s; positive acceleration: 1.0 m/s².
  Startup ramps from zero; terminal STOP is zero. These are limits, not constant speeds.
- Desired body gap: 15 cm, a soft preference; hard gap: 8 cm; seed reserve: 7 cm.
  Wall margin: 3 cm. The configured preference is not a measured minimum clearance.
- Controller hard deadline: 200 ms. MPC soft budget: 45 ms with three consecutive
  misses allowed; refinement budget: 85 ms. Source freshness: 0.5 s.
- Dynamic bicycle model, motor and servo response states; joint speed/steering
  control, 24 stages at 0.05 s.
- Scene assumption: **only the associated lead vehicle may move**. Other geometry
  remains occupied and fixed. This is a controlled-course assumption, not a general
  solution for pedestrians or multiple moving vehicles.

## Geometric template matching

Production implementation:
[`lead_identity.hpp`](../real_vehicle/candidate_cpp_gpu_20260909/ground_381_2_0_dynamic_avoidance_20261115_v2/src/cpp_frenet/lead_identity.hpp).
The Python helper is a read-only prototype, not the production controller.

1. **Candidate gate.** At acquisition, use observed, repeatedly supported components
   0.6–3.3 m ahead along the reference and within |d| ≤ 0.65 m. Check plausible size
   against the operator's outer dimensions: length 32 cm, width 26 cm. Try individual
   components and compatible neighboring pairs, allowing up to 20 cm between them.
2. **Visible contour.** Extract measured returns inside candidate bounds plus 2 cm.
   For recognition, keep the largest connected support using an 8.5 cm neighbor
   radius, require six points, and subtract its centroid. Decimate dense inputs
   toward roughly 48 points. Do not rescale the shape.
3. **Template search.** Compare with all eight captured views, each at 16 rotations
   spaced 22.5° apart: 128 shape comparisons per candidate.
4. **Shape score.** Find nearest-point distances in both directions. In each
   direction retain the smallest 80%; divide their combined distance sum by their
   combined retained count. This trimmed symmetric Chamfer-style score must be
   ≤ 6 cm. It is an average shape mismatch, not clearance or identification probability.
5. **Continuity.** Acquire after at least three observations spanning at least
   0.15 s. Reject similarly plausible disjoint candidates. After acquisition add
   predicted-position mismatch to the shape cost and associate across changing
   cluster IDs. Predict along route s while retaining the measured lateral d;
   lateral motion can revoke the route-aligned prior.

The latest stationary physical trial matched the rear template at approximately
1.0–1.1 cm shape error. That does not establish position accuracy or recognition
accuracy in other environments. Recognition filtering never removes unmatched
returns from collision checking. Identity loss alone does not command STOP;
unresolved geometry near the lead remains conservative.

## Evidence and reproduction scope

- [Native matching regression](../real_vehicle/candidate_cpp_gpu_20260909/ground_381_2_0_dynamic_avoidance_20261115_v2/review/lead_identity_regression.json):
  eight same-capture views, changing IDs, ambiguous acquisition, temporary loss,
  crossing motion, unchanged collision geometry, recorded terminal-scene replay.
- [Current PID and dMPC software result](../real_vehicle/candidate_cpp_gpu_20260909/ground_381_2_0_dynamic_avoidance_20261115_v2/review/both_controller_regression.json):
  both actual C++ node pairs completed the stationary-target synthetic closed loop.
  This does not establish current PID physical acceptance or moving-target acceptance.
- [Selected failed-scene records](../real_vehicle/candidate_cpp_gpu_20260909/ground_381_2_0_dynamic_avoidance_20261115_v2/review):
  compute, false-motion, planning and contact-scene failures are retained alongside
  successful regressions. Historical report fields retain their original dates/scope.
- [Eight-angle captures](../real_vehicle/analysis/lead_shell_multiview_20261117/README.md):
  1,608 target scans plus 201 empty-background scans, compressed without loss.
  These are 2D scanner-height contours, not a complete 3D vehicle model.

From repository root, this check compiles only a stdin/stdout C++ probe and
replays compressed captures. It starts no ROS node or actuator driver:

```bash
python3 tools/check_381_lead_templates.py
```

Requires a C++14 compiler, Eigen3/JsonCpp headers, Python NumPy and SciPy.
The full vehicle launcher additionally needs the commissioned Jetson ROS/driver/
GPU/acados installation and regenerated assets. Build products, installation
trees and large run logs are intentionally excluded. Vehicle source/binary hashes
are historical evidence, not binaries to install on a fresh clone.
Some original replay scripts reference those retained local logs and are not
standalone repository tests. Original release README/status files are preserved
as historical working notes; **this dated checkpoint describes the current state**.

Remaining work: reduce return oscillation, repeat stationary trials, then qualify
moving-target association and full dynamic passing before declaring overtaking complete.
