# Compact overtaking v4

Status: bounded moving-lead ROS/plant validation and target-loss stop validation passed; no physical v4 test yet. Ground zero/localization must pass for the new fingerprint before motion.

## Geometry

Same F1 start and heading. Right offset 0.55 m (previously 0.60); entry 1.90 m, parallel section 2.15 m, return 1.70 m. Return begins at reference x=4.05 m (previously 3.80), finishes at 5.75 m; endpoint is 5.95 m (previously 5.85). Lead initial center is 1.80 +/-0.05 m (previously 1.60): approximately 1.38 m from the F1 LiDAR to the lead rear surface for the confirmed 30 cm lead.

The inherited command cap is 1.85 m/s with 0.15 m/s headroom under the 2.0 m/s cap; maximum acceleration remains 2 m/s², moving-command floor 0.5 m/s, start/end zero.

## Joint MPC

Shift the previous control sequence one sample, then roll it forward from the current measured state with the exact generated discrete model. Allow up to 8 RTI iterations under a 45 ms refinement budget. Acceptance still requires both equality and inequality residuals below 0.001, and an additional independently evaluated dynamics-defect check. Existing 45 ms soft/75 ms hard timing guards and all collision/freshness/L1 checks remain. Rejected solves now retain iteration/residual diagnostics.

The original field solver failure was not reproduced exactly in the faster offline replay; both verified old and new code can pass recorded inputs offline. Passing that replay alone is not evidence of physical resolution.

## Test scope and stop behavior

This candidate passes synthetic occupied-body prediction checks for a 0.5 m/s lead released after about 3 seconds, including a small lateral perturbation, and a stationary lead. Continuous-speed physical overtaking has not been commissioned; next-trial scope remains the 3-second lead-release protocol. The next attended test retains the two independent L1 handsets and 3-second lead-release protocol.

STOP still commands zero motor current and centered steering, which is coast rather than active braking. This revision does not claim to shorten physical stopping distance. Brake current/deceleration needs separate physical identification before an active braking mode is enabled. The operator's existing waiver of a full coast screening prerequisite remains recorded; no such screen is silently reinstated.

Reports: deployment/381_2_0/reports/compact_v4_design. Current physical commissioning status must be read from compact_config.json, not inferred from these software checks.

The changed geometry permits a rear-edge visibility gap up to 0.90 s (v3: 0.65 s), only under the existing rear-bearing/forward-center-clearance conditions. Forward loss remains 0.30 s; ordinary measurement freshness remains 0.50 s. A retained old observation is never timestamped as fresh. This addresses the extra time needed for full rear-body clearance on the revised return.

V4 recognizes an observed completed merge after return has begun only when absolute lateral error is below 0.03 m, absolute heading error below 0.08 rad, the lead was already longitudinally passed, and the entire current reachable lead front plus physical margin is behind the F1 rear. This replaces the v3 fixed nominal merge station for the attended following-car policy; current reachable-body and map sweeps remain checked. It never applies while alongside or unaligned.

Terminal-coast handoff: once a fresh controller status reports valid terminal_coast/route_complete with speed command exactly zero, planner may recognize COMPACT_FINISH near the endpoint (<=0.10 m, <=0.20 rad) at measured speed <=0.15 m/s after the lead was fully passed. It checks the map and retains zero-output mode instead of repeatedly extending an unseen following car into the stopped F1. This is not a standstill claim: controller completion still requires speed <=0.03 m/s for 0.2 s. Stale/invalid/nonterminal controller states cannot activate the handoff; ordinary driving-body checks remain.

Run after restoring the F1 start and moving the lead approximately 20 cm forward: `cd ~/f1tenth_real_migration_ws_20260829` then `./381_2_0_compact_overtake.sh run`. Matching offline checks are reused; this new release needs its own 5-second physical zero/localization check. Keep both L1 released until READY, then hold both; release lead L1 after about 3 seconds and keep F1 L1 held to its stop.
