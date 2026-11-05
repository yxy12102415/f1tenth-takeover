# Why Frenet + spline planning + dynamic MPC

This is the engineering rationale for the current implementation, not an
experimental claim that it outperforms every alternative. Dynamic passing is
still being validated in simulation; real-v8 acceptance covers static avoidance.

## Different layers, not three competing algorithms

The reference route defines a tangent and a normal. Frenet coordinates express
position as `p(s,d) = r(s) + d n(s)`: `s` is progress, `d` is lateral offset.
They simplify route boundaries, opponent gaps, passing sides and rejoining.
They do not by themselves predict opponents, generate controls or ensure safety.

A spline describes a smooth lateral path `d(s)`. Our optional `upstream_sqp`
backend solves for piecewise cubic coefficients subject to lateral corridor and
curvature conditions. Recovery uses boundary-matched polynomial transitions.
Spline smoothness does not establish dynamic feasibility: Cartesian footprint,
timed opponent-body and MPC checks remain necessary.

Two distinct limits must not be confused. Geometric steerability requires
`abs(kappa(s)) <= tan(delta_max)/wheelbase`. Driving that geometry requires
`v(s)^2 * abs(kappa(s)) <= a_lateral_budget`, with enough distance and actuator
response to reach that speed. Applying the current speed's curvature budget to
every future spline knot can reject a turn that is executable after braking.
Conversely, relaxing geometric constraints does not permit driving a tight
curve at the current speed: timed retiming and the MPC must still reject that.

The time parameterization supplies `s(t)` and therefore `d(s(t))`, speed and
timestamped poses. The important coupling is `d_dot = d_s * s_dot`: increasing
speed on the same spline increases lateral motion and changes collision times.
Reference-route speed also differs from physical speed on an offset bend:
`v_path = s_dot * sqrt((1 - kappa_ref*d)^2 + d_s^2)`.

The dynamic MPC rolls out vehicle/actuator states and optimizes acceleration and
steering jointly. It follows the timestamped reference with input, rate and map
constraints, and its predicted bodies are checked against the opponent forecast.
Only the first control is applied before estimating and optimizing again.

## Alternatives and practical tradeoffs

| Approach | Useful strengths | Costs or limitations for this task | Current choice |
| --- | --- | --- | --- |
| Hybrid A* / state lattice | Handles nonholonomic geometry, alternative routes, large heading changes and reversing | A geometric search still needs smoothing, time allocation and moving-obstacle prediction; adding time/velocity grows the search | Attractive for route changes or tight recovery, less direct for repeated passes along one known track |
| DWA | Small velocity-space rollout, straightforward local reactive avoidance | A basic short-horizon formulation does not supply the entire pass/rejoin decision; Ackermann dynamics and actuator states need explicit modeling | Not the first choice for this route-relative passing task |
| TEB | Optimizes poses and time intervals; can model car-like constraints and multiple topologies | Would require adapting opponent prediction, dynamic vehicle/actuator behavior and existing interfaces; no measured advantage here yet | Valid alternative, especially for mobile-robot navigation |
| MPPI | Samples control sequences; accommodates nonlinear models and flexible trajectory costs | Sampling distribution/count, compute, cost scaling and collision handling require validation; a finite sampled penalty is not a hard safety guarantee | Worth a future matched comparison, does not automatically repair a conservative reference or stale prediction |
| Joint NMPC / MPCC | Can optimize progress, lateral motion and inputs together, reducing planner/controller timing mismatch | Nonconvex passing-side choices, initialization and model/solver tuning move into a larger optimization | The most relevant later alternative if the layered interface remains the bottleneck |
| Pure Pursuit / Stanley / heading PID | Low cost, transparent, good tracking baselines | Need an external path/speed planner and separate treatment of dynamics, actuator lag and coupled input limits | Retain as baselines rather than assume they cannot work |
| RL / neural control | Can learn residual effects or complex policies from data | Requires suitable training coverage and independent validation under changing conditions; current hardware data do not establish this coverage | Defer a controller replacement; model/residual learning can be tested separately |

## Why keep the current stack

The task has a known smooth closed route and mainly forward motion. Frenet
directly expresses passing and rejoining. Splines allow continuous lateral
transitions and corridor adjustment. The existing C++ dynamic MPC already has
a physical static-avoidance deployment, actuator modeling and joint inputs.
Sharing it between simulation and hardware avoids introducing another controller
before the passing planner is validated.

The tradeoff is an interface that must be maintained carefully: a collision-free
spline with a wrong speed schedule can be unsafe, and an executable first MPC
horizon does not certify the whole maneuver. Fixed-route coordinates also become
ambiguous near self-intersections and poorly conditioned as `1-kappa_ref*d`
approaches zero. Map projection, same-side commitment, forecast age, retiming
and a checked recovery policy are essential parts of this choice.

Our use of Predictive Spliner is selective: the upstream numerical lateral
optimizer is adapted. We do not claim the complete upstream GP opponent-learning
pipeline. The current opponent estimate and route-assisted prediction are ours.

## Primary implementation references

- [ForzaETH Predictive Spliner](https://github.com/ForzaETH/predictive-spliner):
  opponent GP prediction and spatial/temporal overtaking planner.
- [ETH MPCC](https://github.com/alexliniger/MPCC): contour/lag error and progress
  optimization with vehicle/track/input constraints. Its README explicitly says
  obstacle avoidance is supported in the Matlab implementation, not its C++ port.
- [Nav2 MPPI documentation](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/):
  sampled predictive control, motion models and cost configuration.
- [TEB upstream](https://github.com/rst-tu-dortmund/teb_local_planner): timed elastic
  bands and local trajectory optimization.

These references support descriptions of those implementations. Our choice and
integration-cost assessments are project-specific judgments. We have not run
a controlled cross-method benchmark and cannot rank their performance here.
