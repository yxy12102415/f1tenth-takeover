# Route-aligned opponent prediction and wall margin

Requested assumption: the opponent normally follows the frozen reference route. This remains generic obstacle avoidance, with no designated target identity or ID-loss stop.

The prior applies only to fresh observed components with >=4 hits, extent 12–65 cm, centre within 35 cm of the route, lateral estimated speed <=0.08 m/s and longitudinal estimated speed >=-0.08 m/s. Mapped surfaces, hazards, missed/occluded supports, stale measurements and clear lateral/reversing motion retain the unconstrained model. The prior does not label every obstacle stationary.

The box follows reference arc length, preserving its measured lateral offset and measured lateral velocity. Its observed support and hidden-body padding are retained and rotated with the local route direction. Longitudinal uncertainty remains; lateral uncertainty becomes 0.02*min(age,0.5)+0.02*future metres in addition to measurement/body padding. Forecasts beyond the route endpoint fall back to the unconstrained model.

`compact_config.json`: `route_aligned_opponent_prior_v1=true`, `wall_margin_m=0.03`.
`runtime_assets.json`: the same 3 cm wall margin, used by swept-body planning, control and emergency coast checks. The 3.1 cm body-sampling cover is retained. The fast circle speed preview retains its own additional conservative envelope. Map resolution is 5 cm; this is a model margin, not a centimetre-accurate physical distance switch.

Desired opponent body gap remains 10 cm; hard predicted-body gap remains 8 cm; running command floor remains 0.6 m/s, maximum 2 m/s, terminal zero. The zero-current coast model remains 0.4 m/s² plus 100 ms transport and 250 ms steering centring.

Captured occupancy replay: forecast world-Y width at 1.55 s shrank from 78.5 cm to 49.8 cm. A fresh replan of the latest field scene, using the same new wall margin, reduced reference offset magnitude from 27.7 cm without the prior to 20.9 cm with it. These are offline plans, not driven paths. The last recorded high-speed coast state still rejects at the new 3 cm margin; this change alone does not resolve the longitudinal-response/coast-model issue.

Tests: `test_route_prior.py` checks measured support, direct collision rejection and model fallback; `test_wall_margin.py` checks margin algebra and continued wall rejection. Current physical scope is stationary-opponent PID and dynamic MPC, tested separately. PID and dynamic MPC now pass the stationary-opponent production-node closed-loop fixture. No full moving overtake or new physical success is claimed.

## MPC integration and timing

The optimizer now receives a future obstacle-aware tube at each 50 ms stage, using the same measured occupancies and bounds as the independent guard. It selects a nearby free reference if necessary, within 8 cm of the nominal local path, requiring a 2 cm tube before maximizing available radius. The captured rejected-reference regression finds a 2.5 cm correction with a 2.03 cm tube; its centre and edges pass the full-body guard. This is a local MPC reference correction, not a replacement global route.

Cold and warm seeds are exact generated-model rollouts from measured state. The nonlinear refinement loop can use up to 60 ms inside the unchanged 75 ms hard deadline. Nonlinear residual tolerance remains 0.001; failed cold prearm solves remain rejected.

Exact support/stamp/velocity route projection caching and per-stage forecast-box reuse reduce repeated calculations. No geometry is quantized and no source stamps are renewed; freshness gates run before cache lookup. Cached and fresh-process forecasts match in 27 replay checks.

Current C++ stationary-opponent stress fixture: both controllers complete the route. Dynamic MPC maximum accepted compute time 32.3 ms; maximum control plus guard time 60.1 ms. Its minimum software body separation is 22.1 cm, PID 20.7 cm. These are synthetic-plant results, not physical clearances.

Entrypoints from the workspace: `./381_2_0_dynamic_avoidance.sh pid run` and `./381_2_0_dynamic_avoidance.sh dynamic_mpc run`. Test one controller per reset; first physical scope is a stationary opponent with its ESC off. Each tracker reuses its own exact-fingerprint offline evidence; real zero/localization qualification is reused only while fresh.

Final readiness: exact-fingerprint offline window checks passed for PID and dynamic MPC. A preceding PID full-stack check hit the unchanged 75 ms guard at 81 ms; that failure is retained in `reports/compact_fix/pid_both_ready_offline.log`. The successful checks are `pid_both_ready_offline_retry.log` and `dynamic_mpc_both_ready_offline_retry.log`. Occasional scheduling stops remain possible. No physical motion was started during this preparation.
