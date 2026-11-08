# Compact overtaking v2: attended first trial

Status: software validation in progress; no physical run yet.

- Same original map 381 2.0 start; reference endpoint extended from 5.6 to 5.85 m.
- Right offset 0.60 m: 1.8 m entry, 2.0 m parallel section, 1.8 m return.
- F1 command cap 1.85 m/s, inherited 2 m/s² acceleration limit; normal software rollout peak about 1.7 m/s.
- Lead: 1150 us, one observed ground speed about 0.48 m/s. Prediction bounds forward speed in [0, 0.55] m/s.
- Initial center interval: 1.60 ± 0.05 m ahead of the F1 base frame. LiDAR-to-rear surface is about 1.18 m with the 0.27 m sensor offset and 0.15 m lead half-length.
- Dimensions including foam confirmed by operator: F1 <=58×31 cm, lead <=30×18 cm.
- Two independent handsets: F1 B4, lead A3. F1 L1 cannot stop the lead.

## Changed trial scope

The operator explicitly requested no emergency-coast screening as a startup prerequisite. This trial does not require that screen. The existing physical stop remains zero motor current plus centered steering; no active brake has been added.

Normal whole-path map clearance and each actual MPC predicted-body sweep remain enforced while passing and merging. After longitudinal body clearance AND completed return (x>=5.6 m, lateral error<0.10 m, heading error<0.15 rad), the following car's current reachable region is still checked but hypothetical future rear-end collision of the parked F1 is no longer used to reject the completed overtake. The lead operator must release L1 after about 3 seconds as instructed. V2 admits the launch horizon, then uses fresh LiDAR and a receding prediction horizon. Unlike V1 it does not require the whole maneuver to avoid every possible lead speed through the endpoint before moving. Completion is not guaranteed: an unsafe prediction, stale input, lost target, or released L1 still stops the F1 and latches the session.

## First physical sequence

Keep both L1 buttons released during setup. After the F1 console reports ready, start the two cars together. Release the lead L1 after about 3 seconds, while continuing to hold F1 L1 until F1 completes the route; release both immediately if the path is blocked. This is a bounded first trial with a moving lead that subsequently stops, not a demonstrated continuous-speed full overtake.

A 15 cm forward adjustment of the lead was requested after the latest LiDAR placement check. The revised 1.60 m center gap must be confirmed by real LiDAR before motion.
