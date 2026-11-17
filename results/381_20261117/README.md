# 381 2.0 stationary-opponent field result

Run: `20261117_114358-closed-frenet-ground-attempt`, current C++ dynamic MPC.
Route completed; operator confirmed no contact. Return oscillation remains.
Full moving-target overtaking is not qualified by this run.

- `trajectory_overview.png`: original reference, selected local plan and measured trajectory.
- `return_detail.png`: late return and correction.
- `steering_error.png`: steering and tracking-error history.
- `actual_trajectory.csv`: measured trajectory exported from the recording.
- `recorded_result.json`: unedited vehicle result.
- `summary.json`: analysis; `contact_user_confirmed:false` means no contact, as
  explicitly stated in its `user_report` field.
- `provenance.json` and `raw_run_hashes.json`: original source paths and hashes.
- `export_source_hashes.json`: byte-identical source/evidence export hashes.

Large raw traces remain on the vehicle. The plots are derived from those records,
not generated illustrations. Original absolute paths in provenance and summary
identify the source installation; use files in this directory for portable viewing.

[Current progress and limitations](../../docs/VEHICLE_PROGRESS_20261117.md).
