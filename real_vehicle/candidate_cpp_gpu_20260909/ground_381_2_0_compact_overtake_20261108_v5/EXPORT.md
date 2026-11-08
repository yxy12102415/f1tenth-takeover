# v5 source checkpoint

This is the text-source snapshot of the on-car v5 release, exported on
2026-11-08. `provenance/source_sha256.json` records the original source bytes.
`provenance/on_vehicle_runtime_manifest.sha256` is historical evidence of the
installed release, **not a valid manifest for this source-only checkout**.

The installed v5 binaries reused v4 libraries by absolute RPATH. Binaries,
libraries, install trees, raw logs and cached runtime assets are deliberately
excluded. Rebuild from the retained generated C model on the target architecture;
do not copy or claim the installed release fingerprint for a new build.

From the repository root, with acados v0.4.0 installed, Eigen3, jsoncpp, a C/C++14
compiler and the offline Python requirements:

```bash
export ACADOS_INSTALL_DIR=/path/to/acados_install
bash deployment/381_2_0/build_v5_native.sh
```

This builds non-actuating native probes and re-exports the map/route cache.
It starts no ROS nodes or hardware drivers. The source snapshot also retains
the actual ROS package and on-car launch scripts; those require the original
Foxy/CUDA/driver workspace and a fresh build/freeze/verification before use on
another installation. Existing absolute driver paths are intentionally recorded
as on-car configuration, not advertised as portable launch instructions.

The map YAML image path in `real_vehicle/maps/381_2_0.yaml` is made relative.
The release source itself is retained byte-for-byte. Current geometry is in
`compact_config.json`; route/profile are under `real_vehicle/routes/`.
The earlier `prepare_compact_release.py` creates historical v1, not this v5.

Latest physical outcome: no reported contact, but `joint_mpc_solver_4` before
completion. See [the checkpoint](../../../docs/VEHICLE_PROGRESS_20261108.md).
