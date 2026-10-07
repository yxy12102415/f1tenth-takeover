# Build and source scope

## Supported in this export

`tools/build_native.sh` rebuilds the v8 native JSON probe from C++ and generated C, and regenerates the map-distance assets from the included map and route. `tools/test_native.sh` runs the same independent-plant two-obstacle/actuator-response model checks used during development. These are non-actuating Linux tools; no ROS nodes or device access are involved.

Dependencies:

- C++14 compiler, a C compiler, Eigen3 (`libeigen3-dev`), jsoncpp (`libjsoncpp-dev`).
- Python 3 with NumPy, SciPy, Pillow and PyYAML; see `tools/requirements-offline.txt`.
- acados **v0.4.0**, upstream commit `43354449192aa51557abda37d1bc17d907077354`, including its matching BLASFEO/HPIPM submodules, headers and shared libraries.

Build acados according to its upstream CMake instructions and set `ACADOS_INSTALL_DIR` to the installation prefix. Generated solver C is included, so CasADi and the template renderer are unnecessary for this build. Regenerating the solver requires the acados template tools and the reviewed CasADi 3.6.7 generation environment; `generate_joint_solver.py` records the equations and constraints.

## Real vehicle runtime

`real_vehicle/` preserves the original workspace-relative layout for the v8 MPC and v2 PID source snapshots, supervisor/health/window sources, baseline localization sources, integration adapters, modified VESC driver source, selected configuration, map and route. The generated C solver is portable; installed aarch64 binaries are intentionally omitted.

The frozen physical launch scripts depend on the commissioned ROS 2 Foxy/Jetson CUDA environment, built overlays, drivers, GPU range_libc extension and matching runtime manifests. This export does **not** claim a fresh machine can be made field-ready with the native build command. Existing `freeze.py`, historical validation scripts and physical launch scripts remain provenance/porting sources; some intentionally reference historical traces, prior releases or external installed overlays not shipped here. The supported standalone test entry is `tools/test_native.sh`.

Existing installed vehicle operation stays at `~/f1tenth_real_migration_ws_20260829`; exporting or cleaning build caches does not replace its binaries or bypass admission checks.

## Source verification

`docs/SOURCE_SNAPSHOT.json` records hashes of the original source files. The map YAML uses a relative image path, and stale summary fields in two config metadata JSON files are reconciled with the v8 result. Original and exported hashes are recorded; vehicle-control source remains unchanged. `docs/EXPORT_VALIDATION.json` records checks performed on the exported build. Raw field traces remain on the vehicle; selected original result summaries and trajectory CSV/PNGs are under `results/`.

## Attribution

The existing Duke hardware/software material is preserved with its notices in `README_LEGACY.md`, `Pages/`, `Code/`, and other historical directories. New native modules retain their source/package notices; third-party acados/BLASFEO/HPIPM notices are in each release's `licenses/`. No inference of a new project-wide license should be made.
