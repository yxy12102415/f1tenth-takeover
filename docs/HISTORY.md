# Historical materials

The current tree focuses on the Fitzpatrick real-vehicle export and the independent risk-aware following reference. The original Duke F1TENTH material remains recoverable from Git history.

- [Pre-cleanup repository](https://github.com/yxy12102415/f1tenth-takeover/tree/84828275e0817388cc956ae9eedde86ec8534c1d)
- [Original project overview and attribution](https://github.com/yxy12102415/f1tenth-takeover/tree/84828275e0817388cc956ae9eedde86ec8534c1d/README_LEGACY.md)
- [Hardware/software documentation](https://github.com/yxy12102415/f1tenth-takeover/tree/84828275e0817388cc956ae9eedde86ec8534c1d/Pages)
- [Old maps](https://github.com/yxy12102415/f1tenth-takeover/tree/84828275e0817388cc956ae9eedde86ec8534c1d/map) and [trajectories](https://github.com/yxy12102415/f1tenth-takeover/tree/84828275e0817388cc956ae9eedde86ec8534c1d/Trajectory)
- [Historical experiments and workspaces](https://github.com/yxy12102415/f1tenth-takeover/tree/84828275e0817388cc956ae9eedde86ec8534c1d/Code)

The original overview attributes the documentation to the F1 Tenth team at Duke University and states that Duke holds copyright in its instructions and accompanying materials. Its upstream project is F1TENTH from the University of Pennsylvania. These attributions and the individual notices in retained source packages continue to apply; this cleanup does not create a new project-wide license.

Removed current-tree content includes large photos/videos, hardware design attachments, obsolete maps/trajectories, vision/raceline experiments and duplicate ROS workspaces. `Code/sim_ws/` and its documented `Code/f1tenth_stack/` reference are retained. The latest `real_vehicle/`, model tests and field evidence are unchanged.

Exact removed paths and object IDs are in [REPOSITORY_CLEANUP_PHASE2.json](REPOSITORY_CLEANUP_PHASE2.json). This is a normal deletion commit, not a history rewrite. Full Git clones still download historical large objects.
