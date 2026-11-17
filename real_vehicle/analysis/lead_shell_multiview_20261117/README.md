# Shell-covered lead: eight LiDAR views

November 17 collection: empty background first, then eight approximate 45° views,
201 scans (~5 s) each. F1 remained fixed. Capture started no actuator nodes.
The operator later supplied outer tyre dimensions: **length 32 cm, width 26 cm**.

Each `scans.jsonl.gz` is a lossless compressed original. To use the original
inspection scripts, decompress each to `scans.jsonl` in the same directory.
The portable repository check reads gzip directly:

```bash
python3 tools/check_381_lead_templates.py
```

Individual figures:
[rear](inspection/rear.png) · [rear-left](inspection/rear_left.png) ·
[left](inspection/left_side.png) · [front-left](inspection/front_left.png) ·
[front](inspection/front.png) · [front-right](inspection/front_right.png) ·
[right](inspection/right_side.png) · [rear-right](inspection/rear_right.png).

`inspection/summary.json` and `collection_manifest.json` describe collection-time
state, before recognizer integration. Their `runtime_changed:false` and
`recognizer_installed:false` fields describe that step, not the later C++ deployment.
Foreground ROI may contain isolated returns: its maximum span is not vehicle size.
The matcher uses centered principal supports embedded in the current config.
Held-out frames from the same capture do not establish independent recognition accuracy.

See [current vehicle checkpoint](../../../docs/VEHICLE_PROGRESS_20261117.md).
