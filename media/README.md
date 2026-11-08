# Demo media

## Simulation overtaking

- Video: [Simulation_Overtaking.mp4](Simulation_Overtaking.mp4)
- README preview: [Simulation_Overtaking_preview.gif](Simulation_Overtaking_preview.gif)
- Source: user-provided simulation screen recording, `dcb4fa1b3221eadb6209f4f414a8720b_raw.mp4` (approximately 171.08 s).
- Source SHA-256: `0a6ef7c3b034191acd12bfa2112c7e0b3d80f91cd5d203ba7b1266febd88304e`.
- Selected source interval: `00:01:31.000` to `00:01:49.000`, a single continuous excerpt without internal cuts or speed changes.
- Crop: `1272 x 914`, top-left corner `(28, 110)` in the original `1562 x 1072` recording. This keeps the RViz scene while excluding toolbars, the side panel, the desktop, and the notification.
- MP4: H.264, 24 fps, original-speed playback, no audio (the source has no audio track), fast-start enabled.
- GIF: video seconds 3-15, original-speed playback, 6 fps, 640 px wide, looping.

This is visual evidence of a simulation demonstration, not a physical-vehicle test or a quantitative success-rate evaluation. The recording is not associated with a verified source-code commit; no algorithm or performance claims are inferred from the video alone. The original recording remains unchanged outside the repository.

### Reproduce the edit

With FFmpeg installed and `SOURCE` set to the original recording path, run from the repository root:

```bash
ffmpeg -ss 91 -i "$SOURCE" -t 18 \
  -vf 'crop=1272:914:28:110,setpts=PTS-STARTPTS,fps=24,setsar=1' \
  -an -c:v libx264 -preset slow -crf 21 -pix_fmt yuv420p \
  -movflags +faststart -map_metadata -1 media/Simulation_Overtaking.mp4

ffmpeg -ss 3 -t 12 -i media/Simulation_Overtaking.mp4 \
  -filter_complex '[0:v]fps=6,scale=640:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=64:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=5' \
  -loop 0 media/Simulation_Overtaking_preview.gif
```

## Onboard static-obstacle avoidance

[MPC_Avoidance.mp4](MPC_Avoidance.mp4) and [MPC_Avoidance_preview.gif](MPC_Avoidance_preview.gif) are the existing onboard demonstration linked from the main README. They are unchanged by the simulation-video edit.
