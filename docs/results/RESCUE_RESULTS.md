# Rescue scene (SearchRescueScene): results and comparison with scene 1 (2026-11-14)

Same code as scene 1. Scene-specific settings come from `config/scenes/searchrescuescene.json`:
- floor calibration;
- narrow gaps;
- map window covering the whole floor;
- gyro calibration time;
- no right-angle heading prior;
- no scan heading pull.

Scored against the scene's own geometry: `evaluation/scene_truth_mesh.py` samples the triangles of every shape, then `evaluation/truth_eval.py` scores the map.

## The two scenes

| | Scene 1 (best: run 34) | Rescue scene |
|---|---|---|
| Size | 10 x 10 m, walled room with interior walls, table | 16 x 16 m open floor, **no walls at the edge (drop)** |
| Structure | walls at right angles everywhere | scattered rubble blocks, a damaged building, beam, barrier |
| Walking speed (odometry scale) | 1.0 | 0.701 (net progress) |

## Results

| Run | Length | ATE (cm) | max (cm) | Precision | Recall | F1 | Wall accuracy | Floor observed | Stuck / falls |
|---|---|---|---|---|---|---|---|---|---|
| Scene 1, run 34 (`best-sim-v3`) | 1800 s | **11.7** | 24.5 | 99.7 % | 92.4 % | **95.9 %** | 2.8 cm | 97.7 % | 0 / 0 |
| Rescue run 12 (20261114_041347) | 1290 s (stopped) | **9.9** | 24.4 | 94.8 % | 61.4 % | 74.5 % | 4.2 cm | 55.3 % | 1 / 0 |
| Rescue run 14 (20261114_045740) | 1800 s | 19.0 | 39.3 | 82.6 % | 56.9 % | 67.4 % | 7.9 cm | 67.6 % (170 m² free) | 0 / 0 |

Rescue precision, recall and F1 are against the whole 16 x 16 m floor. Recall is limited by how much of the floor the robot covered in the time.

Run 14 crossed the damaged building through its interior gap and out through the north doorway (about (1.5, 4.2), around 600 s). It had 0 stuck events and no falls, and it detected one real floor edge ahead ("drop ahead at 0.44 m", south edge).

## Why the rescue scene is harder

- **Nothing to match in open floor.** The scan matcher needs structure in view, and much of the rescue floor shows none. There, position comes from leg odometry and gyro alone.
  - Odometry varies about ±7 % with direction and gait wobble, against a calibrated mean of 0.931 net/odometry.
  - A residual gyro bias left after a 60 s stand-still is about 0.003°/s.
  - Run 14's error budget: −0.5° to −1.2° of heading error gives a steady 4–15 cm per minute of sideways drift.
- **Loop closures help little here.** A closure only corrects relative to the keyframe it matches, and those keyframes already carry the drift. In run 14, the 12 closures moved the position by under 1 cm each.
- **No walls at the edge.** The floor ends in a drop, so the robot must keep its distance from unknown space: 0.50 m viewpoint standoff (F72, F77) and a camera drop check (F78).

## Fixes made in this scene (all with tests that fail on the old code)

| Fix | Problem |
|---|---|
| F66 | Stall threshold scales with the floor calibration (false stalls on the slower floor) |
| F67 / F75 | Stand-still gyro calibration; the settled part is bias, not rotation (it left +1.6° in the heading) |
| F68 | No frontier targets on the map window's edge |
| F69 | Position held while standing still |
| F70 | Turning cost instead of a heading bonus (the back-and-forth walking) |
| F71 | Floor calibration on net progress, not path length incl. sway (odometry was 7 % long) |
| F72 / F77 | Frontier viewpoints 0.50 m from unknown space, slid targets included (run 13 walked off the floor) |
| F73 | Exploration "done" only when viewpoints exist |
| F74 | An empty scan still dead-reckons (run 10 dithered 60 s on a frozen heading) |
| F76 | Failures counted by place, not grid bucket (run 12 retried one gap for 300 s) |
| F78 | Drop detection from the floor the camera sees |
| Map window per scene | The whole floor; it was 11 x 11 m, which left the north side unreachable |

## Open points

1. **Coverage.** The north part (y > 4.5) was not swept in 30 min: the robot walks about 6 cm/s and the floor is 256 m². Room-by-room sweeping also re-labels explored floor in an open field (run 14 swept "room 11", already covered floor, for about 150 s).
2. **Accuracy in open floor** (drift between structures). Candidates:
   - a gyro bias kept as a state and refined online;
   - a gait-dependent odometry model (net/odometry falls from 0.96 to 0.91 as yaw wobble rises, correlation −0.70);
   - prefer routes that keep structure in view.
