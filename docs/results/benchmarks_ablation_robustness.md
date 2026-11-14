# Ablation and robustness (benchmarks 3b, 3c; 2026-11-14)

All numbers: ATE RMSE from replaying stored recordings through SLAM (`evaluation/slam_replay.py`), same scoring as the
live runs. Scene 1 = run 34 recording (single replay; scene 1 replays are deterministic and reproduce the live 11.7 cm).
Rescue = run 19 recording, mean of 3 seeds (seed = 0.2 % odometry noise via `evaluation/noise_stage.py`), because
single rescue replays swing by several cm.

## Ablation: remove one part at a time

| Variant | Scene 1 | Rescue (3-seed mean) |
|---|---|---|
| Full system (scene profile) | **11.7 cm** (submap 8) | **11.0 cm** (submap 16) |
| No loop closure | 25.9 cm | 12.4 cm |
| Plain least-squares closure instead of robust (DCS) pose graph | 27.3 cm | 9.3 cm |
| Scan-to-scan matching instead of keyframe submap | 131.8 cm | 29.0 cm |
| Submap 16 (scene 1) / submap 8 (rescue) | 15.2 cm | 8.3 cm |
| Point-to-point ICP instead of point-to-line | 291.4 cm (= odometry only) | 15.9 cm (= odometry only) |
| No degeneracy filter | 16.1 cm | 10.9 cm |
| No heading prior (Manhattan + ICP yaw) | 295.6 cm | not used in rescue |
| No stand-still gyro calibration | not used in scene 1 | 178.0 cm |
| Odometry only (no SLAM) | 291.4 cm | 15.7 cm |

Findings
- Every part earns its place in scene 1; the submap is the largest single gain (131.8 -> 11.7 cm).
- In the rescue scene the gyro calibration is essential (178 -> 11 cm) and the submap matters (29 -> 11 cm).
- Point-to-point ICP never passes the odometry-agreement gate, so it degenerates to odometry in both scenes.
- On the run 19 recording, submap 8 (8.3 cm) and the plain pose graph (9.3 cm) beat the adopted settings. The adopted
  submap 16 was chosen on the mean of 9 replays of three other rescue recordings (13.9 vs 16.3 cm). The evidence is
  mixed; re-decide on all four recordings together before the next rescue run. Results stay frozen until then.

## Robustness: seeded sensor faults (rescue, run 19 recording, 3 seeds each)

| Fault | Ours | Odometry only |
|---|---|---|
| None | 12.3 cm | 15.7 cm |
| Depth range noise 1 cm / 3 cm | 13.6 / 11.1 cm | 15.7 / 15.7 cm |
| Depth points dropped 20 % / 50 % | 10.4 / 9.7 cm | 15.7 / 15.7 cm |
| Leg odometry noise 5 % / 10 % per command | 9.5 / 10.0 cm | 15.9 / 16.2 cm |
| Leg odometry scale error -5 % / +5 % | 15.9 / 15.7 cm | 31.9 / 30.7 cm |
| Residual gyro bias 0.01 / 0.03 deg/s | 29.9 / 100.5 cm | 29.6 / 98.0 cm |
| Gyro bias random walk 0.005 deg/s per sqrt(s) (harsh) | 263.9 cm | 279.7 cm |
| Combined (2 cm, 20 %, 0.01 deg/s, 0.002 walk, 5 %, -3 %) | 47.8 cm | 50.9 cm |

Findings
- Depth noise, dropouts and leg-odometry noise are absorbed; scale errors are halved by scan matching.
- Gyro bias left after calibration passes straight into the error: in the rescue profile heading correction from scans
  is off (icp_yaw_weight 0, manhattan_gain 0), so heading comes from the gyro alone. This is the main sim-to-real risk.
- The random-walk level was chosen as a stress value; check it against the real gyro's datasheet before quoting it.
