# SLAM baselines on our recordings (benchmark 3a, 2026-11-14)

Same recordings, same inputs, same scoring (`evaluation/accuracy_evaluator.evaluate_trajectory`, ATE RMSE against
CoppeliaSim ground truth). Recordings: scene 1 run 34 (20261113_232046, 1800 s, 156.9 m) and rescue run 19
(20261114_171629, 720 s, 38.5 m). Runner: `python -m evaluation.baselines <recording> [--calibrate-s 62] --method ...`.

| Method | Input | Scene 1 | Rescue | Average |
|---|---|---|---|---|
| Odometry only (dead reckoning) | leg odometry + gyro (bias from stand-still calibration) | 291.4 cm | 15.7 cm | 153.6 cm |
| BreezySLAM RMHC, default settings | 2D scan + forward odometry + gyro | 1791.9 cm | 568.4 cm | 1180.2 cm |
| BreezySLAM RMHC, best of 11 settings | same | 189.7 cm | 109.5 cm | 149.6 cm |
| **Ours** (replay; identical to the live runs) | 2D scan + leg odometry + gyro | **11.7 cm** | **12.3 cm** | **12.0 cm** |

- Our replays reproduce the live numbers exactly (11.7 and 12.3 cm), so all rows come from one pipeline.
- Odometry only: scene 1 has no stand-still gyro calibration, so heading drifts 44.6 deg over 30 min; scan matching in our
  SLAM holds heading to 0.76 deg. In the rescue scene our SLAM cuts the odometry error by 22 % (15.7 -> 12.3 cm).
- BreezySLAM check: with scan matching off (Deterministic_SLAM) it reproduces odometry-only to 0.1 cm, so the frame
  conversion is right; the error comes from its scan matching.
- BreezySLAM tuning (rescue run, then applied to scene 1): search spread sigma_xy 5-100 mm, sigma_theta 0.5-20 deg,
  hole width 200-600 mm, scan rate every 1-20 scans. Best: sigma_xy 10 mm, sigma_theta 1 deg, hole 200 mm, every 20th
  scan. All settings were worse than odometry alone in the rescue scene.
- Why BreezySLAM fails here: it was designed for a 360 deg lidar; our scan is a 58 deg depth-camera slice
  (~210 points, up to 6.6 m) and it has no loop closure. It also takes no lateral odometry.

Limits: BreezySLAM is a light 2D SLAM (CoreSLAM/tinySLAM); a stronger same-data reference is Cartographer (3f).
Our system was developed on these scenes; BreezySLAM got 11 settings of tuning on one of them.

Sources: BreezySLAM, github.com/simondlevy/BreezySLAM (built from source, 2026-11-14); CoreSLAM: B. Steux,
O. El Hamzaoui, "tinySLAM: A SLAM algorithm in less than 200 lines C-language program", ICARCV 2010.
