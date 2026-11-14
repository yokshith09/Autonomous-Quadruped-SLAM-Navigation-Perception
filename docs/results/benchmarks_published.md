# Our cross-environment results next to published systems (2026-11-14)

Ours = average of scene 1 run 34 and rescue run 19 (tag `final-multi-env-v1`), see BEST_RESULTS_ALL_ENVIRONMENTS.md.
Published numbers come from `evaluation/benchmarks.py`, each tied to its source table.

**Read this first.** These are different robots, sensors, scenes and path lengths, and ours is in simulation
(clean sensors, exact ground truth) while every published number below is from real hardware or real recordings.
The tables place our numbers in context; they do not rank systems. A same-data comparison needs T3/T4.

## 1. Localization (ATE RMSE)

| System | Platform / sensors | Setting | Path | ATE | ATE / path |
|---|---|---|---|---|---|
| **Ours (avg of 2 scenes)** | sim quadruped, depth camera + leg odometry + gyro | CoppeliaSim, rooms + rescue field | 157 m / 38.5 m | **12.0 cm** | ~0.2 % |
| FAST-LIVO2 [QUAD] | ANYmal D, LiDAR + camera + IMU | real, GrandTour, 7 missions | 145-332 m | 10.5 cm | ~0.05 % |
| ORB-SLAM3 [QUAD] | ANYmal D, stereo + IMU | same | 145-332 m | 22.9 cm | ~0.1 % |
| ORB-SLAM3 [QUAD] | ANYmal D, stereo | same | 145-332 m | 33.0 cm | |
| RTAB-Map [QUAD] | ANYmal D, stereo | same | 145-332 m | 57.9 cm | |
| DPV-SLAM [QUAD] | ANYmal D, camera | same | 145-332 m | 96.7 cm | |
| RTAB-Map [QUAD] | ANYmal D, RGB-D (ZED2i) | same | 145-332 m | 241.3 cm | |
| ORB-SLAM3 [QUAD] | ANYmal D, RGB-D (ZED2i) | same | 145-332 m | 821.4 cm | |
| A1 SLAM [A1] | Unitree A1, onboard sensors | real indoor, 4 runs | ~5 m | 4.3-14.5 cm | 1-3 % |
| Cartographer [A1] | Unitree A1 | same | ~5 m | 4.0-139.7 cm | |
| RTABMap [MM] | Pioneer, Kinect RGB-D | TUM fr2/pioneer (4 seq.) | 16-40 m | 28.7 cm | |
| OpenVSLAM [MM] | same | same | 16-40 m | 39.2 cm | |
| ORB-SLAM3 [MM] | same | same | 16-40 m | 172.0 cm | |

## 2. Mapping

| System | Setting | Map accuracy (mapped -> true) | Notes |
|---|---|---|---|
| **Ours (avg)** | 2 sim scenes, 2D occupancy | **4.0 cm**; precision 97.8 %, F1 75.6 % | F1 limited by 12-min rescue coverage |
| Neural RGB-D SLAM (iMAP ... MNE-SLAM) [MNE] | Replica, synthetic rooms, dense 3D | 1.6-2.9 cm | dense 3D meshes, small rooms, given camera images |

## 3. Runtime

| System | Hardware / setting | Time per update |
|---|---|---|
| **Ours** | whole control step, all agents, Windows PC | p50 25 ms, p95 128 ms |
| ORB-SLAM2 [ORB2] | tracking per frame, TUM fr3, i7-4790 | 25.6 ms |
| MNE-SLAM / ESLAM / Co-SLAM [MNE] | tracking per frame, Replica, GPU | 22-69 ms (+ 0.14-0.29 s mapping) |

## 4. Exploration

| System | Setting | Result |
|---|---|---|
| **Ours** | sim quadruped, ~0.1 m/s; 10 x 10 m rooms / 16 x 16 m rescue field | 97.7 % floor in 1800 s (scene 1); 59 % in 720 s (rescue); 0 collisions measured |
| TARE, DSVP, ARiADNE, HEADER, FARE [FARE] | Gazebo indoor, wheeled, 3D LiDAR, up to 2 m/s | full exploration in 576-931 s over 1030-1511 m |
| Spot (Autowalk) [SPOT] | commercial | does not explore unknown space; replays a recorded map |

## What can honestly be claimed
- Localization error in the same range as published quadruped SLAM on real robots, with a cheaper sensor set
  (one depth camera, no LiDAR), across two different environments with the same code.
- Real-time on a PC (median 25 ms per full control step).
- Not claimable from these tables: "more accurate than X" (different data; ours is simulated).

Sources: [QUAD] Corlito et al. 2026, arXiv 2606.19067; [A1] Chen & Dellaert, arXiv 2211.14432;
[MM] Merzlyakov & Macenski, arXiv 2107.07589; [MNE] Deng et al., CVPR 2025; [ORB2] Mur-Artal & Tardos, arXiv 1610.06475;
[FARE] Liao et al. 2026, arXiv 2601.14681; [SPOT] Boston Dynamics product/SDK docs.
