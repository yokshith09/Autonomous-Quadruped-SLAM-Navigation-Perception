# Baseline (run 20261024_210638)

[`PROJECT_REPORT.pdf`](../results/PROJECT_REPORT.pdf) is the complete formal report. [`RESULTS.pdf`](../results/RESULTS.pdf) is the shareable document (values, definitions, comparison). [`DIAGNOSTIC_REPORT.pdf`](../results/DIAGNOSTIC_REPORT.pdf) is for
internal use: why each value came out that way, a Good / Improve / Poor class, then the comparison with published systems. The raw record is
[`runs/20261024_210638/REPORT.md`](../../runs/20261024_210638/REPORT.md). Every fix needed is in [`FIXES.md`](FIXES.md).
Nothing has been optimized yet.

## Setup
- Code: snapshot commit `6f9c01e` (the run recorded `1f231d2` plus uncommitted changes, which are exactly that commit).
- One run, 1800 simulated seconds (a cap I set), 5666 s of wall time (0.32x real time). Ended by **timeout**, not completion:
  6 of 8 rooms completed, 48 frontiers left.
- The replay is faithful: the full-system replay matches the live trajectory to 0.01 cm; two identical replays are identical.

## Summary
38 metrics were classified: **10 Good, 12 Improve, 16 Poor** (limits and their basis are in the diagnostic PDF, section 2).

| Area | Measured | Class |
|---|---|---|
| Localization drift | final error 38.5 cm = 0.24 % of 157.9 m; never above 63 cm; heading error 0.74 deg | Good |
| Localization absolute / local | ATE RMSE 31.0 cm (target 15 cm); relative error 7.4 % per metre | Improve / Poor |
| Mapping | recall 91 %, update time 37 ms and memory 0.19 MB | Good |
| Mapping | F1 77 %, completion 6.4 cm, free-space IoU 81 % | Improve |
| Mapping | wall accuracy 12.9 cm, precision 67 %, completion ratio 54 % | Poor |
| Exploration | not completed; 0.088 m/s; 46 % of the path walked after coverage had plateaued; 2.25 room entries per room | Poor |
| Compute | SLAM 10.6 ms per update; loop p95 380 ms, stalls up to 2.2 s; real-time factor 0.32 | Good (SLAM) / Poor (loop) |
| SLAM components | removing scan matching: ATE +756 %; Manhattan prior +446 %; loop closure +76 %; degeneracy filter +6 % | Helps / Helps / Helps / Not shown |

## Against the real scene
Wall and object geometry read from the CoppeliaSim scene (`evaluation/data/scene_truth.json`, scoring only; the robot never sees it):
| Measured | Value |
|---|---|
| Map accuracy / completion vs real walls and objects | 10.6 / 9.2 cm (the same rays with the true pose: 0.2 cm, so the error is pose error) |
| Map precision / recall / F1 (15 cm) | 73.9 / 81.8 / 77.7 % |
| Real floor mapped as free | 88.2 % of 85.2 m2 |
| Free space mapped outside the outer wall | 0.59 m2 |
| Object detections lying on a real object | 24 of 113 (21 %); all 113 are labelled "chair"; 1 lies on the scene's only chair |

Object detection is a size rule on map blobs, not recognition (fix F28); the knowledge graph has placeholder doorways and room centres (F29).

Observation to investigate, not a conclusion: odometry plus gyro alone has a *lower* local error (RPE 1.8 %) than the full system
(7.4 %) but drifts in heading to 44.6 deg; scan matching trades local smoothness for global consistency (fix F7).

## Why the run did not finish (diagnosis; to be confirmed by the fix run)
1. The explored area reached 97 % of its final value at t = 930 s. The last 73.2 m (46 % of the path) added 2.1 points of coverage.
2. Of the 48 remaining frontiers, 21 lie outside the outer wall and 13 within 0.5 m of it; the map has 579 free cells outside the wall.
   Even the reference grid built with the true pose has 34 frontiers (9 outside), so the depth rays leak past walls independent of
   pose error; pose error makes it worse (48 vs 34). A frontier is any free cell next to unknown, so these cannot be cleared from inside.
3. A frontier within 0.3 m of reachable floor counts as reachable, so the robot drives to the wall, the frontier survives, the
   target is blacklisted, and the blacklist is cleared when it exceeds 40 entries or no candidate remains. The same targets return
   (single targets planned 11-21 times; 1814 paths).
4. Room labels change as the map grows: 18 room-entered events for 8 rooms, which restarts room sweeps.

## Comparison (diagnostic PDF section 3, shareable PDF sections 6-10)
SLAM as a whole (pose and map together, against RGB-D systems evaluated on Replica): pose error 31.0 cm against 0.37-1.06 cm, map accuracy 10.6 cm
(against the real scene) against 1.6-2.1 cm; but 0.19 MB of map against 20-31 MB and 0.02 s of mapping per frame against 0.14-10.5 s. The scenes,
map representation (2D wall cells against 3D surfaces) and sensing differ, so this locates the gap, not a ranking.
No published system was evaluated under our conditions, so **none is directly comparable**. Closest, all partial: the A1 SLAM paper
(quadruped, 5 m paths), the ANYmal D evaluation (quadruped, 145-332 m, mostly outdoors) and FARE (exploration, wheeled).
- Localization: our ATE per path length (0.20 %) is the same order as ORB-SLAM3 stereo (derived 0.14 %) and above FAST-LIVO2 (0.04 %); no claim to beat them.
- Speed: slower than every listed system (FARE indoor planners 1.6-1.8 m/s derived; ANYmal 0.75 m/s; Spot 1.6 m/s).
- Runtime: our SLAM update (10.6 ms) is cheaper than camera-based tracking (ORB-SLAM2 25.6 ms), which says nothing about accuracy.
- Mapping: not comparable (2D 5 cm grid against 3D neural meshes).
- Market (capabilities from manufacturer pages, 2026-10-11 and 2026-10-25): Spot and Go2 follow a recorded map or user path; ANYmal, Unitree B2 and
  DEEP Robotics X30 do not state whether they explore unknown space; Emesent Hovermap (LiDAR payload, mainly drones) has an autonomous exploration mode.
  All are far faster (0.75 to over 6 m/s) than our 0.088 m/s; endurance, protection and price do not apply to a simulation.
- Like-for-like reference (RESULTS.pdf section 14): TUM RGB-D Pioneer robot sequences, ATE RMSE averaged over 4 sequences: ORB-SLAM3 1.720 m,
  OpenVSLAM 0.392 m, RTABMap 0.287 m (Merzlyakov & Macenski, arXiv:2107.07589). Ours: not yet run.
- The fair experiments that are still missing are fixes F25-F27 (head-to-head on our own recording, same-arena baseline explorer, repeated runs).

## What this baseline cannot show
One run (no variance); one arena; simulation only; time to complete exploration is unknown (the run ended at its time limit); where the
54 % of wall time outside the agents goes is not profiled; the contribution of mapping, frontier selection and planning is unmeasured;
scan-match acceptance and tracking-loss counts are diagnostic; interventions and collisions were not measurable; nothing was run on a
public dataset or on target robot hardware.
