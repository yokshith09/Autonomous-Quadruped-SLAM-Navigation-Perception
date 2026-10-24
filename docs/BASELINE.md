# Baseline (run 20261024_210638)

The shareable result is [`BASELINE_REPORT.pdf`](BASELINE_REPORT.pdf): our results with what each value is, why it came out that
way and a Good / Improve / Poor class, then the comparison with published systems. The raw record is
[`runs/20261024_210638/REPORT.md`](../runs/20261024_210638/REPORT.md). Every fix needed is in [`FIXES.md`](FIXES.md).
Nothing has been optimized yet.

## Setup
- Code: snapshot commit `6f9c01e` (the run recorded `1f231d2` plus uncommitted changes, which are exactly that commit).
- One run, 1800 simulated seconds (a cap I set), 5666 s of wall time (0.32x real time). Ended by **timeout**, not completion:
  6 of 8 rooms completed, 48 frontiers left.
- The replay is faithful: the full-system replay matches the live trajectory to 0.01 cm; two identical replays are identical.

## Summary
31 metrics were classified: **10 Good, 9 Improve, 12 Poor** (limits and their basis are in the PDF, section 2).

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

## Comparison (PDF section 3)
No published system was evaluated under our conditions, so **none is directly comparable**. Closest, all partial: the A1 SLAM paper
(quadruped, 5 m paths), the ANYmal D evaluation (quadruped, 145-332 m, mostly outdoors) and FARE (exploration, wheeled).
- Localization: our ATE per path length (0.20 %) is the same order as ORB-SLAM3 stereo (derived 0.14 %) and above FAST-LIVO2 (0.04 %); no claim to beat them.
- Speed: slower than every listed system (FARE indoor planners 1.6-1.8 m/s derived; ANYmal 0.75 m/s; Spot 1.6 m/s).
- Runtime: our SLAM update (10.6 ms) is cheaper than camera-based tracking (ORB-SLAM2 25.6 ms), which says nothing about accuracy.
- Mapping: not comparable (2D 5 cm grid against 3D neural meshes).
- The fair experiments that are still missing are fixes F25-F27 (head-to-head on our own recording, same-arena baseline explorer, repeated runs).

## What this baseline cannot show
One run (no variance); simulation only; the map reference is built from the same sensor with the true pose; the contribution of mapping,
frontier selection and planning is unmeasured; scan-match acceptance and tracking-loss counts are diagnostic; interventions and
collisions were not measurable.
