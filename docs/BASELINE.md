# Baseline interpretation (run 20261024_210638)

Measured values are in [`runs/20261024_210638/REPORT.md`](../runs/20261024_210638/REPORT.md). This file interprets
them. It uses only that run and states what it does not show. Nothing has been optimized yet.

## Setup of the baseline
- Code: commit `6f9c01e` (the run itself recorded `1f231d2` + uncommitted changes, which are exactly that commit).
- One run, 1800 simulated seconds (the cap chosen by me; the sim took 5666 s of wall time, 0.32x real time).
- Ended by **timeout**, not completion: 6 of 8 rooms completed, 48 frontiers left.
- Replay is faithful: the full-system replay matches the live trajectory to 0.01 cm, and two identical replays are identical.

## What the measurements show
| Area | Result | Reading |
|---|---|---|
| Localization | ATE RMSE 31.0 cm, final error 38.5 cm (0.24 % of 157.9 m), error never above 63 cm, heading 0.74 deg | No catastrophic drift over 30 minutes. This is the strong part |
| Mapping | wall accuracy 12.9 cm, recall 91 %, precision 67 %, 5407 vs 3184 occupied cells | Walls are found but smeared/doubled by the pose error. Mapping is weaker than localization |
| Speed | 0.088 m/s average | Slow; this limits mission time more than compute does |
| Compute | 86 % of one CPU core, 3 GB peak memory, real-time factor 0.32 | Not real time on this machine; agents account for only 46 % of wall time |
| Contribution (offline replay) | removing scan matching: ATE 31 -> 265 cm; removing the Manhattan prior: 31 -> 169 cm; removing loop closure: 31 -> 54 cm; removing the degeneracy filter: 31 -> 33 cm | Scan matching, the heading prior and loop closure each matter on this run. The degeneracy filter shows **no measurable benefit** here (final error was lower without it: 30.7 vs 38.5 cm) |

Observation to investigate, not a conclusion: odometry plus gyro alone has a *lower* local error (RPE 1.8 %) than
the full system (7.4 %) but drifts in heading to 44.6 deg. Scan matching buys global consistency at the price of
local jitter; the cause was not isolated.

## Why the run did not finish (diagnosis, not yet confirmed by a fix run)
Evidence: final map image (`map_overlay.png`), the frontier code, and the log.
1. Coverage reached 78.4 % of the map box by t = 1000 s and 80.1 % at 1800 s. The arena fills about 81 % of the box,
   so the interior was essentially mapped by 1000 s; the remaining 42 % of the distance walked (66.8 of 157.9 m) added 1.7 points.
2. Of the 48 remaining frontiers, 21 lie outside the outer wall and 13 within 0.5 m of it. The map has 579 free cells
   outside the wall (rays leaking past thin or smeared walls). A frontier is any free cell next to unknown, so these
   can never be cleared by looking from inside.
3. A frontier within 0.3 m of reachable floor counts as reachable, so the robot drives to the wall, the frontier
   survives, the target is blacklisted, and the blacklist is cleared when it exceeds 40 entries or no candidate is left.
   The same targets then return (single targets were planned 11-21 times; 1814 paths for 36001 steps).
4. Room labels flip as the map grows: 17 "entered new room" events for 8 rooms, which restarts room sweeps.

## Planned fixes (to be tested against this baseline; they change behaviour, so numbers will change)
1. Ignore frontiers with no navigable floor within about 0.2 m and any outside the arena bounds or behind an occupied wall.
2. Keep the blacklist per area permanent and declare completion when only blacklisted or unreachable frontiers remain.
3. Freeze room labels once a room is marked explored.
4. Remove the time cap as the normal stop condition (keep a large safety cap).
Then: profile (`cProfile`) to find where the 54 % of wall time outside the agents goes; speed-ups must leave results
unchanged, which is checked by replaying this recording.

## What this baseline cannot show
- Repeatability: one run. Variance needs several runs with different gyro seeds.
- Real-robot performance: simulation only; the gait speed here is not a real robot's.
- Independent map accuracy: the reference grid uses the true pose, not a floor plan.
- Contribution of mapping, frontier selection and planning (only SLAM components were ablated).
- External comparison: not started; see `docs/OBJECTIVES.md` (R14).
