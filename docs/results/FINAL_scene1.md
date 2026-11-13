# Final results, scene 1 (`scene/robot_working.ttt`), 2026-11-14

Code: tag `final-sim-v1` on `improve/exploration`. Mission: 3000 s simulated (50 min), explore then patrol. All numbers
come from the run folders under `runs/`; the three-run comparison is `docs/results/comparison_final_scene1.md`.

## Every run with the current gait that went past 30 minutes

| Run | Folder | Code at the time | Length | ATE | Largest error | Map accuracy | Map F1 | Floor mapped | All explored at | Real rooms |
|---|---|---|---|---|---|---|---|---|---|---|
| 19 | 20261112_232106 | `best-sim-v2` | 3000 s | 16.5 cm | 62 cm | 4.7 cm | 92.0 % | 92.8 % | 1371 s | 7 / 7 |
| 24 | 20261113_120000 | + F55, F56 | 3000 s | 17.9 cm | 59 cm | 6.0 cm | 90.7 % | 91.7 % | 1257 s | 7 / 7 |
| 29 | 20261113_171057 | run 19 + F56, F57 | 2108 s (stopped) | 30.7 cm | 79 cm | 7.8 cm | 86.7 % | 91.2 % | 1751 s | 7 / 7 |
| 30 | 20261113_183454 | + F59 | 1826 s (stopped) | 27.1 cm | 63 cm | 9.5 cm | 79.3 % | 91.1 % | 1451 s | 7 / 7 |
| 31 | 20261113_192627 | `final-sim-v1` | 3000 s | 49.1 cm | 105 cm | 18.6 cm | 69.1 % | 88.7 % | 1688 s | 7 / 7 |
| Baseline | 20261024_210638 | `baseline-code-run1` | 1800 s | 31.0 cm | 63 cm | 10.6 cm | 77.7 % | 88.2 % | never | 6 |

Run 19 started from (4.28, -1.77); the others from (4.00, 4.00). Runs 29 and 30 were stopped by hand after they failed
(a wrong 67 cm loop closure; the robot boxed in by its own contact marks, fixed by F60).

## What to report

**Exploration and reliability (solid; "±" figures are the three full missions 19, 24, 31):**
- All 7 real rooms explored in 5 of 5 runs (baseline: 6, and it never finished).
- Everything reachable explored in 1257 to 1751 s; the three full missions average 1439 ± 183 s.
- Wasted walking while exploring 28 ± 5 % (baseline 46 %).
- No tip-over in any of the five. Contacts per run: 0 to 4. Run 30 got boxed in by its own contact marks; fixed by F60, and run 31 never got stuck.
- Real-time factor 0.66 ± 0.05 (baseline 0.32); peak memory 1364 ± 25 MB (baseline 2973 MB).
- Planning 9 ms, frontier selection 14 ms, control step 152 ms (baseline 128, 155, 380 ms).

**Localisation and mapping (varies from run to run):**
- ATE over the five runs: 16.5, 17.9, 27.1, 30.7, 49.1 cm. Median 27.1 cm, best 16.5 cm.
- Three full missions: ATE 27.9 ± 15.1 cm, map accuracy 9.8 ± 6.3 cm, F1 84.0 ± 10.5 %.
- Best observed full mission (run 19): ATE 16.5 cm, map accuracy 4.7 cm, F1 92.0 %.
- Replaying the five recordings through the final SLAM settings (closure cap 0.50 m) gives 15.2, 19.0, 17.6, 27.1,
  49.1 cm (median 19.0 cm). This is a replay, not a live result: the route is the recorded one.
- Heading error stays at 0.75 deg in every run.

Against the baseline, the mean of the three full missions is better on 25 of 32 metrics, worse on 4, same on 3.

## Why accuracy varies

A run stays at 6 to 20 cm until one loop closure is wrong: the scan of a similar-looking place matches as well as the
right one. Runs 29 and 31 were at 3 cm and 20 cm one second before a 67 cm and a 49 cm wrong closure. Eleven ways to
reject wrong closures were tested by replay or live run and none helped on all recordings (see `docs/project/PROGRESS.md`).
The sensor is one 60 degree depth camera reduced to a 2D slice.

## Not usable as a result

Run 8 (20261107_180907, ATE 14.7 cm) cannot be reproduced: its code rerun twice on 2026-11-13 reached 75 cm at 300 s.
