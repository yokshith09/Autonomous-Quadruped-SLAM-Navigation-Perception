# Best result, scene 1 (`scene/robot_working.ttt`), 2026-11-14

Run 34, folder `runs/20261113_232046`, code tag `best-sim-v3` on `improve/exploration`. 1800 s simulated (30 min), start
(4.00, 4.00, -72.6 deg). Comparison with the baseline: `docs/results/comparison_run34_best.md`.

| Measure | Baseline (run 20261024_210638) | Run 34 |
|---|---|---|
| Position error (ATE RMSE) | 31.0 cm | **11.7 cm** |
| Largest position error | 63.1 cm | **24.5 cm** |
| Map accuracy vs real scene | 10.8 cm | **2.8 cm** |
| Map precision / recall / F1 | 73.4 / 83.0 / 77.9 % | **99.7 / 92.4 / 95.9 %** |
| Real floor mapped | 88.3 % | **94.6 %** |
| Real rooms explored | 6 of 7 | **7 of 7** |
| Contacts / stuck events / tip-overs | 4 stuck | **0 / 0 / 0** |
| Under the table | no (tabletop mapped as a block) | **yes: 29 s underneath, within 7 cm of the centre** |
| Real-time factor | 0.32 | **0.73** |
| Peak memory | 2973 MB | **930 MB** |

Against the baseline: 24 better, 3 worse, 5 within 3 % of 32 metrics. Both runs' map figures are scored with the same obstacle band, 0.04-0.25 m above the floor
(F62; the baseline was re-scored on it). Wasted walking while exploring is 48 % (baseline 46 %): from ~1000 s to 1785 s the robot crossed the map for 3-8 cell
frontier slivers along walls (coverage 80.0 -> 81.0 %). F64 (after this run) skips far slivers; not yet measured.

## What changed to get here

| Fix | What it does | Evidence |
|---|---|---|
| F61 robust pose graph | Every loop closure is a soft link; the graph is re-solved on each closure with dynamic covariance scaling, so a wrong closure is scaled down after the fact instead of applied forever | Replay of 5 full recordings: worst run 49.1 -> 10.2 cm, mean 25.6 -> 19.3 cm |
| F62 split height bands | Scan matching keeps walls up to 0.60 m; map and bumper use up to 0.20 m above the body (~0.25 m above the floor), so the robot fits under tables | Run 32 with one low band: 15 cm at 150 s; run 33 split: 4 cm |
| F63 floor-coverage patrol | After exploring, patrol walks floor the robot has never stood on, not only floor it has not seen | Test fails on the old rule; run 34 went under the table |
| F56, F59, F60 | Contact memory, closure cap 0.50 m, planner escape from its own margins | Runs 23, 29, 30 |

## Limits

One 30-minute run. Earlier code varied from run to run (16 to 49 cm over full missions); run 33 with the same SLAM code
gave 11 cm over 15 minutes. A 50-minute mission on this code has not been run.
