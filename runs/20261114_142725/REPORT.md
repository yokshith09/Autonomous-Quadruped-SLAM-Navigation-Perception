# Baseline report: run 20261114_142725

Every number comes from this run's own files. Measured = computed from telemetry or ground truth; estimate / diagnostic indicator are labelled as such; n/a = could not be measured (reason shown).

## Run identification

| Item | Value |
|---|---|
| Started | 2026-11-14T14:27:25 |
| Command | `main_agentic.py --headless --max-time 720` |
| Code | git 0cd717e (+ uncommitted changes at run time) |
| CPU / GPU | AMD Ryzen 7 7735HS with Radeon Graphics (16 threads) / NVIDIA GeForce RTX 3050 6GB Laptop GPU, 6144 MiB |
| Platform | Windows-11-10.0.26300-SP0, Python 3.12.10 |
| Configuration | `{"max_time_s": 720.0, "max_steps": null, "grid_res_m": 0.05, "map_bounds_m": [-8.25, 8.25, -8.25, 8.25], "ground_truth_nav": false, "headless": true, "agent_rates_hz": {"PerceptionAgent": 10.0, "SLAMAgent": 10.0, "MappingAgent": 10.0, "FrontierAgent": 1.0, "PlanningAgent": 5.0, "LocomotionAgent": 20.0, "SupervisorAgent": 2.0}}` |
| Ground truth | CoppeliaSim /Base pose every control step (used for scoring only) |
| Runs in this report | 1 (no variability can be shown) |

| Source data | File | Detail |
|---|---|---|
| raw SLAM input recording | recording.npz | 60 MB, sha256 9c82b550f7988916... |
| per-step ground-truth and estimated pose | trajectory.csv |  |
| agent table | agents.csv |  |
| all metrics | metrics.json |  |
| run manifest | manifest.json |  |
| reference/SLAM grids, knowledge graph | artifacts |  |

## 1. Overall robot performance: what the robot achieved

| Metric | Value | Meaning |
|---|---|---|
| Outcome | timeout (720s) before all frontiers were cleared | why the run ended |
| Task success | no | mission reported all reachable frontiers explored |
| Simulated time | 720 s | robot time |
| Wall-clock time | 1485 s (real-time factor 0.49) | how long the computer needed; < 1 means slower than real time |
| Distance travelled | 31.9 m | path length from ground truth |
| Average speed | 0.044 m/s | distance / simulated time |
| Rooms completed / discovered | 0 / 7 | rooms the mapper segmented |
| Doorways detected | 18 | knowledge-graph entries; they accumulate as room segmentation changes, so this over-counts real doorways |
| Explored free area | 98.9 m2 | free cells x 0.05 m x 0.05 m |
| Grid observed | 36.8 % | of the 11 x 11 m map box, not of the arena |
| Stuck events / unreachable targets | 0 / 0 | watchdog and planner events |
| Human interventions | n/a (no intervention channel exists in the stack; only the 'q' key / external stop, recorded in end_reason) |  |
| Collisions | 0 |  |

## 2. Agent and component performance

### 2a. Operational performance (how the agents ran, not whether they helped)

| Agent | Rate (Hz) | Calls | Errors | p50 (ms) | p95 (ms) | max (ms) | Total (s) | Share | Task metric |
|---|---|---|---|---|---|---|---|---|---|
| PerceptionAgent | 10 | 14401 | 0 | 6.74 | 17.13 | 117 | 120 | 26 % | median 183 scan pts/update |
| SLAMAgent | 10 | 14401 | 0 | 0.58 | 21.43 | 348 | 92 | 20 % | 21.9% scans matched, 2 loop closures |
| MappingAgent | 10 | 14401 | 0 | 1.16 | 7.34 | 1717 | 38 | 8 % | 36.8% of grid observed |
| FrontierAgent | 1 | 1441 | 0 | 5.73 | 13.38 | 102 | 10 | 2 % | 0 rooms completed |
| PlanningAgent | 5 | 2881 | 0 | 0.38 | 9.59 | 840 | 16 | 4 % | 61 paths, 0 unreachable |
| LocomotionAgent | 20 | 14401 | 0 | 11.90 | 20.94 | 1778 | 179 | 39 % | 0 stuck/watchdog events |
| SupervisorAgent | 2 | 1441 | 0 | 0.06 | 0.18 | 1 | 0 | 0 % | 39 events logged, 7 rooms in graph |

Whole control-loop step: p50 33 ms, p95 190 ms, max 2932 ms. Agent time is 456 s of 1485 s wall time; the rest is simulator stepping, image transfer, HUD and per-step map queries outside `agent.step()` (split not measured).

### 2b. Contribution to the system result (offline replay of this run's recording, one component removed)

n/a (ablation not run: `python -m evaluation.ablate runs/<run_id>`)

## 3. Localization performance (independent of mapping)

Frame: 2D world frame of the simulator scene (x, y in m, yaw in rad). SLAM starts at the true start pose, so no trajectory alignment is applied. Evaluated at every control step (0.05 s) against simulator ground truth.

| Metric | Value | Meaning |
|---|---|---|
| ATE RMSE | 22.2 cm | typical position error over the whole run |
| ATE median / max | 9.3 / 41.8 cm |  |
| Final position error | 41.5 cm = 1.30 % of path | end-of-run drift |
| RPE translation (1 m segments) | 4.3 cm RMSE, mean 3.9 % of segment | local accuracy, independent of accumulated drift |
| RPE rotation | 0.65 deg/m |  |
| Heading error mean / max | 1.07 / 3.7 deg |  |
| Error first above 0.5 m / 1 m | n/a / n/a | n/a = never reached |
| Scan-match acceptance (diagnostic) | 21.9 % of 14400 updates | see warning in section 6 |
| Tracking-loss episodes (diagnostic) | 70 (443 s total) | >= 10 consecutive rejected updates |
| Followed by a loop closure within 30 s | 9 | relocalization proxy |
| Loop closures accepted | 2 |  |
| Odometry distance vs true | 30.3 m (-5.1 %) | diagnostic only: ground-truth path length also counts body motion the gait did not command, and the odometry-only replay in 2b has a low local error (RPE), so this is not a clean odometry scale error |

## 4. Mapping performance (independent of localization)

Reference = occupancy grid built from the same depth stream with the ground-truth pose (not an independent floor plan). Map size, coverage and accuracy are separate rows.

| Metric | Value | Meaning |
|---|---|---|
| Coverage: explored free area | 98.9 m2 | size of what was seen |
| Wall accuracy | 11.5 cm | SLAM walls to reference walls (lower is better) |
| Wall completion | 8.4 cm | reference walls to SLAM walls (lower is better) |
| Completion ratio (5 cm) | 53.8 % | reference wall cells matched within 5 cm |
| Chamfer / Hausdorff-95 | 10.0 / 36.1 cm |  |
| Precision / recall / F1 | 74.9 / 84.6 / 79.5 % | tolerance 15 cm |
| Free-space IoU | 79.8 % |  |
| Occupied cells SLAM / reference | 758 / 571 | more SLAM cells = thicker or smeared walls |
| Map update p50 / p95 | 1.2 / 7.3 ms |  |
| Map memory | 0.42 MB | occupancy grid |
| Frontiers remaining | 105 | unexplored boundaries left (see map_overlay.png) |

![map overlay](map_overlay.png)

### 4b. Against the real scene (wall and object geometry read from CoppeliaSim, scoring only)

| Metric | Value | Meaning |
|---|---|---|
| Map accuracy / completion | 8.7 / 147.4 cm | mapped obstacle cells to real ones / real to mapped (mean) |
| Map precision / recall / F1 | 80.1 / 21.4 / 33.8 % | within 15 cm of a real wall or object |
| Same, map built with the true pose | accuracy 0.6 cm, recall 23.6 % | what the sensor allows with perfect localization |
| Real floor mapped as free | 38.7 % of 234.0 m2 | true coverage |
| Free space mapped outside the arena | 5.39 m2 | impossible in reality; source of phantom frontiers |
| Object detections on a real object | 0 of 36 (0.0 %) | within 0.5 m of one of 0 real objects |
| Real objects with a detection nearby | 0 of 0 | inflated by the many scattered detections |
| 'Chair' labels on the real chair | 0 of 36 (scene has 0 chair) | chair label = blob 0.18-1.05 m across |

## 5. Computational performance

| Metric | Value |
|---|---|
| Process CPU mean / p95 | 46 / 71 % (of one core; CoppeliaSim not included) |
| Memory (RSS) mean / peak | 112 / 384 MB (includes the in-memory recording buffer) |
| GPU | NVIDIA GeForce RTX 3050 6GB Laptop GPU, 6144 MiB: n/a (no code path uses the GPU (NumPy/SciPy/OpenCV on CPU); nothing to attribute) |
| SLAM tracking p50 / p95 | 0.6 / 21.4 ms |
| Largest agent by total time | LocomotionAgent |
| Telemetry overhead (estimate) | 0.0064 ms/step, about 0.019 % of the median loop step |

## 6. Data quality and limitations

**Warnings**

- run ended by a limit (timeout (720s) before all frontiers were cleared), not by mission completion: coverage, rooms completed and task success are lower bounds
- scan matching passed its acceptance gates on only 22 % of updates; rejection reasons are not recorded (and the flag is not refreshed for scans with <20 points), so this is a diagnostic indicator, not a failure rate

**Limitations**

- Single run: no repeatability or variance claims.
- Ground truth = CoppeliaSim body pose (exact, same timestamps; no alignment applied because SLAM starts at the GT start pose).
- Map reference = occupancy grid built from the same depth stream using the GT pose: it isolates SLAM pose error, but is limited by what the sensor saw; it is not an independent CAD floor plan.
- Simulation only: sensor noise, gait slip and lighting are simulated, not measured on hardware.

**Integrity checks** (`audit.py`)

| Check | Result | Detail |
|---|---|---|
| No ground-truth pose API in SLAM, mapping, perception, planning or odometry (static source scan) | pass | 6 files + odometry clean; gyro reads orientation delta only |
| SLAMAgent does not subscribe to pose/ground_truth in closed-loop mode | pass | subscribes to: control/actuation, sensor/scan, sensor/thumb |
| Estimated trajectory is not a copy of ground truth | pass | estimate differs from ground truth (max 41.8 cm over 14401 steps) |
| Run navigated on the SLAM pose (not ground truth) | pass | config.ground_truth_nav = False |

**Metric definitions**

- frame: 2D world frame of the CoppeliaSim scene (x, y in m, yaw in rad, CCW); SLAM initialised at the GT start pose, no trajectory alignment
- ate: per-step Euclidean position error |p_est - p_gt| over every control step; RMSE/mean/median/max
- final_drift_pct: final position error / GT path length * 100
- rpe: distance-based relative pose error over 1.0 m GT segments, relative motion in the start body frame; trans RMSE (m), mean trans error as % of segment, rot RMSE (deg/m)
- icp_acceptance: share of SLAM updates whose scan match passed the acceptance gates (else odometry-only)
- tracking_loss_episode: >= 10 consecutive rejected SLAM updates
- relocalized: tracking-loss episode followed by an accepted loop closure within 30 s
- wall_accuracy/completion: mean distance SLAM->reference occupied cells / reference->SLAM (m)
- precision/recall/f1: occupied-cell agreement with reference grid within tolerance_m
- free_space_iou: IoU of free cells between SLAM grid and reference grid
- latency: wall-clock time of each agent.step() call (ms), p50/p95/max over all calls
- real_time_factor: simulated seconds per wall-clock second

## 7. External benchmark comparison

Not yet done: added only after these measurements are verified.
