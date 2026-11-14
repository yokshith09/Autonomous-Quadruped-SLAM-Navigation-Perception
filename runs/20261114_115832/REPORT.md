# Baseline report: run 20261114_115832

Every number comes from this run's own files. Measured = computed from telemetry or ground truth; estimate / diagnostic indicator are labelled as such; n/a = could not be measured (reason shown).

## Run identification

| Item | Value |
|---|---|
| Started | 2026-11-14T11:58:32 |
| Command | `main_agentic.py --headless --max-time 3600 --stop-on-complete` |
| Code | git 2ace9f7 (+ uncommitted changes at run time) |
| CPU / GPU | AMD Ryzen 7 7735HS with Radeon Graphics (16 threads) / NVIDIA GeForce RTX 3050 6GB Laptop GPU, 6144 MiB |
| Platform | Windows-11-10.0.26300-SP0, Python 3.12.10 |
| Configuration | `{"max_time_s": 3600.0, "max_steps": null, "grid_res_m": 0.05, "map_bounds_m": [-8.25, 8.25, -8.25, 8.25], "ground_truth_nav": false, "headless": true, "agent_rates_hz": {"PerceptionAgent": 10.0, "SLAMAgent": 10.0, "MappingAgent": 10.0, "FrontierAgent": 1.0, "PlanningAgent": 5.0, "LocomotionAgent": 20.0, "SupervisorAgent": 2.0}}` |
| Ground truth | CoppeliaSim /Base pose every control step (used for scoring only) |
| Runs in this report | 1 (no variability can be shown) |

| Source data | File | Detail |
|---|---|---|
| raw SLAM input recording | recording.npz | 330 MB, sha256 1d3952ee6c5d5308... |
| per-step ground-truth and estimated pose | trajectory.csv |  |
| agent table | agents.csv |  |
| all metrics | metrics.json |  |
| run manifest | manifest.json |  |
| reference/SLAM grids, knowledge graph | artifacts |  |

## 1. Overall robot performance: what the robot achieved

| Metric | Value | Meaning |
|---|---|---|
| Outcome | simulation stopped externally | why the run ended |
| Task success | no | mission reported all reachable frontiers explored |
| Simulated time | 2380 s | robot time |
| Wall-clock time | 3858 s (real-time factor 0.62) | how long the computer needed; < 1 means slower than real time |
| Distance travelled | 125.2 m | path length from ground truth |
| Average speed | 0.053 m/s | distance / simulated time |
| Rooms completed / discovered | 3 / 10 | rooms the mapper segmented |
| Doorways detected | 57 | knowledge-graph entries; they accumulate as room segmentation changes, so this over-counts real doorways |
| Explored free area | 232.2 m2 | free cells x 0.05 m x 0.05 m |
| Grid observed | 87.4 % | of the 11 x 11 m map box, not of the arena |
| Stuck events / unreachable targets | 2 / 1 | watchdog and planner events |
| Human interventions | n/a (no intervention channel exists in the stack; only the 'q' key / external stop, recorded in end_reason) |  |
| Collisions | 0 |  |

## 2. Agent and component performance

### 2a. Operational performance (how the agents ran, not whether they helped)

| Agent | Rate (Hz) | Calls | Errors | p50 (ms) | p95 (ms) | max (ms) | Total (s) | Share | Task metric |
|---|---|---|---|---|---|---|---|---|---|
| PerceptionAgent | 10 | 47611 | 0 | 4.74 | 9.16 | 52 | 265 | 20 % | median 236 scan pts/update |
| SLAMAgent | 10 | 47611 | 0 | 7.03 | 23.95 | 166 | 383 | 28 % | 24.1% scans matched, 27 loop closures |
| MappingAgent | 10 | 47611 | 0 | 0.88 | 4.82 | 640 | 94 | 7 % | 87.4% of grid observed |
| FrontierAgent | 1 | 4762 | 0 | 9.43 | 90.58 | 174 | 166 | 12 % | 3 rooms completed |
| PlanningAgent | 5 | 9523 | 0 | 0.30 | 5.13 | 4616 | 59 | 4 % | 175 paths, 1 unreachable |
| LocomotionAgent | 20 | 47611 | 0 | 7.70 | 11.05 | 71 | 376 | 28 % | 2 stuck/watchdog events |
| SupervisorAgent | 2 | 4762 | 0 | 0.08 | 0.17 | 1048 | 1 | 0 % | 150 events logged, 10 rooms in graph |

Whole control-loop step: p50 27 ms, p95 233 ms, max 4860 ms. Agent time is 1344 s of 3858 s wall time; the rest is simulator stepping, image transfer, HUD and per-step map queries outside `agent.step()` (split not measured).

### 2b. Contribution to the system result (offline replay of this run's recording, one component removed)

n/a (ablation not run: `python -m evaluation.ablate runs/<run_id>`)

## 3. Localization performance (independent of mapping)

Frame: 2D world frame of the simulator scene (x, y in m, yaw in rad). SLAM starts at the true start pose, so no trajectory alignment is applied. Evaluated at every control step (0.05 s) against simulator ground truth.

| Metric | Value | Meaning |
|---|---|---|
| ATE RMSE | 27.4 cm | typical position error over the whole run |
| ATE median / max | 20.4 / 62.1 cm |  |
| Final position error | 42.1 cm = 0.34 % of path | end-of-run drift |
| RPE translation (1 m segments) | 4.4 cm RMSE, mean 3.6 % of segment | local accuracy, independent of accumulated drift |
| RPE rotation | 0.67 deg/m |  |
| Heading error mean / max | 0.74 / 4.9 deg |  |
| Error first above 0.5 m / 1 m | 1524 s / n/a | n/a = never reached |
| Scan-match acceptance (diagnostic) | 24.1 % of 47610 updates | see warning in section 6 |
| Tracking-loss episodes (diagnostic) | 384 (1231 s total) | >= 10 consecutive rejected updates |
| Followed by a loop closure within 30 s | 90 | relocalization proxy |
| Loop closures accepted | 27 |  |
| Odometry distance vs true | 117.3 m (-6.3 %) | diagnostic only: ground-truth path length also counts body motion the gait did not command, and the odometry-only replay in 2b has a low local error (RPE), so this is not a clean odometry scale error |

## 4. Mapping performance (independent of localization)

Reference = occupancy grid built from the same depth stream with the ground-truth pose (not an independent floor plan). Map size, coverage and accuracy are separate rows.

| Metric | Value | Meaning |
|---|---|---|
| Coverage: explored free area | 232.2 m2 | size of what was seen |
| Wall accuracy | 14.4 cm | SLAM walls to reference walls (lower is better) |
| Wall completion | 12.5 cm | reference walls to SLAM walls (lower is better) |
| Completion ratio (5 cm) | 29.3 % | reference wall cells matched within 5 cm |
| Chamfer / Hausdorff-95 | 13.4 / 36.1 cm |  |
| Precision / recall / F1 | 63.0 / 71.1 / 66.8 % | tolerance 15 cm |
| Free-space IoU | 88.6 % |  |
| Occupied cells SLAM / reference | 2847 / 2306 | more SLAM cells = thicker or smeared walls |
| Map update p50 / p95 | 0.9 / 4.8 ms |  |
| Map memory | 0.42 MB | occupancy grid |
| Frontiers remaining | 139 | unexplored boundaries left (see map_overlay.png) |

![map overlay](map_overlay.png)

### 4b. Against the real scene (wall and object geometry read from CoppeliaSim, scoring only)

| Metric | Value | Meaning |
|---|---|---|
| Map accuracy / completion | 10.4 / 18.8 cm | mapped obstacle cells to real ones / real to mapped (mean) |
| Map precision / recall / F1 | 72.8 / 57.1 / 64.0 % | within 15 cm of a real wall or object |
| Same, map built with the true pose | accuracy 0.6 cm, recall 76.7 % | what the sensor allows with perfect localization |
| Real floor mapped as free | 91.2 % of 234.0 m2 | true coverage |
| Free space mapped outside the arena | 7.69 m2 | impossible in reality; source of phantom frontiers |
| Object detections on a real object | 0 of 106 (0.0 %) | within 0.5 m of one of 0 real objects |
| Real objects with a detection nearby | 0 of 0 | inflated by the many scattered detections |
| 'Chair' labels on the real chair | 0 of 106 (scene has 0 chair) | chair label = blob 0.18-1.05 m across |

## 5. Computational performance

| Metric | Value |
|---|---|
| Process CPU mean / p95 | 71 / 95 % (of one core; CoppeliaSim not included) |
| Memory (RSS) mean / peak | 198 / 939 MB (includes the in-memory recording buffer) |
| GPU | NVIDIA GeForce RTX 3050 6GB Laptop GPU, 6144 MiB: n/a (no code path uses the GPU (NumPy/SciPy/OpenCV on CPU); nothing to attribute) |
| SLAM tracking p50 / p95 | 7.0 / 24.0 ms |
| Largest agent by total time | SLAMAgent |
| Telemetry overhead (estimate) | 0.0048 ms/step, about 0.018 % of the median loop step |

## 6. Data quality and limitations

**Warnings**

- mission did not report completion (simulation stopped externally)
- scan matching passed its acceptance gates on only 24 % of updates; rejection reasons are not recorded (and the flag is not refreshed for scans with <20 points), so this is a diagnostic indicator, not a failure rate

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
| Estimated trajectory is not a copy of ground truth | pass | estimate differs from ground truth (max 62.1 cm over 47611 steps) |
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
