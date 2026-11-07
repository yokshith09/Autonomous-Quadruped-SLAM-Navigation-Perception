# Baseline report: run 20261107_112958

Every number comes from this run's own files. Measured = computed from telemetry or ground truth; estimate / diagnostic indicator are labelled as such; n/a = could not be measured (reason shown).

## Run identification

| Item | Value |
|---|---|
| Started | 2026-11-07T11:29:58 |
| Command | `main_agentic.py --headless` |
| Code | git 00361c7 |
| CPU / GPU | AMD Ryzen 7 7735HS with Radeon Graphics (16 threads) / NVIDIA GeForce RTX 3050 6GB Laptop GPU, 6144 MiB |
| Platform | Windows-11-10.0.26300-SP0, Python 3.12.10 |
| Configuration | `{"max_time_s": 3600.0, "max_steps": null, "grid_res_m": 0.05, "map_bounds_m": [-5.5, 5.5, -5.5, 5.5], "ground_truth_nav": false, "headless": true, "agent_rates_hz": {"PerceptionAgent": 10.0, "SLAMAgent": 10.0, "MappingAgent": 10.0, "FrontierAgent": 1.0, "PlanningAgent": 5.0, "LocomotionAgent": 20.0, "SupervisorAgent": 2.0}}` |
| Ground truth | CoppeliaSim /Base pose every control step (used for scoring only) |
| Runs in this report | 1 (no variability can be shown) |

| Source data | File | Detail |
|---|---|---|
| raw SLAM input recording | recording.npz | 297 MB, sha256 8796b3a732966a3e... |
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
| Simulated time | 1504 s | robot time |
| Wall-clock time | 2661 s (real-time factor 0.57) | how long the computer needed; < 1 means slower than real time |
| Distance travelled | 136.6 m | path length from ground truth |
| Average speed | 0.091 m/s | distance / simulated time |
| Rooms completed / discovered | 11 / 14 | rooms the mapper segmented |
| Doorways detected | 22 | knowledge-graph entries; they accumulate as room segmentation changes, so this over-counts real doorways |
| Explored free area | 84.4 m2 | free cells x 0.05 m x 0.05 m |
| Grid observed | 78.1 % | of the 11 x 11 m map box, not of the arena |
| Stuck events / unreachable targets | 11 / 37 | watchdog and planner events |
| Human interventions | n/a (no intervention channel exists in the stack; only the 'q' key / external stop, recorded in end_reason) |  |
| Collisions | n/a (CoppeliaSim contact sensing is not wired into the stack) |  |

## 2. Agent and component performance

### 2a. Operational performance (how the agents ran, not whether they helped)

| Agent | Rate (Hz) | Calls | Errors | p50 (ms) | p95 (ms) | max (ms) | Total (s) | Share | Task metric |
|---|---|---|---|---|---|---|---|---|---|
| PerceptionAgent | 10 | 30080 | 0 | 6.39 | 12.67 | 88 | 212 | 16 % | median 639 scan pts/update |
| SLAMAgent | 10 | 30080 | 0 | 10.76 | 24.28 | 109 | 371 | 28 % | 23.8% scans matched, 61 loop closures |
| MappingAgent | 10 | 30080 | 0 | 1.19 | 5.98 | 705 | 67 | 5 % | 78.1% of grid observed |
| FrontierAgent | 1 | 3008 | 0 | 153.97 | 253.42 | 347 | 456 | 34 % | 11 rooms completed |
| PlanningAgent | 5 | 6016 | 0 | 0.25 | 15.58 | 826 | 40 | 3 % | 562 paths, 37 unreachable |
| LocomotionAgent | 20 | 30080 | 0 | 6.50 | 9.60 | 246 | 200 | 15 % | 11 stuck/watchdog events |
| SupervisorAgent | 2 | 3008 | 0 | 0.02 | 0.03 | 0 | 0 | 0 % | 299 events logged, 14 rooms in graph |

Whole control-loop step: p50 30 ms, p95 277 ms, max 2076 ms. Agent time is 1346 s of 2661 s wall time; the rest is simulator stepping, image transfer, HUD and per-step map queries outside `agent.step()` (split not measured).

### 2b. Contribution to the system result (offline replay of this run's recording, one component removed)

n/a (ablation not run: `python -m evaluation.ablate runs/<run_id>`)

## 3. Localization performance (independent of mapping)

Frame: 2D world frame of the simulator scene (x, y in m, yaw in rad). SLAM starts at the true start pose, so no trajectory alignment is applied. Evaluated at every control step (0.05 s) against simulator ground truth.

| Metric | Value | Meaning |
|---|---|---|
| ATE RMSE | 49.9 cm | typical position error over the whole run |
| ATE median / max | 33.0 / 164.3 cm |  |
| Final position error | 158.2 cm = 1.16 % of path | end-of-run drift |
| RPE translation (1 m segments) | 7.2 cm RMSE, mean 4.6 % of segment | local accuracy, independent of accumulated drift |
| RPE rotation | 1.66 deg/m |  |
| Heading error mean / max | 4.56 / 27.4 deg |  |
| Error first above 0.5 m / 1 m | 881 s / 1362 s | n/a = never reached |
| Scan-match acceptance (diagnostic) | 23.8 % of 30078 updates | see warning in section 6 |
| Tracking-loss episodes (diagnostic) | 472 (671 s total) | >= 10 consecutive rejected updates |
| Followed by a loop closure within 30 s | 257 | relocalization proxy |
| Loop closures accepted | 61 |  |
| Odometry distance vs true | 116.6 m (-14.7 %) | diagnostic only: ground-truth path length also counts body motion the gait did not command, and the odometry-only replay in 2b has a low local error (RPE), so this is not a clean odometry scale error |

## 4. Mapping performance (independent of localization)

Reference = occupancy grid built from the same depth stream with the ground-truth pose (not an independent floor plan). Map size, coverage and accuracy are separate rows.

| Metric | Value | Meaning |
|---|---|---|
| Coverage: explored free area | 84.4 m2 | size of what was seen |
| Wall accuracy | 22.5 cm | SLAM walls to reference walls (lower is better) |
| Wall completion | 15.2 cm | reference walls to SLAM walls (lower is better) |
| Completion ratio (5 cm) | 24.8 % | reference wall cells matched within 5 cm |
| Chamfer / Hausdorff-95 | 18.9 / 47.2 cm |  |
| Precision / recall / F1 | 42.9 / 57.4 / 49.1 % | tolerance 15 cm |
| Free-space IoU | 79.0 % |  |
| Occupied cells SLAM / reference | 4399 / 3180 | more SLAM cells = thicker or smeared walls |
| Map update p50 / p95 | 1.2 / 6.0 ms |  |
| Map memory | 0.19 MB | occupancy grid |
| Frontiers remaining | 48 | unexplored boundaries left (see map_overlay.png) |

![map overlay](map_overlay.png)

### 4b. Against the real scene (wall and object geometry read from CoppeliaSim, scoring only)

| Metric | Value | Meaning |
|---|---|---|
| Map accuracy / completion | 19.3 / 16.4 cm | mapped obstacle cells to real ones / real to mapped (mean) |
| Map precision / recall / F1 | 51.2 / 54.4 / 52.7 % | within 15 cm of a real wall or object |
| Same, map built with the true pose | accuracy 0.2 cm, recall 89.8 % | what the sensor allows with perfect localization |
| Real floor mapped as free | 86.4 % of 85.2 m2 | true coverage |
| Free space mapped outside the arena | 0.93 m2 | impossible in reality; source of phantom frontiers |
| Object detections on a real object | 23 of 113 (20.4 %) | within 0.5 m of one of 13 real objects |
| Real objects with a detection nearby | 13 of 13 | inflated by the many scattered detections |
| 'Chair' labels on the real chair | 2 of 113 (scene has 1 chair) | chair label = blob 0.18-1.05 m across |

## 5. Computational performance

| Metric | Value |
|---|---|
| Process CPU mean / p95 | 70 / 90 % (of one core; CoppeliaSim not included) |
| Memory (RSS) mean / peak | 1213 / 2465 MB (includes the in-memory recording buffer) |
| GPU | NVIDIA GeForce RTX 3050 6GB Laptop GPU, 6144 MiB: n/a (no code path uses the GPU (NumPy/SciPy/OpenCV on CPU); nothing to attribute) |
| SLAM tracking p50 / p95 | 10.8 / 24.3 ms |
| Largest agent by total time | FrontierAgent |
| Telemetry overhead (estimate) | 0.0057 ms/step, about 0.019 % of the median loop step |

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
| SLAMAgent does not subscribe to pose/ground_truth in closed-loop mode | pass | subscribes to: control/actuation, sensor/scan |
| Estimated trajectory is not a copy of ground truth | pass | estimate differs from ground truth (max 164.3 cm over 30080 steps) |
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
