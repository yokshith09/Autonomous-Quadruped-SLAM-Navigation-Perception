# Baseline report: run 20261113_171057

Every number comes from this run's own files. Measured = computed from telemetry or ground truth; estimate / diagnostic indicator are labelled as such; n/a = could not be measured (reason shown).

## Run identification

| Item | Value |
|---|---|
| Started | 2026-11-13T17:10:57 |
| Command | `main_agentic.py --headless` |
| Code | git 15c7ed6 |
| CPU / GPU | AMD Ryzen 7 7735HS with Radeon Graphics (16 threads) / NVIDIA GeForce RTX 3050 6GB Laptop GPU, 6144 MiB |
| Platform | Windows-11-10.0.26300-SP0, Python 3.12.10 |
| Configuration | `{"max_time_s": 3000.0, "max_steps": null, "grid_res_m": 0.05, "map_bounds_m": [-5.5, 5.5, -5.5, 5.5], "ground_truth_nav": false, "headless": true, "agent_rates_hz": {"PerceptionAgent": 10.0, "SLAMAgent": 10.0, "MappingAgent": 10.0, "FrontierAgent": 1.0, "PlanningAgent": 5.0, "LocomotionAgent": 20.0, "SupervisorAgent": 2.0}}` |
| Ground truth | CoppeliaSim /Base pose every control step (used for scoring only) |
| Runs in this report | 1 (no variability can be shown) |

| Source data | File | Detail |
|---|---|---|
| raw SLAM input recording | recording.npz | 471 MB, sha256 d3764dd82959d022... |
| per-step ground-truth and estimated pose | trajectory.csv |  |
| agent table | agents.csv |  |
| all metrics | metrics.json |  |
| run manifest | manifest.json |  |
| reference/SLAM grids, knowledge graph | artifacts |  |

## 1. Overall robot performance: what the robot achieved

| Metric | Value | Meaning |
|---|---|---|
| Outcome | simulation stopped externally | why the run ended |
| Task success | yes | mission reported all reachable frontiers explored |
| Simulated time | 2108 s | robot time |
| Wall-clock time | 3656 s (real-time factor 0.58) | how long the computer needed; < 1 means slower than real time |
| Distance travelled | 182.7 m | path length from ground truth |
| Average speed | 0.087 m/s | distance / simulated time |
| Rooms completed / discovered | 9 / 14 | rooms the mapper segmented |
| Doorways detected | 59 | knowledge-graph entries; they accumulate as room segmentation changes, so this over-counts real doorways |
| Explored free area | 87.6 m2 | free cells x 0.05 m x 0.05 m |
| Grid observed | 82.1 % | of the 11 x 11 m map box, not of the arena |
| Stuck events / unreachable targets | 0 / 5 | watchdog and planner events |
| Human interventions | n/a (no intervention channel exists in the stack; only the 'q' key / external stop, recorded in end_reason) |  |
| Collisions | n/a (CoppeliaSim contact sensing is not wired into the stack) |  |

## 2. Agent and component performance

### 2a. Operational performance (how the agents ran, not whether they helped)

| Agent | Rate (Hz) | Calls | Errors | p50 (ms) | p95 (ms) | max (ms) | Total (s) | Share | Task metric |
|---|---|---|---|---|---|---|---|---|---|
| PerceptionAgent | 10 | 42162 | 0 | 5.07 | 10.46 | 38 | 262 | 18 % | median 622 scan pts/update |
| SLAMAgent | 10 | 42162 | 0 | 12.89 | 24.79 | 83 | 587 | 40 % | 24.3% scans matched, 107 loop closures |
| MappingAgent | 10 | 42162 | 0 | 0.66 | 5.01 | 472 | 57 | 4 % | 82.1% of grid observed |
| FrontierAgent | 1 | 4217 | 0 | 24.99 | 37.62 | 69 | 104 | 7 % | 9 rooms completed |
| PlanningAgent | 5 | 8433 | 0 | 0.38 | 6.13 | 1772 | 28 | 2 % | 243 paths, 5 unreachable |
| LocomotionAgent | 20 | 42162 | 0 | 9.88 | 13.62 | 249 | 428 | 29 % | 0 stuck/watchdog events |
| SupervisorAgent | 2 | 4217 | 0 | 0.13 | 0.26 | 3 | 1 | 0 % | 291 events logged, 14 rooms in graph |

Whole control-loop step: p50 35 ms, p95 181 ms, max 1959 ms. Agent time is 1468 s of 3656 s wall time; the rest is simulator stepping, image transfer, HUD and per-step map queries outside `agent.step()` (split not measured).

### 2b. Contribution to the system result (offline replay of this run's recording, one component removed)

n/a (ablation not run: `python -m evaluation.ablate runs/<run_id>`)

## 3. Localization performance (independent of mapping)

Frame: 2D world frame of the simulator scene (x, y in m, yaw in rad). SLAM starts at the true start pose, so no trajectory alignment is applied. Evaluated at every control step (0.05 s) against simulator ground truth.

| Metric | Value | Meaning |
|---|---|---|
| ATE RMSE | 30.7 cm | typical position error over the whole run |
| ATE median / max | 17.1 / 78.8 cm |  |
| Final position error | 72.6 cm = 0.40 % of path | end-of-run drift |
| RPE translation (1 m segments) | 9.3 cm RMSE, mean 5.7 % of segment | local accuracy, independent of accumulated drift |
| RPE rotation | 1.30 deg/m |  |
| Heading error mean / max | 0.75 / 4.8 deg |  |
| Error first above 0.5 m / 1 m | 1838 s / n/a | n/a = never reached |
| Scan-match acceptance (diagnostic) | 24.3 % of 42161 updates | see warning in section 6 |
| Tracking-loss episodes (diagnostic) | 683 (933 s total) | >= 10 consecutive rejected updates |
| Followed by a loop closure within 30 s | 443 | relocalization proxy |
| Loop closures accepted | 107 |  |
| Odometry distance vs true | 159.7 m (-12.6 %) | diagnostic only: ground-truth path length also counts body motion the gait did not command, and the odometry-only replay in 2b has a low local error (RPE), so this is not a clean odometry scale error |

## 4. Mapping performance (independent of localization)

Reference = occupancy grid built from the same depth stream with the ground-truth pose (not an independent floor plan). Map size, coverage and accuracy are separate rows.

| Metric | Value | Meaning |
|---|---|---|
| Coverage: explored free area | 87.6 m2 | size of what was seen |
| Wall accuracy | 10.0 cm | SLAM walls to reference walls (lower is better) |
| Wall completion | 6.1 cm | reference walls to SLAM walls (lower is better) |
| Completion ratio (5 cm) | 66.9 % | reference wall cells matched within 5 cm |
| Chamfer / Hausdorff-95 | 8.1 / 25.0 cm |  |
| Precision / recall / F1 | 82.6 / 96.0 / 88.8 % | tolerance 15 cm |
| Free-space IoU | 84.0 % |  |
| Occupied cells SLAM / reference | 5025 / 3367 | more SLAM cells = thicker or smeared walls |
| Map update p50 / p95 | 0.7 / 5.0 ms |  |
| Map memory | 0.19 MB | occupancy grid |
| Frontiers remaining | 74 | unexplored boundaries left (see map_overlay.png) |

![map overlay](map_overlay.png)

### 4b. Against the real scene (wall and object geometry read from CoppeliaSim, scoring only)

| Metric | Value | Meaning |
|---|---|---|
| Map accuracy / completion | 7.8 / 9.2 cm | mapped obstacle cells to real ones / real to mapped (mean) |
| Map precision / recall / F1 | 85.9 / 87.5 / 86.7 % | within 15 cm of a real wall or object |
| Same, map built with the true pose | accuracy 0.2 cm, recall 90.7 % | what the sensor allows with perfect localization |
| Real floor mapped as free | 91.2 % of 85.3 m2 | true coverage |
| Free space mapped outside the arena | 1.78 m2 | impossible in reality; source of phantom frontiers |
| Object detections on a real object | 19 of 105 (18.1 %) | within 0.5 m of one of 13 real objects |
| Real objects with a detection nearby | 13 of 13 | inflated by the many scattered detections |
| 'Chair' labels on the real chair | 1 of 105 (scene has 1 chair) | chair label = blob 0.18-1.05 m across |

## 5. Computational performance

| Metric | Value |
|---|---|
| Process CPU mean / p95 | 62 / 81 % (of one core; CoppeliaSim not included) |
| Memory (RSS) mean / peak | 201 / 1058 MB (includes the in-memory recording buffer) |
| GPU | NVIDIA GeForce RTX 3050 6GB Laptop GPU, 6144 MiB: n/a (no code path uses the GPU (NumPy/SciPy/OpenCV on CPU); nothing to attribute) |
| SLAM tracking p50 / p95 | 12.9 / 24.8 ms |
| Largest agent by total time | SLAMAgent |
| Telemetry overhead (estimate) | 0.0061 ms/step, about 0.018 % of the median loop step |

## 6. Data quality and limitations

**Warnings**

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
| Estimated trajectory is not a copy of ground truth | pass | estimate differs from ground truth (max 78.8 cm over 42162 steps) |
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
