# Baseline report: run 20261114_034037

Every number comes from this run's own files. Measured = computed from telemetry or ground truth; estimate / diagnostic indicator are labelled as such; n/a = could not be measured (reason shown).

## Run identification

| Item | Value |
|---|---|
| Started | 2026-11-14T03:40:37 |
| Command | `main_agentic.py --headless --max-time 1500` |
| Code | git 8345dca (+ uncommitted changes at run time) |
| CPU / GPU | AMD Ryzen 7 7735HS with Radeon Graphics (16 threads) / NVIDIA GeForce RTX 3050 6GB Laptop GPU, 6144 MiB |
| Platform | Windows-11-10.0.26300-SP0, Python 3.12.10 |
| Configuration | `{"max_time_s": 1500.0, "max_steps": null, "grid_res_m": 0.05, "map_bounds_m": [-8.25, 8.25, -8.25, 8.25], "ground_truth_nav": false, "headless": true, "agent_rates_hz": {"PerceptionAgent": 10.0, "SLAMAgent": 10.0, "MappingAgent": 10.0, "FrontierAgent": 1.0, "PlanningAgent": 5.0, "LocomotionAgent": 20.0, "SupervisorAgent": 2.0}}` |
| Ground truth | CoppeliaSim /Base pose every control step (used for scoring only) |
| Runs in this report | 1 (no variability can be shown) |

| Source data | File | Detail |
|---|---|---|
| raw SLAM input recording | recording.npz | 105 MB, sha256 91da9d0e39317752... |
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
| Simulated time | 756 s | robot time |
| Wall-clock time | 856 s (real-time factor 0.88) | how long the computer needed; < 1 means slower than real time |
| Distance travelled | 42.0 m | path length from ground truth |
| Average speed | 0.055 m/s | distance / simulated time |
| Rooms completed / discovered | 1 / 10 | rooms the mapper segmented |
| Doorways detected | 45 | knowledge-graph entries; they accumulate as room segmentation changes, so this over-counts real doorways |
| Explored free area | 131.4 m2 | free cells x 0.05 m x 0.05 m |
| Grid observed | 49.3 % | of the 11 x 11 m map box, not of the arena |
| Stuck events / unreachable targets | 0 / 1 | watchdog and planner events |
| Human interventions | n/a (no intervention channel exists in the stack; only the 'q' key / external stop, recorded in end_reason) |  |
| Collisions | n/a (CoppeliaSim contact sensing is not wired into the stack) |  |

## 2. Agent and component performance

### 2a. Operational performance (how the agents ran, not whether they helped)

| Agent | Rate (Hz) | Calls | Errors | p50 (ms) | p95 (ms) | max (ms) | Total (s) | Share | Task metric |
|---|---|---|---|---|---|---|---|---|---|
| PerceptionAgent | 10 | 15122 | 0 | 3.99 | 8.44 | 33 | 75 | 21 % | median 298 scan pts/update |
| SLAMAgent | 10 | 15122 | 0 | 5.29 | 13.47 | 100 | 86 | 24 % | 25.0% scans matched, 10 loop closures |
| MappingAgent | 10 | 15122 | 0 | 0.55 | 3.59 | 457 | 19 | 5 % | 49.3% of grid observed |
| FrontierAgent | 1 | 1513 | 0 | 38.84 | 49.24 | 113 | 52 | 15 % | 1 rooms completed |
| PlanningAgent | 5 | 3025 | 0 | 0.19 | 7.11 | 1160 | 6 | 2 % | 133 paths, 1 unreachable |
| LocomotionAgent | 20 | 15122 | 0 | 7.48 | 10.52 | 129 | 117 | 33 % | 0 stuck/watchdog events |
| SupervisorAgent | 2 | 1513 | 0 | 0.04 | 0.11 | 0 | 0 | 0 % | 80 events logged, 10 rooms in graph |

Whole control-loop step: p50 22 ms, p95 142 ms, max 1377 ms. Agent time is 356 s of 856 s wall time; the rest is simulator stepping, image transfer, HUD and per-step map queries outside `agent.step()` (split not measured).

### 2b. Contribution to the system result (offline replay of this run's recording, one component removed)

n/a (ablation not run: `python -m evaluation.ablate runs/<run_id>`)

## 3. Localization performance (independent of mapping)

Frame: 2D world frame of the simulator scene (x, y in m, yaw in rad). SLAM starts at the true start pose, so no trajectory alignment is applied. Evaluated at every control step (0.05 s) against simulator ground truth.

| Metric | Value | Meaning |
|---|---|---|
| ATE RMSE | 20.1 cm | typical position error over the whole run |
| ATE median / max | 13.1 / 48.0 cm |  |
| Final position error | 47.7 cm = 1.14 % of path | end-of-run drift |
| RPE translation (1 m segments) | 4.2 cm RMSE, mean 3.3 % of segment | local accuracy, independent of accumulated drift |
| RPE rotation | 0.70 deg/m |  |
| Heading error mean / max | 1.62 / 4.7 deg |  |
| Error first above 0.5 m / 1 m | n/a / n/a | n/a = never reached |
| Scan-match acceptance (diagnostic) | 25.0 % of 15121 updates | see warning in section 6 |
| Tracking-loss episodes (diagnostic) | 121 (314 s total) | >= 10 consecutive rejected updates |
| Followed by a loop closure within 30 s | 43 | relocalization proxy |
| Loop closures accepted | 10 |  |
| Odometry distance vs true | 39.2 m (-6.5 %) | diagnostic only: ground-truth path length also counts body motion the gait did not command, and the odometry-only replay in 2b has a low local error (RPE), so this is not a clean odometry scale error |

## 4. Mapping performance (independent of localization)

Reference = occupancy grid built from the same depth stream with the ground-truth pose (not an independent floor plan). Map size, coverage and accuracy are separate rows.

| Metric | Value | Meaning |
|---|---|---|
| Coverage: explored free area | 131.4 m2 | size of what was seen |
| Wall accuracy | 11.4 cm | SLAM walls to reference walls (lower is better) |
| Wall completion | 10.6 cm | reference walls to SLAM walls (lower is better) |
| Completion ratio (5 cm) | 38.9 % | reference wall cells matched within 5 cm |
| Chamfer / Hausdorff-95 | 11.0 / 31.6 cm |  |
| Precision / recall / F1 | 74.5 / 78.1 / 76.3 % | tolerance 15 cm |
| Free-space IoU | 76.9 % |  |
| Occupied cells SLAM / reference | 1477 / 1125 | more SLAM cells = thicker or smeared walls |
| Map update p50 / p95 | 0.5 / 3.6 ms |  |
| Map memory | 0.42 MB | occupancy grid |
| Frontiers remaining | 164 | unexplored boundaries left (see map_overlay.png) |

![map overlay](map_overlay.png)

### 4b. Against the real scene (wall and object geometry read from CoppeliaSim, scoring only)

| Metric | Value | Meaning |
|---|---|---|
| Map accuracy / completion | 7.7 / 51.2 cm | mapped obstacle cells to real ones / real to mapped (mean) |
| Map precision / recall / F1 | 80.8 / 40.2 / 53.7 % | within 15 cm of a real wall or object |
| Same, map built with the true pose | accuracy 0.6 cm, recall 47.5 % | what the sensor allows with perfect localization |
| Real floor mapped as free | 51.9 % of 234.0 m2 | true coverage |
| Free space mapped outside the arena | 3.49 m2 | impossible in reality; source of phantom frontiers |
| Object detections on a real object | 0 of 65 (0.0 %) | within 0.5 m of one of 0 real objects |
| Real objects with a detection nearby | 0 of 0 | inflated by the many scattered detections |
| 'Chair' labels on the real chair | 0 of 65 (scene has 0 chair) | chair label = blob 0.18-1.05 m across |

## 5. Computational performance

| Metric | Value |
|---|---|
| Process CPU mean / p95 | 66 / 86 % (of one core; CoppeliaSim not included) |
| Memory (RSS) mean / peak | 210 / 515 MB (includes the in-memory recording buffer) |
| GPU | NVIDIA GeForce RTX 3050 6GB Laptop GPU, 6144 MiB: n/a (no code path uses the GPU (NumPy/SciPy/OpenCV on CPU); nothing to attribute) |
| SLAM tracking p50 / p95 | 5.3 / 13.5 ms |
| Largest agent by total time | LocomotionAgent |
| Telemetry overhead (estimate) | 0.0037 ms/step, about 0.017 % of the median loop step |

## 6. Data quality and limitations

**Warnings**

- mission did not report completion (simulation stopped externally)
- scan matching passed its acceptance gates on only 25 % of updates; rejection reasons are not recorded (and the flag is not refreshed for scans with <20 points), so this is a diagnostic indicator, not a failure rate

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
| Estimated trajectory is not a copy of ground truth | pass | estimate differs from ground truth (max 48.0 cm over 15122 steps) |
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
