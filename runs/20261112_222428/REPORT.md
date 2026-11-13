# Baseline report: run 20261112_222428

Every number comes from this run's own files. Measured = computed from telemetry or ground truth; estimate / diagnostic indicator are labelled as such; n/a = could not be measured (reason shown).

## Run identification

| Item | Value |
|---|---|
| Started | 2026-11-12T22:24:28 |
| Command | `main_agentic.py --headless` |
| Code | git 12bce31 |
| CPU / GPU | AMD Ryzen 7 7735HS with Radeon Graphics (16 threads) / NVIDIA GeForce RTX 3050 6GB Laptop GPU, 6144 MiB |
| Platform | Windows-11-10.0.26300-SP0, Python 3.12.10 |
| Configuration | `{"max_time_s": 3000.0, "max_steps": null, "grid_res_m": 0.05, "map_bounds_m": [-5.5, 5.5, -5.5, 5.5], "ground_truth_nav": false, "headless": true, "agent_rates_hz": {"PerceptionAgent": 10.0, "SLAMAgent": 10.0, "MappingAgent": 10.0, "FrontierAgent": 1.0, "PlanningAgent": 5.0, "LocomotionAgent": 20.0, "SupervisorAgent": 2.0}}` |
| Ground truth | CoppeliaSim /Base pose every control step (used for scoring only) |
| Runs in this report | 1 (no variability can be shown) |

| Source data | File | Detail |
|---|---|---|
| raw SLAM input recording | recording.npz | 208 MB, sha256 0b04911460b8f158... |
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
| Simulated time | 913 s | robot time |
| Wall-clock time | 1516 s (real-time factor 0.60) | how long the computer needed; < 1 means slower than real time |
| Distance travelled | 79.2 m | path length from ground truth |
| Average speed | 0.087 m/s | distance / simulated time |
| Rooms completed / discovered | 5 / 9 | rooms the mapper segmented |
| Doorways detected | 27 | knowledge-graph entries; they accumulate as room segmentation changes, so this over-counts real doorways |
| Explored free area | 79.3 m2 | free cells x 0.05 m x 0.05 m |
| Grid observed | 71.9 % | of the 11 x 11 m map box, not of the arena |
| Stuck events / unreachable targets | 0 / 2 | watchdog and planner events |
| Human interventions | n/a (no intervention channel exists in the stack; only the 'q' key / external stop, recorded in end_reason) |  |
| Collisions | n/a (CoppeliaSim contact sensing is not wired into the stack) |  |

## 2. Agent and component performance

### 2a. Operational performance (how the agents ran, not whether they helped)

| Agent | Rate (Hz) | Calls | Errors | p50 (ms) | p95 (ms) | max (ms) | Total (s) | Share | Task metric |
|---|---|---|---|---|---|---|---|---|---|
| PerceptionAgent | 10 | 18265 | 0 | 5.29 | 11.57 | 40 | 116 | 18 % | median 627 scan pts/update |
| SLAMAgent | 10 | 18265 | 0 | 13.39 | 33.51 | 92 | 284 | 44 % | 25.6% scans matched, 46 loop closures |
| MappingAgent | 10 | 18265 | 0 | 0.65 | 4.74 | 715 | 24 | 4 % | 71.9% of grid observed |
| FrontierAgent | 1 | 1827 | 0 | 18.40 | 31.70 | 70 | 33 | 5 % | 5 rooms completed |
| PlanningAgent | 5 | 3653 | 0 | 0.28 | 6.70 | 536 | 8 | 1 % | 276 paths, 2 unreachable |
| LocomotionAgent | 20 | 18265 | 0 | 9.39 | 15.21 | 343 | 185 | 28 % | 0 stuck/watchdog events |
| SupervisorAgent | 2 | 1827 | 0 | 0.09 | 0.21 | 0 | 0 | 0 % | 169 events logged, 9 rooms in graph |

Whole control-loop step: p50 36 ms, p95 153 ms, max 2257 ms. Agent time is 650 s of 1516 s wall time; the rest is simulator stepping, image transfer, HUD and per-step map queries outside `agent.step()` (split not measured).

### 2b. Contribution to the system result (offline replay of this run's recording, one component removed)

n/a (ablation not run: `python -m evaluation.ablate runs/<run_id>`)

## 3. Localization performance (independent of mapping)

Frame: 2D world frame of the simulator scene (x, y in m, yaw in rad). SLAM starts at the true start pose, so no trajectory alignment is applied. Evaluated at every control step (0.05 s) against simulator ground truth.

| Metric | Value | Meaning |
|---|---|---|
| ATE RMSE | 28.3 cm | typical position error over the whole run |
| ATE median / max | 24.7 / 102.2 cm |  |
| Final position error | 97.3 cm = 1.23 % of path | end-of-run drift |
| RPE translation (1 m segments) | 11.6 cm RMSE, mean 5.4 % of segment | local accuracy, independent of accumulated drift |
| RPE rotation | 1.31 deg/m |  |
| Heading error mean / max | 0.76 / 5.8 deg |  |
| Error first above 0.5 m / 1 m | 897 s / 902 s | n/a = never reached |
| Scan-match acceptance (diagnostic) | 25.6 % of 18264 updates | see warning in section 6 |
| Tracking-loss episodes (diagnostic) | 318 (361 s total) | >= 10 consecutive rejected updates |
| Followed by a loop closure within 30 s | 162 | relocalization proxy |
| Loop closures accepted | 46 |  |
| Odometry distance vs true | 69.2 m (-12.6 %) | diagnostic only: ground-truth path length also counts body motion the gait did not command, and the odometry-only replay in 2b has a low local error (RPE), so this is not a clean odometry scale error |

## 4. Mapping performance (independent of localization)

Reference = occupancy grid built from the same depth stream with the ground-truth pose (not an independent floor plan). Map size, coverage and accuracy are separate rows.

| Metric | Value | Meaning |
|---|---|---|
| Coverage: explored free area | 79.3 m2 | size of what was seen |
| Wall accuracy | 12.2 cm | SLAM walls to reference walls (lower is better) |
| Wall completion | 8.3 cm | reference walls to SLAM walls (lower is better) |
| Completion ratio (5 cm) | 47.8 % | reference wall cells matched within 5 cm |
| Chamfer / Hausdorff-95 | 10.2 / 30.0 cm |  |
| Precision / recall / F1 | 72.9 / 86.5 / 79.1 % | tolerance 15 cm |
| Free-space IoU | 84.1 % |  |
| Occupied cells SLAM / reference | 3396 / 2506 | more SLAM cells = thicker or smeared walls |
| Map update p50 / p95 | 0.6 / 4.7 ms |  |
| Map memory | 0.19 MB | occupancy grid |
| Frontiers remaining | 56 | unexplored boundaries left (see map_overlay.png) |

![map overlay](map_overlay.png)

### 4b. Against the real scene (wall and object geometry read from CoppeliaSim, scoring only)

| Metric | Value | Meaning |
|---|---|---|
| Map accuracy / completion | 11.7 / 21.4 cm | mapped obstacle cells to real ones / real to mapped (mean) |
| Map precision / recall / F1 | 79.7 / 73.4 / 76.4 % | within 15 cm of a real wall or object |
| Same, map built with the true pose | accuracy 3.6 cm, recall 81.9 % | what the sensor allows with perfect localization |
| Real floor mapped as free | 82.3 % of 85.5 m2 | true coverage |
| Free space mapped outside the arena | 0.39 m2 | impossible in reality; source of phantom frontiers |
| Object detections on a real object | 18 of 77 (23.4 %) | within 0.5 m of one of 13 real objects |
| Real objects with a detection nearby | 12 of 13 | inflated by the many scattered detections |
| 'Chair' labels on the real chair | 3 of 77 (scene has 1 chair) | chair label = blob 0.18-1.05 m across |

## 5. Computational performance

| Metric | Value |
|---|---|
| Process CPU mean / p95 | 60 / 82 % (of one core; CoppeliaSim not included) |
| Memory (RSS) mean / peak | 206 / 510 MB (includes the in-memory recording buffer) |
| GPU | NVIDIA GeForce RTX 3050 6GB Laptop GPU, 6144 MiB: n/a (no code path uses the GPU (NumPy/SciPy/OpenCV on CPU); nothing to attribute) |
| SLAM tracking p50 / p95 | 13.4 / 33.5 ms |
| Largest agent by total time | SLAMAgent |
| Telemetry overhead (estimate) | 0.0059 ms/step, about 0.016 % of the median loop step |

## 6. Data quality and limitations

**Warnings**

- scan matching passed its acceptance gates on only 26 % of updates; rejection reasons are not recorded (and the flag is not refreshed for scans with <20 points), so this is a diagnostic indicator, not a failure rate

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
| Estimated trajectory is not a copy of ground truth | pass | estimate differs from ground truth (max 102.2 cm over 18265 steps) |
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
