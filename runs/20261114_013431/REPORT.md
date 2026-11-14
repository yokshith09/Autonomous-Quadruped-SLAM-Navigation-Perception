# Baseline report: run 20261114_013431

Every number comes from this run's own files. Measured = computed from telemetry or ground truth; estimate / diagnostic indicator are labelled as such; n/a = could not be measured (reason shown).

## Run identification

| Item | Value |
|---|---|
| Started | 2026-11-14T01:34:31 |
| Command | `main_agentic.py --headless --max-time 900` |
| Code | git 902c96d (+ uncommitted changes at run time) |
| CPU / GPU | AMD Ryzen 7 7735HS with Radeon Graphics (16 threads) / NVIDIA GeForce RTX 3050 6GB Laptop GPU, 6144 MiB |
| Platform | Windows-11-10.0.26300-SP0, Python 3.12.10 |
| Configuration | `{"max_time_s": 900.0, "max_steps": null, "grid_res_m": 0.05, "map_bounds_m": [-5.5, 5.5, -5.5, 5.5], "ground_truth_nav": false, "headless": true, "agent_rates_hz": {"PerceptionAgent": 10.0, "SLAMAgent": 10.0, "MappingAgent": 10.0, "FrontierAgent": 1.0, "PlanningAgent": 5.0, "LocomotionAgent": 20.0, "SupervisorAgent": 2.0}}` |
| Ground truth | CoppeliaSim /Base pose every control step (used for scoring only) |
| Runs in this report | 1 (no variability can be shown) |

| Source data | File | Detail |
|---|---|---|
| raw SLAM input recording | recording.npz | 24 MB, sha256 ca2c128113b23cb8... |
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
| Simulated time | 167 s | robot time |
| Wall-clock time | 184 s (real-time factor 0.91) | how long the computer needed; < 1 means slower than real time |
| Distance travelled | 9.3 m | path length from ground truth |
| Average speed | 0.056 m/s | distance / simulated time |
| Rooms completed / discovered | 0 / 4 | rooms the mapper segmented |
| Doorways detected | 10 | knowledge-graph entries; they accumulate as room segmentation changes, so this over-counts real doorways |
| Explored free area | 24.3 m2 | free cells x 0.05 m x 0.05 m |
| Grid observed | 20.7 % | of the 11 x 11 m map box, not of the arena |
| Stuck events / unreachable targets | 0 / 0 | watchdog and planner events |
| Human interventions | n/a (no intervention channel exists in the stack; only the 'q' key / external stop, recorded in end_reason) |  |
| Collisions | n/a (CoppeliaSim contact sensing is not wired into the stack) |  |

## 2. Agent and component performance

### 2a. Operational performance (how the agents ran, not whether they helped)

| Agent | Rate (Hz) | Calls | Errors | p50 (ms) | p95 (ms) | max (ms) | Total (s) | Share | Task metric |
|---|---|---|---|---|---|---|---|---|---|
| PerceptionAgent | 10 | 3342 | 0 | 4.04 | 8.51 | 181 | 17 | 25 % | median 338 scan pts/update |
| SLAMAgent | 10 | 3342 | 0 | 5.40 | 10.36 | 107 | 18 | 27 % | 44.3% scans matched, 4 loop closures |
| MappingAgent | 10 | 3342 | 0 | 0.36 | 2.11 | 479 | 3 | 4 % | 20.7% of grid observed |
| FrontierAgent | 1 | 335 | 0 | 6.55 | 11.47 | 23 | 2 | 4 % | 0 rooms completed |
| PlanningAgent | 5 | 669 | 0 | 0.17 | 4.42 | 72 | 1 | 1 % | 27 paths, 0 unreachable |
| LocomotionAgent | 20 | 3342 | 0 | 7.86 | 10.97 | 19 | 27 | 40 % | 0 stuck/watchdog events |
| SupervisorAgent | 2 | 335 | 0 | 0.03 | 0.09 | 0 | 0 | 0 % | 30 events logged, 4 rooms in graph |

Whole control-loop step: p50 22 ms, p95 68 ms, max 957 ms. Agent time is 68 s of 184 s wall time; the rest is simulator stepping, image transfer, HUD and per-step map queries outside `agent.step()` (split not measured).

### 2b. Contribution to the system result (offline replay of this run's recording, one component removed)

n/a (ablation not run: `python -m evaluation.ablate runs/<run_id>`)

## 3. Localization performance (independent of mapping)

Frame: 2D world frame of the simulator scene (x, y in m, yaw in rad). SLAM starts at the true start pose, so no trajectory alignment is applied. Evaluated at every control step (0.05 s) against simulator ground truth.

| Metric | Value | Meaning |
|---|---|---|
| ATE RMSE | 23.2 cm | typical position error over the whole run |
| ATE median / max | 20.6 / 39.6 cm |  |
| Final position error | 37.1 cm = 4.00 % of path | end-of-run drift |
| RPE translation (1 m segments) | 3.9 cm RMSE, mean 3.2 % of segment | local accuracy, independent of accumulated drift |
| RPE rotation | 2.54 deg/m |  |
| Heading error mean / max | 5.26 / 16.3 deg |  |
| Error first above 0.5 m / 1 m | n/a / n/a | n/a = never reached |
| Scan-match acceptance (diagnostic) | 44.3 % of 3341 updates | see warning in section 6 |
| Tracking-loss episodes (diagnostic) | 15 (41 s total) | >= 10 consecutive rejected updates |
| Followed by a loop closure within 30 s | 3 | relocalization proxy |
| Loop closures accepted | 4 |  |
| Odometry distance vs true | 9.3 m (0.3 %) | diagnostic only: ground-truth path length also counts body motion the gait did not command, and the odometry-only replay in 2b has a low local error (RPE), so this is not a clean odometry scale error |

## 4. Mapping performance (independent of localization)

Reference = occupancy grid built from the same depth stream with the ground-truth pose (not an independent floor plan). Map size, coverage and accuracy are separate rows.

| Metric | Value | Meaning |
|---|---|---|
| Coverage: explored free area | 24.3 m2 | size of what was seen |
| Wall accuracy | 18.7 cm | SLAM walls to reference walls (lower is better) |
| Wall completion | 19.7 cm | reference walls to SLAM walls (lower is better) |
| Completion ratio (5 cm) | 25.6 % | reference wall cells matched within 5 cm |
| Chamfer / Hausdorff-95 | 19.2 / 58.5 cm |  |
| Precision / recall / F1 | 56.7 / 56.5 / 56.6 % | tolerance 15 cm |
| Free-space IoU | 58.4 % |  |
| Occupied cells SLAM / reference | 386 / 313 | more SLAM cells = thicker or smeared walls |
| Map update p50 / p95 | 0.4 / 2.1 ms |  |
| Map memory | 0.19 MB | occupancy grid |
| Frontiers remaining | 57 | unexplored boundaries left (see map_overlay.png) |

![map overlay](map_overlay.png)

### 4b. Against the real scene (wall and object geometry read from CoppeliaSim, scoring only)

| Metric | Value | Meaning |
|---|---|---|
| Map accuracy / completion | 10.9 / 165.6 cm | mapped obstacle cells to real ones / real to mapped (mean) |
| Map precision / recall / F1 | 72.0 / 12.0 / 20.6 % | within 15 cm of a real wall or object |
| Same, map built with the true pose | accuracy 0.1 cm, recall 17.5 % | what the sensor allows with perfect localization |
| Real floor mapped as free | 20.8 % of 103.9 m2 | true coverage |
| Free space mapped outside the arena | 0.00 m2 | impossible in reality; source of phantom frontiers |
| Object detections on a real object | 0 of 25 (0.0 %) | within 0.5 m of one of 0 real objects |
| Real objects with a detection nearby | 0 of 0 | inflated by the many scattered detections |
| 'Chair' labels on the real chair | 0 of 25 (scene has 0 chair) | chair label = blob 0.18-1.05 m across |

## 5. Computational performance

| Metric | Value |
|---|---|
| Process CPU mean / p95 | 49 / 72 % (of one core; CoppeliaSim not included) |
| Memory (RSS) mean / peak | 191 / 335 MB (includes the in-memory recording buffer) |
| GPU | NVIDIA GeForce RTX 3050 6GB Laptop GPU, 6144 MiB: n/a (no code path uses the GPU (NumPy/SciPy/OpenCV on CPU); nothing to attribute) |
| SLAM tracking p50 / p95 | 5.4 / 10.4 ms |
| Largest agent by total time | LocomotionAgent |
| Telemetry overhead (estimate) | 0.0039 ms/step, about 0.018 % of the median loop step |

## 6. Data quality and limitations

**Warnings**

- mission did not report completion (simulation stopped externally)
- scan matching passed its acceptance gates on only 44 % of updates; rejection reasons are not recorded (and the flag is not refreshed for scans with <20 points), so this is a diagnostic indicator, not a failure rate

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
| Estimated trajectory is not a copy of ground truth | pass | estimate differs from ground truth (max 39.6 cm over 3342 steps) |
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
