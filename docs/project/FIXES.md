# Fix register

Every fix needed after the baseline (run `20261024_210638`, git tag `baseline-code-run1`), with its evidence, planned change and
how it will be verified. Nothing here has been applied yet except where status says so. Verify each fix by re-running
`python main_agentic.py` and comparing `runs/<new>/REPORT.md` against the baseline; speed-ups must additionally leave
the replay of the baseline recording unchanged.

Priority: **P1** blocks mission completion, **P2** large effect on accuracy or time, **P3** measurement quality,
**P4** housekeeping.

## Exploration (why the mission never completes)
| ID | Pri | Problem and evidence | Planned change | Verify by |
|---|---|---|---|---|
| F1 | P1 | 48 frontiers remain at the end; 21 are outside the outer wall and 13 within 0.5 m of it, so they can never be cleared. Even the reference grid built with the true pose has 34 frontiers (9 outside), so the depth rays leak past walls independent of pose error | **Implemented 2026-11-02 (live check pending)**: a frontier is a candidate only if at least 3 of its unknown cells are visible from the cell the robot can stand on (ray-cast; walls block). This drops frontiers behind walls and inside objects without any arena prior. The map-side leak (rays passing walls) stays open under F30 | No reachable frontier behind a wall; mission reports completion |
| F2 | P1 | Unreachable frontiers are blacklisted, then the blacklist is cleared (when over 40 entries or no candidate is left), so the same targets return: single targets were planned 11-21 times, 1814 paths in 1800 s, 9 unreachable events, 13 stalled-target clears | **Implemented 2026-11-02 (live check pending)**. Root cause found: the blacklist stored the *navigation* cell but was checked against the *frontier* cell (up to 6 cells away), so a reached frontier was picked again at once. Baseline last 90 s: the spot (-0.60, -0.55), 0.17 m from the robot, was chosen 141 times; replaying the old code on the saved final map repeats it 298 of 300 times. Fix: retire the frontier cell and the navigation cell; retire on reaching the viewpoint, after 2 planner failures, or when the path is lost; never clear the list; a 9 s commitment timeout no longer retires a frontier. Same replay with the fix: MISSION_COMPLETE after 3 decisions (true-pose map: 21) | Repeated targets (same target planned more than 3 times) = 0 |
| F3 | P1 | 18 room-entered events for 8 rooms: the room labels change as the map grows, which restarts sweeps (79 sweep targets in one room) | **Part 1 implemented 2026-11-07 (live check pending)**. Live run `20261107_004151` (F1, F2 in place) livelocked: from 660 s the active room alternated 4 / 6 twenty times and one room-4 target was picked 19 times, because the active room switched whenever the label under the robot changed. Fix: the active room changes only after it is marked explored (no reachable frontier left under its ID); a vanished label closes on the next decision, so no deadlock. Test `test_unfinished_room_is_not_abandoned_on_label_change` fails on the old code and passes. Part 2 still open: labels drift because room cores are cut at a fraction of the current largest clearance and centroids are overwritten on each match | Room entries per room at most 1.25 |
| F4 | P1 | The baseline stopped at a time cap I set (1800 s) while still wandering | Default cap raised to 3600 s (done); a run should end only on completion. No-progress stop not added: the owner wants the robot to keep exploring, and F1/F2 remove the loop it would have caught | Run ends with end_reason = all reachable frontiers explored |
| F5 | P2 | Average speed 0.088 m/s; the calibrated gait tops out near 0.08 m/s; 42 % of the distance was walked after coverage had plateaued | After F1-F3, measure again; then raise the gait speed and re-calibrate `GAIT_SPEED_CURVE` | Average speed and time to completion |
| F6 | P2 | Doorway detections are unstable: 17 knowledge-graph entries against 10 in the final map | Track doorways by position, not list index | Doorway count stable across the run |

## Localization and mapping accuracy
| ID | Pri | Problem and evidence | Planned change | Verify by |
|---|---|---|---|---|
| F33 | P1 | Map noise seals doorways, so whole rooms become unreachable. Final baseline map: with the planner's 15 cm clearance the navigable floor splits into 46.6 m2 reachable plus two sealed rooms (2.7 and 1.9 m2); the true-pose map is one connected 68.6 m2 area. 244 of the 337 occupied blobs are 1 to 3 cells (355 cells): isolated false hits. Removing blobs under 4 cells reconnects one room; the other is closed by doubled walls from pose error | Require more evidence before a cell turns occupied in free space (fixes the specks; do not filter small blobs in the planner, real table legs are 1 to 2 cells); the doubled walls follow from F7 to F9 | Navigable floor is one connected area on the final map |
| F7 | P2 | Local error is high: RPE 7.4 % per metre, while odometry plus gyro alone has 1.8 % (but drifts to 44 deg heading). Scan matching adds local jitter | Find the cause (ICP translation noise vs acceptance gating); smooth or blend the translation | RPE below 3 % with ATE not worse, on replay of the baseline recording |
| F8 | P2 | Wall accuracy 12.9 cm and precision 67 % (5407 occupied cells against 3184 in the reference): walls are thick or doubled | Reduce wall smear: tighter occupancy update, ray rejection at grazing angles (same change as F1), pose-graph corrections applied to the map | Wall accuracy at most 10 cm, precision at least 80 % |
| F9 | P2 | Absolute ATE 31 cm exceeds the 15 cm map tolerance | Result of F7/F8; revisit loop-closure acceptance | ATE at most 15 cm |
| F10 | P3 | Degeneracy filter shows no benefit on this run (ATE +2 cm without it) | Test on a second recording with long corridors; remove it if still no benefit | Ablation on the new recording |
| F11 | P3 | Odometry-only replay (RPE 1.8 %) conflicts with the "odometry scale error -14.7 %" diagnostic | Define the scale metric against commanded distance, not ground-truth path length | Consistent metric |

## Speed and compute
| ID | Pri | Problem and evidence | Planned change | Verify by |
|---|---|---|---|---|
| F12 | P2 | Real-time factor 0.32; agents use only 46 % of wall time; the other 3074 s is unmeasured (simulator stepping, image transfer, HUD, `detect_frontiers()` called every step in `main_agentic.py`) | **Profiled 2026-11-07** (run `20261107_003107`, 120 s, cProfile). Loop wall time: occupancy ray-casting `grid.update_from_depth_points` about 40 % (Mapping agent 93 s, the evaluation-only reference grid built in the loop 47 s); simulator calls (ZMQ) about 30 %; SLAM 10 %; headless HUD about 9 % (`detect_frontiers()` every step, debug image, PNG write); frontier geodesic field 3 %. The 368 s matplotlib entry in the raw profile is a thread-wait artifact: the post-run reports took 10 s by file timestamps. Done: `detect_frontiers()` now runs only on HUD frames (every 5th step). Next: ray-casting (F14), then fewer simulator round trips | Real-time factor at least 1.0 |
| F13 | P2 | Stalls: control-loop step max 2.2 s (Perception max 1986 ms, Planning max 1061 ms, p95 128 ms); loop p95 380 ms against a 50 ms budget | Find the cause of the spikes (depth transfer, A* on a large known area); cap planner work per step | Loop p95 at most 100 ms, max at most 500 ms |
| F14 | P3 | Frontier agent p50 155 ms (geodesic search in Python) and Mapping 20 ms p50 (largest total) | Ray-casting is the largest item (F12 profile). **Done 2026-11-07**: Bresenham and the state test inlined, attributes bound to locals; 1.33x faster, map bit-identical to the previous code over 306 sequential updates (scratch check against git `HEAD`). Pure Python is near its limit here because rays in one scan depend on each other; a compiled kernel (numba, a new dependency, owner decision) is the next step. Frontier geodesic field still open | Latencies; replay unchanged |
| F15 | P3 | The in-memory recording buffer grows with run length (peak memory 3 GB over 30 min) | Stream the recording to disk | Peak memory |

## Measurement quality
| ID | Pri | Problem | Planned change |
|---|---|---|---|
| F16 | P3 | Scan-match acceptance is 27 % and the rejection reasons are not recorded | Record the reason per rejected update (few points, rmse gate, translation gate, rotation gate) |
| F17 | P3 | No collision or contact signal and no intervention channel | Wire CoppeliaSim contact detection; log operator stops |
| F18 | P3 | One run only | At least 3 runs with different gyro seeds; report spread |
| F19 | P3 | Map reference is not independent of the sensor | Build a reference occupancy grid from the scene geometry (known arena dimensions) |
| F20 | P3 | The baseline never finished, so time-to-complete of the current system is unknown | After F1-F4, run uncapped and record time-to-complete; keep a separate capped run of the old behaviour only if needed for comparison |
| F21 | P3 | Telemetry overhead is a calibrated estimate | An A/B run with telemetry disabled (needs a `--no-telemetry` switch) |
| F32 | P3 | PDF builder defect found on 2026-10-25: PyMuPDF's HTML renderer silently dropped content when a table was split across a page break (an earlier `RESULTS.pdf` was missing its comparison heading, sections 8 and 9 and a table) | **Fixed**: every block is laid out and drawn separately, tables split only between rows, and the builder raises if any heading or caption is missing from the finished PDF | Done |
| F31 | P3 | The automatic report at the end of a run failed: a local variable named `report` hid the report module | **Fixed** (variable renamed; verified by a live smoke run) | Done |
| F22 | P3 | `accuracy_results/` and `captured_matrices/` are still written by `main_agentic.py`, duplicating the run directory | Write everything into `runs/<id>/` directly; needs a live smoke run to verify |

## Perception and knowledge graph (found by scoring against the real scene)
| ID | Pri | Problem and evidence | Planned change | Verify by |
|---|---|---|---|---|
| F28 | P2 | There is no object recognition: any occupied blob 0.18-1.05 m across is labelled "chair" by size (`mapping_agent.extract_semantic_entities`); the RGB image is not used. Baseline: 113 "chair" detections for 1 real chair; 21 % of detections lie on any real object | Detect objects from the RGB-D image (or at least classify blobs that are not part of a wall); drop detections that are no longer supported by the map | Detection precision at least 80 %, chair labels on real chairs |
| F29 | P3 | Knowledge-graph placeholders: every doorway is recorded as connecting rooms 1 and 2 (`supervisor_agent.py` hard-codes it) and every room centroid stays (0, 0) | Take room connectivity and centroids from the mapping agent's segmentation | Graph matches the arena layout |
| F30 | P3 | 0.59 m2 of free space is mapped outside the outer wall (same cause as F1) | Fixed with F1 | Free space outside the arena about 0 |

## Realism
| ID | Pri | Problem | Planned change |
|---|---|---|---|
| F23 | P4 | `LizardRobot.steer_heading` turns the body by setting its orientation directly (simulation shortcut) | Replace with gait-driven turning, then re-measure |
| F24 | P4 | Simulation only | Hardware validation (outside this repo) |

## External comparison (Phase B follow-ups)
| ID | Pri | Experiment |
|---|---|---|
| F25 | P2 | Run Cartographer (or another open 2D SLAM) on this run's recorded scans and odometry: identical data and ground truth, the only fair head-to-head available |
| F26 | P3 | Run a standard nearest-frontier explorer in the same arena with the same robot for a time-to-complete comparison |
| F27 | P3 | Evaluate on a public dataset with our metrics if a suitable one (indoor, 2D scan or RGB-D, ground truth) is available |
