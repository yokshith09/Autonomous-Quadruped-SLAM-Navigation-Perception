# Plan from the frozen results to the benchmarks (set 2026-11-14)

One system, several environments: every result is reported per scene and as the average across scenes.
Rules: save every run (commit run folder, copy best runs to E:\project_phase1\best_results_backup), log every decision in
PROGRESS.md, never overwrite a best result (tags), short runs first.

## Phase 0 — Done: frozen results (tag `final-multi-env-v1`)
| | Scene 1 (run 34) | Scene 2 rescue (run 19) | Average |
|---|---|---|---|
| ATE | 11.7 cm | 12.3 cm | 12.0 cm |
| Precision / F1 | 99.7 / 95.9 % | 95.8 / 55.2 % | 97.8 / 75.6 % |
| Floor covered | 97.7 % | 59.0 % (12 min) | 78.4 % |
| Stuck / collisions | 0 / not measured | 1 recovered / 0 | |

## Phase 1 — Scene 1 verification run (~45 min wall time)
Why: scene 1 has not run since F70-F81; it also has no measured collision count.
- Load scene 1 in CoppeliaSim; `python -u main_agentic.py --headless --max-time 1800`.
- Pass: ATE <= 13 cm (11.7 + run-to-run spread), F1 >= 95 %, 0 collisions, 0 stuck.
- If it fails: replay the run 34 recording on current code (`evaluation/slam_replay.py`) to tell a SLAM change from a route change; compare against tag `best-sim-v3`; fix or restore.
- Output: verified scene 1 numbers on current code (or run 34 stays the reference with a stated reason).

## Phase 2 — Scene 2 full run on the requested route (~1.5 h)
Spec: docs/project/TODO_RESCUE_ROUTE.md and the drawing docs/results/rescue_requested_route_2026-11-14.png.
1. Trace the drawn route into `route` waypoints in config/scenes/searchrescuescene.json; snap to safe floor; plot for review.
2. 12-min check run: route followed in order, no stuck, no collisions. Fix anything that breaks, rerun.
3. Full run (25-30 min).
- Targets: building gap passed, >= 90 % of reachable floor, 0 collisions, 0 stuck, ATE < 11 cm, F1 > 95 %.
- Output: new scene 2 result; new averages; tag `final-multi-env-v2` (v1 stays).

## Phase 3 — Benchmarking (all on recordings unless stated; both scenes, plus average)
| # | Benchmark | What it shows | Needs | Time |
|---|---|---|---|---|
| 3a | SLAM baselines on our recordings: odometry only, KISS-ICP, BreezySLAM vs ours | same input, same scoring | pip `kiss-icp`, BreezySLAM (GitHub) | 0.5 day |
| 3b | Ablation from replays: no loop closure, no gyro calibration, submap 8 vs 16, no route, no narrow-gap mode | what each part adds | nothing | 2 h |
| 3c | Robustness (T2): seeded depth noise, dropouts, gyro drift, leg slip | sim-to-real sensitivity | nothing | 0.5 day |
| 3d | Runtime: per-agent time, CPU %, RAM; Raspberry Pi estimate | real-time claim | nothing | 1 h |
| 3e | Exploration baseline (T5): nearest-frontier (Yamauchi 1997) switch, one run per scene | value of our exploration | 2 live runs | 1 day |
| 3f | Cartographer (T3) on our recordings (and TUM laser) | standard 2D SLAM, same data | Docker ROS image ~2 GB | 1 day |
| 3g | TUM Pioneer (T4): 4 bags with wheel odometry; vs RTABMap 28.7, OpenVSLAM 39.2, ORB-SLAM3 172 cm | external public dataset | download ~4.6 GB, pip `rosbags` | 0.5-1 day |

Order: 3a -> 3b -> 3c -> 3d (no downloads, ~1.5 days), then 3e, then 3f and 3g.

## Phase 4 — Write-up (T6, ~0.5 day)
docs/results/benchmarks.md: per-scene + average tables, coverage curves, ablation and noise tables,
same-data comparisons (3a, 3f, 3g), published-context table (docs/results/benchmarks_published.md), and the limits.

## Later
- A third scene (generalisation to any indoor scene).
- Real robot. Team's Video-Mamba model (set aside).

## Approvals needed
- Phase 1/2: user loads the scene in CoppeliaSim.
- 3a: install kiss-icp, BreezySLAM. 3f: Docker image pull. 3g: 4.6 GB download + rosbags.
- Merging to main.
