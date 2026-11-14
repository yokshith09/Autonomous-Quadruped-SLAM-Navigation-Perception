# Best results across environments (one system, same code)

The system is evaluated as one robot across several environments. The final result is the performance over
all environments, not one table per scene. Only scene settings (floor calibration, map window, gyro
calibration time, route) differ, and they live in `config/scenes/<scene>.json`.

## Preserved best results (do not overwrite; restore with the git tag)

| Environment | Run (folder) | Code tag | ATE (cm) | Map F1 | Floor seen | Stuck | Collisions |
|---|---|---|---|---|---|---|---|
| Scene 1 (10 x 10 m rooms) | run 34 (20261113_232046, see BEST_RESULT.md) | `best-sim-v3` | **11.7** | **95.9 %** | 97.7 % | 0 | not measured |
| Rescue (16 x 16 m open floor) | run 16 (20261114_115832), coverage best | `rescue-best-v1` | 27.4 | 64.0 % | 96.6 % of reachable | 2 | 0 |
| Rescue | run 18 (20261114_142725), 12 min, 0 stuck | `rescue-best-v1` | 22.2 (replay with 16-keyframe submap: 14.4) | P 80.1 % | 37.4 % | 0 | 0 |
| Rescue | run 14 (20261114_045740), crossed the building gap | `e3cc965`-era code | 19.0 | 67.4 % | 67.6 % | 0 | not measured |
| Rescue | run 12 (20261114_041347), lowest ATE | – | 9.9 | 74.5 % | 55.3 % | 1 | not measured |

Rescue replay mean (3 recordings x 3 seeds, latest code): **13.9 cm**.

## Combined (current best per environment)

| | ATE mean (cm) | F1 mean | Stuck | Collisions |
|---|---|---|---|---|
| Scene 1 run 34 + rescue run 19 (accuracy best) | (11.7 + 12.3) / 2 = **12.0** | (95.9 + 55.2) / 2 = 75.6 %; precision (99.7 + 95.8) / 2 = **97.8 %** | 1 (recovered) | 0 |
| Scene 1 run 34 + rescue run 16 (coverage best) | (11.7 + 27.4) / 2 = 19.6 | (95.9 + 64.0) / 2 = 80.0 % | 2 | 0 |

Rescue F1 is limited by recall: a 12-minute run covers 59 % of the 234 m² floor. What the robot maps is 95.8 % correct.

## Final average results (frozen 2026-11-14, tag final-multi-env-v1)

| Metric | Scene 1 (run 34) | Rescue (run 19) | Average |
|---|---|---|---|
| ATE RMSE | 11.7 cm | 12.3 cm | **12.0 cm** |
| Map precision vs scene model | 99.7 % | 95.8 % | **97.8 %** |
| Map recall vs scene model | 92.4 % | 38.8 % | 65.6 % |
| Map F1 vs scene model | 95.9 % | 55.2 % | 75.6 % |
| Obstacle position accuracy (mapped to true) | 2.8 cm | 5.1 cm | 4.0 cm |
| Floor covered | 97.7 % | 59.0 % | 78.4 % |
| Run length / distance | 1800 s / 156.9 m | 720 s / 38.5 m | – |
| Stuck events | 0 | 1 (recovered) | 0.5 per run |
| Collisions | not measured | 0 | 0 measured |
| Control step p50 / p95 | 27 / 138 ms | 22 / 118 ms | 25 / 128 ms |

Pending: full-length rescue run (coverage, recall) and a scene 1 verification run on current code.

## Decisions log

- 2026-11-14: results are reported as one system across environments (user decision).
- 2026-11-14: rescue changes are scene-profile settings only (submap 16, route, room_sweep off); scene 1 code path unchanged.
- 2026-11-14: rescue SLAM changes judged on the 9-replay mean, since single-run ATE varies 13–19 cm from tiny perturbations.
- Full copies of the best runs, recordings included, are at `E:\project_phase1\best_results_backup` (outside git).
- Run folders are committed (recordings excluded, see .gitignore) so results survive a lost working tree.
- 2026-11-14: run 19 confirms the 16-keyframe submap live: rescue ATE 22.2 cm (run 18) -> 12.3 cm, same 12 min, same route. Kept.
