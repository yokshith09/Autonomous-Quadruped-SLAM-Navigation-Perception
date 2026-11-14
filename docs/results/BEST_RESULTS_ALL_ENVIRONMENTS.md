# Best results across environments (one system, same code)

The system is evaluated as one robot across several environments. The final result is the performance over
all environments, not one table per scene. Only scene settings (floor calibration, map window, gyro
calibration time, route) differ, and they live in `config/scenes/<scene>.json`.

## Preserved best results (do not overwrite; restore with the git tag)

| Environment | Run (folder) | Code tag | ATE (cm) | Map F1 | Floor seen | Stuck | Collisions |
|---|---|---|---|---|---|---|---|
| Scene 1 (10 x 10 m rooms) | run 34 (20261113_*, see BEST_RESULT.md) | `best-sim-v3` | **11.7** | **95.9 %** | 97.7 % | 0 | not measured |
| Rescue (16 x 16 m open floor) | run 16 (20261114_115832), coverage best | `rescue-best-v1` | 27.4 | 64.0 % | 96.6 % of reachable | 2 | 0 |
| Rescue | run 18 (20261114_142725), 12 min, 0 stuck | `rescue-best-v1` | 22.2 (replay with 16-keyframe submap: 14.4) | P 80.1 % | 37.4 % | 0 | 0 |
| Rescue | run 14 (20261114_045740), crossed the building gap | `e3cc965`-era code | 19.0 | 67.4 % | 67.6 % | 0 | not measured |
| Rescue | run 12 (20261114_041347), lowest ATE | – | 9.9 | 74.5 % | 55.3 % | 1 | not measured |

Rescue replay mean (3 recordings x 3 seeds, latest code): **13.9 cm**.

## Combined (current best per environment)

| | ATE mean (cm) | F1 mean | Stuck | Collisions |
|---|---|---|---|---|
| Scene 1 run 34 + rescue run 16 | (11.7 + 27.4) / 2 = 19.6 | (95.9 + 64.0) / 2 = 80.0 % | 2 | 0 |

## Decisions log

- 2026-11-14: results are reported as one system across environments (user decision).
- 2026-11-14: rescue changes are scene-profile settings only (submap 16, route, room_sweep off); scene 1 code path unchanged.
- 2026-11-14: rescue SLAM changes judged on the 9-replay mean, since single-run ATE varies 13–19 cm from tiny perturbations.
- Run folders are committed (recordings excluded, see .gitignore) so results survive a lost working tree.
