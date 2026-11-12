| Metric | Baseline | New (1 run) | Change | Verdict |
|---|---|---|---|---|
| **Exploration** | | | | |
| Task completed (1 = yes) | 0 | 1 | +1 | Better |
| Rooms completed | 6 | 8 | +2 (+33%) | Better |
| Rooms found | 8 | 9 | +1 (+12%) | - |
| Simulated time | 1800 s | 3000 s | +1200 (+67%) | - |
| Distance travelled | 157.9 m | 260.4 m | +102.4 (+65%) | - |
| Average speed | 0.088 m/s | 0.087 m/s | -0.001 (-1%) | Same |
| Real floor mapped | 88.2 % | 92.8 % | +4.5 (+5%) | Better |
| Free space mapped outside the arena | 0.59 m2 | 0.10 m2 | -0.49 (-83%) | Better |
| Frontiers remaining | 48 | 36 | -12 (-25%) | Better |
| Stuck events | 4 | 4 | +0 (+0%) | Same |
| Unreachable targets | 9 | 7 | -2 (-22%) | Better |
| Room entries per room | 2.25 | 0.67 | -1.58 (-70%) | Better |
| Path walked after coverage plateau (while exploring) | 46 % | 32 % | -14 (-31%) | Better |
| Time until everything reachable was explored | n/a s | 1371 s | - | - |
| Patrol walk after exploration | 0.0 m | 141.4 m | +141.4 | - |
| **Localization** | | | | |
| ATE RMSE | 31.0 cm | 16.5 cm | -14.5 (-47%) | Better |
| ATE as share of path | 0.20 % | 0.06 % | -0.13 (-68%) | Better |
| Final drift | 0.24 % of path | 0.07 % of path | -0.17 (-70%) | Better |
| Largest position error | 63.1 cm | 62.2 cm | -0.9 (-1%) | Same |
| RPE translation | 7.4 % | 6.6 % | -0.8 (-11%) | Better |
| RPE rotation | 1.30 deg/m | 1.28 deg/m | -0.02 (-2%) | Same |
| Heading error (mean) | 0.74 deg | 0.73 deg | -0.01 (-1%) | Same |
| Loop closures | 68 | 252 | +184 (+271%) | - |
| Scan-match acceptance | 26.9 % | 24.4 % | -2.5 (-9%) | Worse |
| **Mapping vs real scene** | | | | |
| Map accuracy | 10.6 cm | 4.7 cm | -5.9 (-56%) | Better |
| Map completion | 9.1 cm | 6.4 cm | -2.7 (-30%) | Better |
| Precision | 73.9 % | 93.6 % | +19.6 (+27%) | Better |
| Recall | 82.0 % | 90.6 % | +8.6 (+10%) | Better |
| F1 | 77.7 % | 92.0 % | +14.3 (+18%) | Better |
| **Detection** | | | | |
| Detections on a real object | 21.2 % | 22.4 % | +1.2 (+6%) | Better |
| **Processing time** | | | | |
| SLAM update (median) | 10.6 ms | 7.9 ms | -2.7 (-25%) | Better |
| Map update (median) | 20.3 ms | 0.4 ms | -19.9 (-98%) | Better |
| Frontier selection (median) | 155 ms | 11 ms | -144 (-93%) | Better |
| Planning (p95) | 128 ms | 6 ms | -122 (-95%) | Better |
| Slowest single agent call | 1986 ms | 1719 ms | -268 (-13%) | Better |
| Control step (p95) | 380 ms | 121 ms | -258 (-68%) | Better |
| Real-time factor | 0.32 | 0.69 | +0.37 (+117%) | Better |
| **Resources** | | | | |
| Process CPU (mean) | 86 % of a core | 55 % of a core | -31 (-36%) | - |
| Memory (peak) | 2973 MB | 1339 MB | -1633 (-55%) | Better |

**Summary:** 26 better, 1 worse, 5 within 3% (of 32 metrics with a direction). Baseline: runs/20261024_210638; new: runs/20261112_232106. Single new run: no variability can be stated.
