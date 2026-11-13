| Metric | Baseline | New (1 run) | Change | Verdict |
|---|---|---|---|---|
| **Exploration** | | | | |
| Task completed (1 = yes) | 0 | 1 | +1 | Better |
| Rooms completed | 6 | 12 | +6 (+100%) | Better |
| Rooms found | 8 | 13 | +5 (+62%) | - |
| Simulated time | 1800 s | 3000 s | +1200 (+67%) | - |
| Distance travelled | 157.9 m | 260.5 m | +102.5 (+65%) | - |
| Average speed | 0.088 m/s | 0.087 m/s | -0.001 (-1%) | Same |
| Real floor mapped | 88.2 % | 91.7 % | +3.4 (+4%) | Better |
| Free space mapped outside the arena | 0.59 m2 | 0.57 m2 | -0.02 (-3%) | Same |
| Frontiers remaining | 48 | 58 | +10 (+21%) | Worse |
| Stuck events | 4 | 2 | -2 (-50%) | Better |
| Unreachable targets | 9 | 32 | +23 (+256%) | Worse |
| Room entries per room | 2.25 | 0.85 | -1.40 (-62%) | Better |
| Path walked after coverage plateau (while exploring) | 46 % | 21 % | -25 (-54%) | Better |
| Time until everything reachable was explored | n/a s | 1257 s | - | - |
| Patrol walk after exploration | 0.0 m | 152.1 m | +152.1 | - |
| **Localization** | | | | |
| ATE RMSE | 31.0 cm | 17.9 cm | -13.0 (-42%) | Better |
| ATE as share of path | 0.20 % | 0.07 % | -0.13 (-65%) | Better |
| Final drift | 0.24 % of path | 0.06 % of path | -0.18 (-76%) | Better |
| Largest position error | 63.1 cm | 58.5 cm | -4.6 (-7%) | Better |
| RPE translation | 7.4 % | 6.0 % | -1.3 (-18%) | Better |
| RPE rotation | 1.30 deg/m | 1.32 deg/m | +0.02 (+2%) | Same |
| Heading error (mean) | 0.74 deg | 0.75 deg | +0.01 (+1%) | Same |
| Loop closures | 68 | 242 | +174 (+256%) | - |
| Scan-match acceptance | 26.9 % | 22.6 % | -4.3 (-16%) | Worse |
| **Mapping vs real scene** | | | | |
| Map accuracy | 10.6 cm | 6.0 cm | -4.6 (-44%) | Better |
| Map completion | 9.1 cm | 6.3 cm | -2.7 (-30%) | Better |
| Precision | 73.9 % | 90.2 % | +16.2 (+22%) | Better |
| Recall | 82.0 % | 91.3 % | +9.4 (+11%) | Better |
| F1 | 77.7 % | 90.7 % | +13.0 (+17%) | Better |
| **Detection** | | | | |
| Detections on a real object | 21.2 % | 24.5 % | +3.3 (+15%) | Better |
| **Processing time** | | | | |
| SLAM update (median) | 10.6 ms | 12.8 ms | +2.2 (+21%) | Worse |
| Map update (median) | 20.3 ms | 0.6 ms | -19.7 (-97%) | Better |
| Frontier selection (median) | 155 ms | 20 ms | -135 (-87%) | Better |
| Planning (p95) | 128 ms | 12 ms | -116 (-91%) | Better |
| Slowest single agent call | 1986 ms | 827 ms | -1160 (-58%) | Better |
| Control step (p95) | 380 ms | 182 ms | -198 (-52%) | Better |
| Real-time factor | 0.32 | 0.58 | +0.26 (+83%) | Better |
| **Resources** | | | | |
| Process CPU (mean) | 86 % of a core | 63 % of a core | -23 (-26%) | - |
| Memory (peak) | 2973 MB | 1354 MB | -1619 (-54%) | Better |

**Summary:** 24 better, 4 worse, 4 within 3% (of 32 metrics with a direction). Baseline: runs/20261024_210638; new: runs/20261113_120000. Single new run: no variability can be stated.
