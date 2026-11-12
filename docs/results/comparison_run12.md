| Metric | Baseline | New (1 run) | Change | Verdict |
|---|---|---|---|---|
| **Exploration** | | | | |
| Task completed (1 = yes) | 0 | 1 | +1 | Better |
| Rooms completed | 6 | 9 | +3 (+50%) | Better |
| Rooms found | 8 | 11 | +3 (+38%) | - |
| Simulated time | 1800 s | 2107 s | +307 (+17%) | - |
| Distance travelled | 157.9 m | 179.1 m | +21.2 (+13%) | - |
| Average speed | 0.088 m/s | 0.085 m/s | -0.003 (-3%) | Worse |
| Real floor mapped | 88.2 % | 91.4 % | +3.1 (+4%) | Better |
| Free space mapped outside the arena | 0.59 m2 | 0.26 m2 | -0.32 (-55%) | Better |
| Frontiers remaining | 48 | 66 | +18 (+38%) | Worse |
| Stuck events | 4 | 3 | -1 (-25%) | Better |
| Unreachable targets | 9 | 8 | -1 (-11%) | Better |
| Room entries per room | 2.25 | 0.82 | -1.43 (-64%) | Better |
| Path walked after coverage plateau (while exploring) | 46 % | 22 % | -24 (-52%) | Better |
| Time until everything reachable was explored | n/a s | 1072 s | - | - |
| Patrol walk after exploration | 0.0 m | 89.0 m | +89.0 | - |
| **Localization** | | | | |
| ATE RMSE | 31.0 cm | 34.6 cm | +3.7 (+12%) | Worse |
| ATE as share of path | 0.20 % | 0.19 % | -0.00 (-1%) | Same |
| Final drift | 0.24 % of path | 0.26 % of path | +0.02 (+7%) | Worse |
| Largest position error | 63.1 cm | 116.3 cm | +53.2 (+84%) | Worse |
| RPE translation | 7.4 % | 7.2 % | -0.2 (-3%) | Same |
| RPE rotation | 1.30 deg/m | 1.33 deg/m | +0.03 (+2%) | Same |
| Heading error (mean) | 0.74 deg | 0.71 deg | -0.03 (-4%) | Better |
| Loop closures | 68 | 91 | +23 (+34%) | - |
| Scan-match acceptance | 26.9 % | 23.8 % | -3.0 (-11%) | Worse |
| **Mapping vs real scene** | | | | |
| Map accuracy | 10.6 cm | 7.0 cm | -3.6 (-34%) | Better |
| Map completion | 9.1 cm | 7.7 cm | -1.4 (-15%) | Better |
| Precision | 73.9 % | 87.3 % | +13.4 (+18%) | Better |
| Recall | 82.0 % | 87.3 % | +5.3 (+7%) | Better |
| F1 | 77.7 % | 87.3 % | +9.6 (+12%) | Better |
| **Detection** | | | | |
| Detections on a real object | 21.2 % | 23.6 % | +2.3 (+11%) | Better |
| **Processing time** | | | | |
| SLAM update (median) | 10.6 ms | 10.7 ms | +0.1 (+0%) | Same |
| Map update (median) | 20.3 ms | 0.6 ms | -19.7 (-97%) | Better |
| Frontier selection (median) | 155 ms | 18 ms | -137 (-88%) | Better |
| Planning (p95) | 128 ms | 5 ms | -123 (-96%) | Better |
| Slowest single agent call | 1986 ms | 649 ms | -1337 (-67%) | Better |
| Control step (p95) | 380 ms | 144 ms | -236 (-62%) | Better |
| Real-time factor | 0.32 | 0.69 | +0.37 (+117%) | Better |
| **Resources** | | | | |
| Process CPU (mean) | 86 % of a core | 62 % of a core | -24 (-28%) | - |
| Memory (peak) | 2973 MB | 606 MB | -2366 (-80%) | Better |

**Summary:** 22 better, 6 worse, 4 within 3% (of 32 metrics with a direction). Baseline: runs/20261024_210638; new: runs/20261112_111339. Single new run: no variability can be stated.
