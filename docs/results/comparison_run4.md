| Metric | Baseline | New (1 run) | Change | Verdict |
|---|---|---|---|---|
| **Exploration** | | | | |
| Task completed (1 = yes) | 0 | 1 | +1 | Better |
| Rooms completed | 6 | 8 | +2 (+33%) | Better |
| Rooms found | 8 | 11 | +3 (+38%) | - |
| Simulated time | 1800 s | 1872 s | +72 (+4%) | - |
| Distance travelled | 157.9 m | 168.7 m | +10.8 (+7%) | - |
| Average speed | 0.088 m/s | 0.090 m/s | +0.002 (+3%) | Same |
| Real floor mapped | 88.2 % | 92.2 % | +3.9 (+4%) | Better |
| Free space mapped outside the arena | 0.59 m2 | 1.41 m2 | +0.82 (+140%) | Worse |
| Frontiers remaining | 48 | 53 | +5 (+10%) | Worse |
| Stuck events | 4 | 17 | +13 (+325%) | Worse |
| Unreachable targets | 9 | 1 | -8 (-89%) | Better |
| Room entries per room | 2.25 | 0.73 | -1.52 (-68%) | Better |
| Path walked after coverage plateau | 46 % | 35 % | -12 (-25%) | Better |
| **Localization** | | | | |
| ATE RMSE | 31.0 cm | 26.9 cm | -4.0 (-13%) | Better |
| ATE as share of path | 0.20 % | 0.16 % | -0.04 (-19%) | Better |
| Final drift | 0.24 % of path | 0.09 % of path | -0.16 (-65%) | Better |
| Largest position error | 63.1 cm | 52.7 cm | -10.5 (-17%) | Better |
| RPE translation | 7.4 % | 4.6 % | -2.8 (-37%) | Better |
| RPE rotation | 1.30 deg/m | 1.33 deg/m | +0.03 (+3%) | Same |
| Heading error (mean) | 0.74 deg | 0.79 deg | +0.05 (+6%) | Worse |
| Loop closures | 68 | 77 | +9 (+13%) | - |
| Scan-match acceptance | 26.9 % | 25.1 % | -1.8 (-7%) | Worse |
| **Mapping vs real scene** | | | | |
| Map accuracy | 10.6 cm | 7.4 cm | -3.2 (-30%) | Better |
| Map completion | 9.2 cm | 8.3 cm | -0.8 (-9%) | Better |
| Precision | 73.9 % | 82.5 % | +8.6 (+12%) | Better |
| Recall | 81.8 % | 83.1 % | +1.3 (+2%) | Same |
| F1 | 77.7 % | 82.8 % | +5.1 (+7%) | Better |
| **Detection** | | | | |
| Detections on a real object | 21.2 % | 22.7 % | +1.4 (+7%) | Better |
| **Processing time** | | | | |
| SLAM update (median) | 10.6 ms | 12.7 ms | +2.1 (+20%) | Worse |
| Map update (median) | 20.3 ms | 1.8 ms | -18.5 (-91%) | Better |
| Frontier selection (median) | 155 ms | 215 ms | +60 (+39%) | Worse |
| Planning (p95) | 128 ms | 33 ms | -94 (-74%) | Better |
| Slowest single agent call | 1986 ms | 1504 ms | -483 (-24%) | Better |
| Control step (p95) | 380 ms | 373 ms | -7 (-2%) | Same |
| Real-time factor | 0.32 | 0.47 | +0.15 (+48%) | Better |
| **Resources** | | | | |
| Process CPU (mean) | 86 % of a core | 74 % of a core | -12 (-14%) | - |
| Memory (peak) | 2973 MB | 3319 MB | +346 (+12%) | Worse |

**Summary:** 20 better, 8 worse, 4 within 3% (of 32 metrics with a direction). Baseline: runs/20261024_210638; new: runs/20261107_125826. Single new run: no variability can be stated.
