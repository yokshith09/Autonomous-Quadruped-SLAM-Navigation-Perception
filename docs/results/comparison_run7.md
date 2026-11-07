| Metric | Baseline | New (1 run) | Change | Verdict |
|---|---|---|---|---|
| **Exploration** | | | | |
| Task completed (1 = yes) | 0 | 1 | +1 | Better |
| Rooms completed | 6 | 12 | +6 (+100%) | Better |
| Rooms found | 8 | 13 | +5 (+62%) | - |
| Simulated time | 1800 s | 2105 s | +305 (+17%) | - |
| Distance travelled | 157.9 m | 158.6 m | +0.7 (+0%) | - |
| Average speed | 0.088 m/s | 0.075 m/s | -0.012 (-14%) | Worse |
| Real floor mapped | 88.2 % | 94.1 % | +5.8 (+7%) | Better |
| Free space mapped outside the arena | 0.59 m2 | 0.17 m2 | -0.42 (-72%) | Better |
| Frontiers remaining | 48 | 41 | -7 (-15%) | Better |
| Stuck events | 4 | 2 | -2 (-50%) | Better |
| Unreachable targets | 9 | 2 | -7 (-78%) | Better |
| Room entries per room | 2.25 | 0.77 | -1.48 (-66%) | Better |
| Path walked after coverage plateau | 46 % | 33 % | -13 (-28%) | Better |
| **Localization** | | | | |
| ATE RMSE | 31.0 cm | 14.6 cm | -16.3 (-53%) | Better |
| ATE as share of path | 0.20 % | 0.09 % | -0.10 (-53%) | Better |
| Final drift | 0.24 % of path | 0.05 % of path | -0.19 (-79%) | Better |
| Largest position error | 63.1 cm | 69.4 cm | +6.3 (+10%) | Worse |
| RPE translation | 7.4 % | 5.9 % | -1.5 (-20%) | Better |
| RPE rotation | 1.30 deg/m | 1.30 deg/m | +0.00 (+0%) | Same |
| Heading error (mean) | 0.74 deg | 0.68 deg | -0.06 (-8%) | Better |
| Loop closures | 68 | 80 | +12 (+18%) | - |
| Scan-match acceptance | 26.9 % | 37.8 % | +11.0 (+41%) | Better |
| **Mapping vs real scene** | | | | |
| Map accuracy | 10.6 cm | 4.1 cm | -6.6 (-62%) | Better |
| Map completion | 9.2 cm | 7.3 cm | -1.9 (-20%) | Better |
| Precision | 73.9 % | 94.8 % | +20.9 (+28%) | Better |
| Recall | 81.8 % | 88.7 % | +6.9 (+8%) | Better |
| F1 | 77.7 % | 91.7 % | +14.0 (+18%) | Better |
| **Detection** | | | | |
| Detections on a real object | 21.2 % | 22.1 % | +0.9 (+4%) | Better |
| **Processing time** | | | | |
| SLAM update (median) | 10.6 ms | 10.1 ms | -0.6 (-5%) | Better |
| Map update (median) | 20.3 ms | 2.7 ms | -17.6 (-87%) | Better |
| Frontier selection (median) | 155 ms | 23 ms | -132 (-85%) | Better |
| Planning (p95) | 128 ms | 6 ms | -121 (-95%) | Better |
| Slowest single agent call | 1986 ms | 1359 ms | -627 (-32%) | Better |
| Control step (p95) | 380 ms | 196 ms | -184 (-48%) | Better |
| Real-time factor | 0.32 | 0.59 | +0.27 (+85%) | Better |
| **Resources** | | | | |
| Process CPU (mean) | 86 % of a core | 65 % of a core | -21 (-24%) | - |
| Memory (peak) | 2973 MB | 3103 MB | +131 (+4%) | Worse |

**Summary:** 28 better, 3 worse, 1 within 3% (of 32 metrics with a direction). Baseline: runs/20261024_210638; new: runs/20261107_170841. Single new run: no variability can be stated.
