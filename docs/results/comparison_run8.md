| Metric | Baseline | New (1 run) | Change | Verdict |
|---|---|---|---|---|
| **Exploration** | | | | |
| Task completed (1 = yes) | 0 | 1 | +1 | Better |
| Rooms completed | 6 | 12 | +6 (+100%) | Better |
| Rooms found | 8 | 13 | +5 (+62%) | - |
| Simulated time | 1800 s | 3000 s | +1200 (+67%) | - |
| Distance travelled | 157.9 m | 272.3 m | +114.3 (+72%) | - |
| Average speed | 0.088 m/s | 0.091 m/s | +0.003 (+3%) | Better |
| Real floor mapped | 88.2 % | 92.6 % | +4.3 (+5%) | Better |
| Free space mapped outside the arena | 0.59 m2 | 0.24 m2 | -0.35 (-59%) | Better |
| Frontiers remaining | 48 | 43 | -5 (-10%) | Better |
| Stuck events | 4 | 2 | -2 (-50%) | Better |
| Unreachable targets | 9 | 3 | -6 (-67%) | Better |
| Room entries per room | 2.25 | 0.77 | -1.48 (-66%) | Better |
| Path walked after coverage plateau | 46 % | 61 % | +15 (+32%) | Worse |
| **Localization** | | | | |
| ATE RMSE | 31.0 cm | 14.7 cm | -16.2 (-52%) | Better |
| ATE as share of path | 0.20 % | 0.05 % | -0.14 (-72%) | Better |
| Final drift | 0.24 % of path | 0.13 % of path | -0.12 (-47%) | Better |
| Largest position error | 63.1 cm | 69.4 cm | +6.3 (+10%) | Worse |
| RPE translation | 7.4 % | 6.5 % | -0.9 (-12%) | Better |
| RPE rotation | 1.30 deg/m | 1.31 deg/m | +0.01 (+1%) | Same |
| Heading error (mean) | 0.74 deg | 0.80 deg | +0.06 (+8%) | Worse |
| Loop closures | 68 | 202 | +134 (+197%) | - |
| Scan-match acceptance | 26.9 % | 26.7 % | -0.2 (-1%) | Same |
| **Mapping vs real scene** | | | | |
| Map accuracy | 10.6 cm | 4.4 cm | -6.2 (-59%) | Better |
| Map completion | 9.2 cm | 6.1 cm | -3.0 (-33%) | Better |
| Precision | 73.9 % | 95.4 % | +21.4 (+29%) | Better |
| Recall | 81.8 % | 91.6 % | +9.7 (+12%) | Better |
| F1 | 77.7 % | 93.4 % | +15.8 (+20%) | Better |
| **Detection** | | | | |
| Detections on a real object | 21.2 % | 21.8 % | +0.5 (+3%) | Same |
| **Processing time** | | | | |
| SLAM update (median) | 10.6 ms | 9.8 ms | -0.8 (-8%) | Better |
| Map update (median) | 20.3 ms | 2.9 ms | -17.4 (-86%) | Better |
| Frontier selection (median) | 155 ms | 22 ms | -133 (-86%) | Better |
| Planning (p95) | 128 ms | 16 ms | -112 (-87%) | Better |
| Slowest single agent call | 1986 ms | 1152 ms | -834 (-42%) | Better |
| Control step (p95) | 380 ms | 248 ms | -132 (-35%) | Better |
| Real-time factor | 0.32 | 0.55 | +0.23 (+73%) | Better |
| **Resources** | | | | |
| Process CPU (mean) | 86 % of a core | 69 % of a core | -16 (-19%) | - |
| Memory (peak) | 2973 MB | 4377 MB | +1405 (+47%) | Worse |

**Summary:** 25 better, 4 worse, 3 within 3% (of 32 metrics with a direction). Baseline: runs/20261024_210638; new: runs/20261107_180907. Single new run: no variability can be stated.
