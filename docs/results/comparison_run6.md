| Metric | Baseline | New (1 run) | Change | Verdict |
|---|---|---|---|---|
| **Exploration** | | | | |
| Task completed (1 = yes) | 0 | 1 | +1 | Better |
| Rooms completed | 6 | 7 | +1 (+17%) | Better |
| Rooms found | 8 | 8 | +0 (+0%) | - |
| Simulated time | 1800 s | 1604 s | -195 (-11%) | - |
| Distance travelled | 157.9 m | 145.5 m | -12.4 (-8%) | - |
| Average speed | 0.088 m/s | 0.091 m/s | +0.003 (+3%) | Better |
| Real floor mapped | 88.2 % | 92.9 % | +4.6 (+5%) | Better |
| Free space mapped outside the arena | 0.59 m2 | 0.40 m2 | -0.19 (-32%) | Better |
| Frontiers remaining | 48 | 46 | -2 (-4%) | Better |
| Stuck events | 4 | 3 | -1 (-25%) | Better |
| Unreachable targets | 9 | 5 | -4 (-44%) | Better |
| Room entries per room | 2.25 | 0.88 | -1.38 (-61%) | Better |
| Path walked after coverage plateau | 46 % | 29 % | -17 (-38%) | Better |
| **Localization** | | | | |
| ATE RMSE | 31.0 cm | 16.1 cm | -14.9 (-48%) | Better |
| ATE as share of path | 0.20 % | 0.11 % | -0.09 (-44%) | Better |
| Final drift | 0.24 % of path | 0.09 % of path | -0.16 (-64%) | Better |
| Largest position error | 63.1 cm | 33.2 cm | -29.9 (-47%) | Better |
| RPE translation | 7.4 % | 4.9 % | -2.4 (-33%) | Better |
| RPE rotation | 1.30 deg/m | 1.40 deg/m | +0.10 (+8%) | Worse |
| Heading error (mean) | 0.74 deg | 0.86 deg | +0.12 (+16%) | Worse |
| Loop closures | 68 | 73 | +5 (+7%) | - |
| Scan-match acceptance | 26.9 % | 24.1 % | -2.8 (-10%) | Worse |
| **Mapping vs real scene** | | | | |
| Map accuracy | 10.6 cm | 5.5 cm | -5.1 (-48%) | Better |
| Map completion | 9.2 cm | 8.5 cm | -0.6 (-7%) | Better |
| Precision | 73.9 % | 91.6 % | +17.7 (+24%) | Better |
| Recall | 81.8 % | 85.9 % | +4.1 (+5%) | Better |
| F1 | 77.7 % | 88.7 % | +11.0 (+14%) | Better |
| **Detection** | | | | |
| Detections on a real object | 21.2 % | 23.4 % | +2.2 (+10%) | Better |
| **Processing time** | | | | |
| SLAM update (median) | 10.6 ms | 10.1 ms | -0.5 (-4%) | Better |
| Map update (median) | 20.3 ms | 1.3 ms | -19.0 (-93%) | Better |
| Frontier selection (median) | 155 ms | 21 ms | -134 (-86%) | Better |
| Planning (p95) | 128 ms | 38 ms | -90 (-70%) | Better |
| Slowest single agent call | 1986 ms | 870 ms | -1116 (-56%) | Better |
| Control step (p95) | 380 ms | 173 ms | -206 (-54%) | Better |
| Real-time factor | 0.32 | 0.64 | +0.33 (+103%) | Better |
| **Resources** | | | | |
| Process CPU (mean) | 86 % of a core | 67 % of a core | -19 (-23%) | - |
| Memory (peak) | 2973 MB | 2585 MB | -388 (-13%) | Better |

**Summary:** 29 better, 3 worse, 0 within 3% (of 32 metrics with a direction). Baseline: runs/20261024_210638; new: runs/20261107_161737. Single new run: no variability can be stated.
