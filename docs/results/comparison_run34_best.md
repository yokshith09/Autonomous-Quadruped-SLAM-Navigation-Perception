| Metric | Baseline | New (1 run) | Change | Verdict |
|---|---|---|---|---|
| **Exploration** | | | | |
| Task completed (1 = yes) | 0 | 1 | +1 | Better |
| Rooms completed | 6 | 10 | +4 (+67%) | Better |
| Rooms found | 8 | 10 | +2 (+25%) | - |
| Simulated time | 1800 s | 1800 s | +0 (+0%) | - |
| Distance travelled | 157.9 m | 156.9 m | -1.0 (-1%) | - |
| Average speed | 0.088 m/s | 0.087 m/s | -0.001 (-1%) | Same |
| Real floor mapped | 88.3 % | 94.6 % | +6.2 (+7%) | Better |
| Free space mapped outside the arena | 0.59 m2 | 0.16 m2 | -0.42 (-72%) | Better |
| Frontiers remaining | 48 | 51 | +3 (+6%) | Worse |
| Stuck events | 4 | 0 | -4 (-100%) | Better |
| Unreachable targets | 9 | 2 | -7 (-78%) | Better |
| Room entries per room | 2.25 | 0.90 | -1.35 (-60%) | Better |
| Path walked after coverage plateau (while exploring) | 46 % | 48 % | +2 (+4%) | Worse |
| Time until everything reachable was explored | n/a s | 1785 s | - | - |
| Patrol walk after exploration | 0.0 m | 1.3 m | +1.3 | - |
| **Localization** | | | | |
| ATE RMSE | 31.0 cm | 11.7 cm | -19.2 (-62%) | Better |
| ATE as share of path | 0.20 % | 0.07 % | -0.12 (-62%) | Better |
| Final drift | 0.24 % of path | 0.05 % of path | -0.19 (-79%) | Better |
| Largest position error | 63.1 cm | 24.5 cm | -38.6 (-61%) | Better |
| RPE translation | 7.4 % | 3.9 % | -3.5 (-48%) | Better |
| RPE rotation | 1.30 deg/m | 1.29 deg/m | -0.01 (-1%) | Same |
| Heading error (mean) | 0.74 deg | 0.76 deg | +0.02 (+3%) | Same |
| Loop closures | 68 | 84 | +16 (+24%) | - |
| Scan-match acceptance | 26.9 % | 23.8 % | -3.0 (-11%) | Worse |
| **Mapping vs real scene** | | | | |
| Map accuracy | 10.8 cm | 2.8 cm | -8.0 (-74%) | Better |
| Map completion | 8.8 cm | 5.8 cm | -3.0 (-34%) | Better |
| Precision | 73.4 % | 99.7 % | +26.2 (+36%) | Better |
| Recall | 83.0 % | 92.4 % | +9.4 (+11%) | Better |
| F1 | 77.9 % | 95.9 % | +18.0 (+23%) | Better |
| **Detection** | | | | |
| Detections on a real object | 21.2 % | 21.6 % | +0.4 (+2%) | Same |
| **Processing time** | | | | |
| SLAM update (median) | 10.6 ms | 10.6 ms | +0.0 (+0%) | Same |
| Map update (median) | 20.3 ms | 0.6 ms | -19.7 (-97%) | Better |
| Frontier selection (median) | 155 ms | 17 ms | -138 (-89%) | Better |
| Planning (p95) | 128 ms | 9 ms | -119 (-93%) | Better |
| Slowest single agent call | 1986 ms | 702 ms | -1284 (-65%) | Better |
| Control step (p95) | 380 ms | 138 ms | -242 (-64%) | Better |
| Real-time factor | 0.32 | 0.73 | +0.41 (+130%) | Better |
| **Resources** | | | | |
| Process CPU (mean) | 86 % of a core | 67 % of a core | -19 (-22%) | - |
| Memory (peak) | 2973 MB | 930 MB | -2042 (-69%) | Better |

**Summary:** 24 better, 3 worse, 5 within 3% (of 32 metrics with a direction). Baseline: runs/20261024_210638; new: runs/20261113_232046. Single new run: no variability can be stated.
