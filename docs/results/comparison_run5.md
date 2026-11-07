| Metric | Baseline | New (1 run) | Change | Verdict |
|---|---|---|---|---|
| **Exploration** | | | | |
| Task completed (1 = yes) | 0 | 1 | +1 | Better |
| Rooms completed | 6 | 7 | +1 (+17%) | Better |
| Rooms found | 8 | 9 | +1 (+12%) | - |
| Simulated time | 1800 s | 2186 s | +387 (+21%) | - |
| Distance travelled | 157.9 m | 198.8 m | +40.9 (+26%) | - |
| Average speed | 0.088 m/s | 0.091 m/s | +0.003 (+4%) | Better |
| Real floor mapped | 88.2 % | 92.6 % | +4.4 (+5%) | Better |
| Free space mapped outside the arena | 0.59 m2 | 0.58 m2 | -0.00 (-0%) | Same |
| Frontiers remaining | 48 | 59 | +11 (+23%) | Worse |
| Stuck events | 4 | 3 | -1 (-25%) | Better |
| Unreachable targets | 9 | 5 | -4 (-44%) | Better |
| Room entries per room | 2.25 | 0.78 | -1.47 (-65%) | Better |
| Path walked after coverage plateau | 46 % | 47 % | +0 (+0%) | Same |
| **Localization** | | | | |
| ATE RMSE | 31.0 cm | 15.3 cm | -15.6 (-50%) | Better |
| ATE as share of path | 0.20 % | 0.08 % | -0.12 (-61%) | Better |
| Final drift | 0.24 % of path | 0.02 % of path | -0.23 (-93%) | Better |
| Largest position error | 63.1 cm | 38.1 cm | -25.0 (-40%) | Better |
| RPE translation | 7.4 % | 5.9 % | -1.5 (-20%) | Better |
| RPE rotation | 1.30 deg/m | 1.35 deg/m | +0.05 (+4%) | Worse |
| Heading error (mean) | 0.74 deg | 0.80 deg | +0.06 (+8%) | Worse |
| Loop closures | 68 | 144 | +76 (+112%) | - |
| Scan-match acceptance | 26.9 % | 23.4 % | -3.5 (-13%) | Worse |
| **Mapping vs real scene** | | | | |
| Map accuracy | 10.6 cm | 5.5 cm | -5.1 (-48%) | Better |
| Map completion | 9.2 cm | 7.0 cm | -2.1 (-23%) | Better |
| Precision | 73.9 % | 89.5 % | +15.5 (+21%) | Better |
| Recall | 81.8 % | 90.5 % | +8.6 (+11%) | Better |
| F1 | 77.7 % | 90.0 % | +12.3 (+16%) | Better |
| **Detection** | | | | |
| Detections on a real object | 21.2 % | 21.0 % | -0.2 (-1%) | Same |
| **Processing time** | | | | |
| SLAM update (median) | 10.6 ms | 13.3 ms | +2.7 (+26%) | Worse |
| Map update (median) | 20.3 ms | 3.7 ms | -16.6 (-82%) | Better |
| Frontier selection (median) | 155 ms | 39 ms | -116 (-75%) | Better |
| Planning (p95) | 128 ms | 16 ms | -112 (-88%) | Better |
| Slowest single agent call | 1986 ms | 1219 ms | -767 (-39%) | Better |
| Control step (p95) | 380 ms | 235 ms | -145 (-38%) | Better |
| Real-time factor | 0.32 | 0.53 | +0.22 (+68%) | Better |
| **Resources** | | | | |
| Process CPU (mean) | 86 % of a core | 72 % of a core | -14 (-16%) | - |
| Memory (peak) | 2973 MB | 3072 MB | +99 (+3%) | Worse |

**Summary:** 23 better, 6 worse, 3 within 3% (of 32 metrics with a direction). Baseline: runs/20261024_210638; new: runs/20261107_150823. Single new run: no variability can be stated.
