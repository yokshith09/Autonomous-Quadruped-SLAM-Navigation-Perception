| Metric | Baseline | New (3 runs) | Change | Verdict |
|---|---|---|---|---|
| **Exploration** | | | | |
| Task completed (1 = yes) | 0 | 1 ± 0 | +1 | Better |
| Rooms completed | 6 | 10 ± 2 | +4 (+67%) | Better |
| Rooms found | 8 | 11 ± 2 | +3 (+33%) | - |
| Simulated time | 1800 s | 3000 ± 0 s | +1200 (+67%) | - |
| Distance travelled | 157.9 m | 260.7 ± 0.4 m | +102.8 (+65%) | - |
| Average speed | 0.088 m/s | 0.087 ± 0.000 m/s | -0.001 (-1%) | Same |
| Real floor mapped | 88.2 % | 91.0 ± 1.7 % | +2.8 (+3%) | Better |
| Free space mapped outside the arena | 0.59 m2 | 1.41 ± 1.54 m2 | +0.83 (+141%) | Worse |
| Frontiers remaining | 48 | 46 ± 9 | -2 (-5%) | Better |
| Stuck events | 4 | 3 ± 1 | -1 (-17%) | Better |
| Unreachable targets | 9 | 17 ± 11 | +8 (+89%) | Worse |
| Room entries per room | 2.25 | 0.80 ± 0.10 | -1.45 (-64%) | Better |
| Path walked after coverage plateau (while exploring) | 46 % | 28 ± 5 % | -18 (-39%) | Better |
| Time until everything reachable was explored | n/a s | 1439 ± 183 s | - | - |
| Patrol walk after exploration | 0.0 m | 135.9 ± 15.9 m | +135.9 | - |
| **Localization** | | | | |
| ATE RMSE | 31.0 cm | 27.9 ± 15.1 cm | -3.1 (-10%) | Better |
| ATE as share of path | 0.20 % | 0.11 ± 0.06 % | -0.09 (-46%) | Better |
| Final drift | 0.24 % of path | 0.11 ± 0.06 % of path | -0.14 (-56%) | Better |
| Largest position error | 63.1 cm | 75.3 ± 21.2 cm | +12.2 (+19%) | Worse |
| RPE translation | 7.4 % | 6.0 ± 0.5 % | -1.4 (-19%) | Better |
| RPE rotation | 1.30 deg/m | 1.31 ± 0.02 deg/m | +0.01 (+1%) | Same |
| Heading error (mean) | 0.74 deg | 0.75 ± 0.01 deg | +0.01 (+1%) | Same |
| Loop closures | 68 | 208 ± 56 | +140 (+205%) | - |
| Scan-match acceptance | 26.9 % | 23.2 ± 0.8 % | -3.6 (-14%) | Worse |
| **Mapping vs real scene** | | | | |
| Map accuracy | 10.6 cm | 9.8 ± 6.3 cm | -0.9 (-8%) | Better |
| Map completion | 9.1 cm | 8.7 ± 3.3 cm | -0.4 (-5%) | Better |
| Precision | 73.9 % | 82.5 ± 13.2 % | +8.6 (+12%) | Better |
| Recall | 82.0 % | 85.7 ± 7.5 % | +3.7 (+5%) | Better |
| F1 | 77.7 % | 84.0 ± 10.5 % | +6.2 (+8%) | Better |
| **Detection** | | | | |
| Detections on a real object | 21.2 % | 22.3 ± 1.9 % | +1.0 (+5%) | Better |
| **Processing time** | | | | |
| SLAM update (median) | 10.6 ms | 9.8 ± 2.1 ms | -0.8 (-7%) | Better |
| Map update (median) | 20.3 ms | 0.5 ± 0.1 ms | -19.8 (-97%) | Better |
| Frontier selection (median) | 155 ms | 14 ± 4 ms | -141 (-91%) | Better |
| Planning (p95) | 128 ms | 9 ± 2 ms | -119 (-93%) | Better |
| Slowest single agent call | 1986 ms | 1047 ± 484 ms | -939 (-47%) | Better |
| Control step (p95) | 380 ms | 152 ± 25 ms | -227 (-60%) | Better |
| Real-time factor | 0.32 | 0.66 ± 0.05 | +0.34 (+106%) | Better |
| **Resources** | | | | |
| Process CPU (mean) | 86 % of a core | 59 ± 3 % of a core | -27 (-31%) | - |
| Memory (peak) | 2973 MB | 1364 ± 25 MB | -1609 (-54%) | Better |

**Summary:** 25 better, 4 worse, 3 within 3% (of 32 metrics with a direction). Baseline: runs/20261024_210638; new: runs/20261112_232106, runs/20261113_120000, runs/20261113_192627. Mean of 3 runs, spread = standard deviation.
