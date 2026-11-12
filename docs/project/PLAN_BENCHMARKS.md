# Plan: best results, then benchmarks, then the real robot (set 2026-11-12)

Order agreed with the user:
1. Best results in simulation (freeze a configuration).
2. Benchmark and compare (target: end of day 2026-11-12).
3. Real-robot testing.
4. Last: integrate the team's own Video-Mamba + GConv model (about 80% accuracy). YOLO is dropped.

## Tasks for today
| # | Task | Done when |
|---|---|---|
| T1 | Freeze the best configuration: re-score run 12 and run 8 with the same tools, and tag the commit | Tag `best-sim-v1`; metrics table in PROGRESS.md |
| T2 | Sim-to-real noise stage: depth noise and dropouts, gyro bias drift, odometry slip; replay the recorded routes under it | ATE, map F1 and coverage reported with noise off and on |
| T3 | Cartographer comparison: export the recorded scans and odometry to a rosbag, run Cartographer 2D (Docker/WSL), and score both systems with the same ATE/RPE tool | Side-by-side table on the same routes |
| T4 | TUM RGB-D: convert depth to a 2D scan, run our SLAM, and score with TUM's evaluation tool; compare with ORB-SLAM2's published numbers | ATE on fr1/fr2 sequences (2D, so x-y only, stated clearly) |
| T5 | Exploration comparison: coverage over time, path length, collisions, rooms finished; compare with a classic nearest-frontier baseline (Yamauchi) run in the same scene | Table plus coverage curve |
| T6 | Write-up: one comparison document with every number traced to a run or a paper | docs/results/benchmarks.md |

## Risks
- Cartographer needs ROS. On Windows that means Docker or WSL, which takes setup time.
- Each TUM sequence is a large download (hundreds of MB), so ask before downloading.
- Our scan is a 2D slice taken from depth, so TUM comparisons cover x-y position and yaw only.

## Semantic frontier selection (for later, after the model is integrated)
Each frontier is scored by distance and unknown area, as today, **plus** a semantic value: how likely it is to show something that matters. In a crime scene that means near a detected anomaly (a fallen chair or an object on the floor), and rooms or doorways where the anomaly classifier fired with low confidence. The detector feeds per-cell "interest" into the grid. The frontier score adds `w * interest`, and an "inspect" goal circles the object at a safe distance to get views from several angles.

Related work:
- VLFM (Yokoyama et al., ICRA 2024)
- ESC (Zhou et al., ICML 2023)
- L3MVN (Yu et al., IROS 2023)
- SemExp (Chaplot et al., NeurIPS 2020)

## Exploration and SLAM papers to compare with or cite
I listed these from memory. Check each one before citing it.
- Yamauchi 1997, frontier-based exploration (the baseline).
- Stachniss, Hähnel & Burgard 2004, exploration with active loop closing. This matches our patrol drift problem.
- Bormann et al. 2016, room segmentation survey (ICRA). This is what vacuum robots use.
- Umari & Mukhopadhyay 2017, RRT exploration (IROS).
- Bircher et al. 2016, receding-horizon next-best-view (ICRA).
- Dai et al. 2020, fast frontier information-driven exploration (ICRA).
- Zhou et al. 2021, FUEL (RA-L).
- Cao et al. 2021, TARE (RSS).
- Dang et al. 2020, GBPlanner (JFR).
- Chaplot et al. 2020, Active Neural SLAM (ICLR).
- Hess et al. 2016, Cartographer.
- Mur-Artal & Tardós 2017, ORB-SLAM2.
- Sturm et al. 2012, TUM RGB-D.
- Kümmerle et al. 2009, "On measuring the accuracy of SLAM algorithms" (the evaluation method).
