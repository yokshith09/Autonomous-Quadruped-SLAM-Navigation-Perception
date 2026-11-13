# Progress tracker

What is done and what is not, updated after every step. Fix details and evidence live in `FIXES.md`; the rules live in
`IMPROVEMENT_PROTOCOL.md`. Branch for all work: `improve/exploration`. `main` changes only with the owner's approval.

_Last updated: 2026-11-07_

## Goal
The robot explores **every room** of an unknown building on its own (no map, no prior knowledge), **remembers** what it has
covered and **never goes back** to an area or frontier it has already seen, builds an **accurate map**, knows **where it
is** with small error, and **detects the objects** in each room.

| Goal part | Measured by | Baseline | Target | Fixes |
|---|---|---|---|---|
| All rooms explored, mission ends by itself | Rooms visited / mission complete | 6 of 8, timeout | 8 of 8, MISSION_COMPLETE | F1, F2, F33, F3, F4 |
| No revisits | Path walked after coverage plateau; room entries per room | 46 %; 2.25 | < 10 %; about 1 | F1, F2, F3, F6 |
| Remembers covered areas | Retired frontiers never re-chosen; room labels stable | not tracked | 0 re-chosen | F2, F3 |
| Localization | ATE; RPE | 31 cm; 7.4 % | <= 15 cm; <= 3 % | F16, F7, F9 |
| Mapping | Wall accuracy; precision vs the real scene | 10.6 cm; 73.9 % | <= 5 cm; >= 85 % | F8, F33 |
| Object detection | Precision of detected objects | 21 % | >= 80 % | F28, F29 |
| Runs in real time | Real-time factor | 0.32 | >= 1 | F12, F13, F14 |

## Plan (one step at a time: diagnose, fix, verify against the baseline, record here)
1. Profile (F12) and live-verify F1, F2. Done; F3 part 1 added after the first live check.
2. **Localization first** (run 2 showed pose drift, not frontier logic, now stops exploration): F16 then F7, F9:
   which SLAM part adds the error, why 73 % of scan matches are rejected; cut ATE and RPE. Then F8 walls.
3. F33 (map noise seals rooms), F3 part 2 (stable room labels): all 8 rooms reachable, each entered once.
4. F13, F14, F5: stalls and speed.
5. F28, F29: object detection from the colour image; real knowledge graph.

## Done
| Date | What | Where / evidence |
|---|---|---|
| 2026-10-24 | Baseline run measured (1800 s, 6 of 8 rooms) | `runs/20261024_210638/`, tag `baseline-code-run1` |
| 2026-10-25 | Results, diagnostic, comparison and project report (PDF, Word) | `docs/results/` |
| 2026-11-01 | Repo cleanup, README, baseline tags, `compare_runs` tool, progress curve per run | tags `baseline-v1`, `baseline-code-run1` |
| 2026-11-01 | Tool attribution lines removed from all commit messages (history rewrite, force-pushed) | 0 trailers on every branch; GitHub API contributors = owner only; sidebar confirmed clean by the owner 2026-11-03 |
| 2026-11-02 | Diagnosis of result 1 (mission never completes) and 2 (wasted exploration) | `FIXES.md` F1, F2, F33 |
| 2026-11-02 | **F1, F2 implemented** (observable-frontier check; retirement by frontier cell, never cleared) | commit `1b37f3f`; offline: old code repeats one target 298/300 times, new code completes in 3 decisions; 2 new tests fail on old code, pass on new |
| 2026-11-07 | **Profiling (F12)**: ray-casting about 40 % of loop time, simulator calls about 30 %, SLAM 10 %, headless HUD 9 % | run `20261107_003107`; `FIXES.md` F12 |
| 2026-11-07 | **F14 part 1**: ray-casting 1.33x faster, map bit-identical; HUD frontiers only on HUD frames | commits `3f38a34`, `638ea9b` |
| 2026-11-07 | **First live check of F1, F2**: failed. Livelock from 660 s: active room alternated 4 / 6 twenty times, one target picked 19 times (stopped at 930 s) | run `20261107_004151`, `runs/live_f1f2.log` |
| 2026-11-07 | **F3 part 1 implemented**: an unfinished room is never abandoned on a label change | new test fails on old code, passes on new |
| 2026-11-07 | **Live run 2 (F1, F2, F3 part 1)**: exploration behaviour fixed (path after coverage plateau 46 % -> 8 %, each room entered once, no repeated targets, mission ends by itself at 1011 s; real-time factor 0.32 -> 0.51). **But** ATE 31 -> 73 cm on this route; the smeared map sealed the rest, so the completion is false (4 rooms cleared, 77 frontiers unreachable, map F1 77.7 % -> 59.7 %) | run `20261107_011529`, `compare_runs` against the baseline |

| 2026-11-07 | **F34**: target kept until reached / cleared / unreachable / stalled (was swapped every 9 s, about 0.8 m) | commit `626bd35`, 2 tests |
| 2026-11-07 | **F35**: `grid_to_world` returns cell centre (22 % of cells landed one cell off) | commit `504d636`, test |
| 2026-11-07 | **A* audit**: optimal (matches Dijkstra on the run-2 map); skips stale entries, 1.4x faster. Blue line off the robot = path drawn in SLAM coordinates, offset by the pose error | commit `c3c1dcf` |
| 2026-11-07 | **numba** ray-casting kernel: 0.17 vs 49.8 ms per update, bit-identical | commit `654650c`, test |
| 2026-11-07 | **F36 localization**: scan-to-submap + drift-bounded loop gate. Replay ATE run 2 72.9 -> 21.8 cm, baseline route 31.0 -> 23.3 cm | commit `d939b26`, 2 recordings |
| 2026-11-07 | **Live run 3 (F1-F3, F34-F36)**: 11 room labels cleared by 1260 s, ATE 32 cm at 1200 s; then heading collapse (22 deg), stopped at 1500 s | run `20261107_112958` |
| 2026-11-07 | **F37**: closures correct position only; ICP fallback bug fixed. Replay ATE 24.8 / 22.0 / 17.8 cm on three routes, heading <= 0.8 deg | commit `bb46462` |
| 2026-11-07 | **Live run 4 (F1-F3, F34-F37): mission completed by itself at 1872 s; all 7 real rooms explored (98-100 % of each room's floor seen, true path entered every room)**. vs baseline: ATE 31.0 -> 26.9 cm, RPE 7.4 -> 4.6 %, final drift 0.24 -> 0.09 %, map F1 77.7 -> 82.8 %, map accuracy 10.6 -> 7.4 cm, real-time factor 0.32 -> 0.47; 20 metrics better, 8 worse | run `20261107_125826`, `docs/results/comparison_run4.md` |
| 2026-11-07 | **Real room count**: 7 enclosed regions from the scene walls (`evaluation/room_coverage.py`); the robot's own labels reached 13 IDs. Correction: the baseline also saw 87-100 % of every real room; its '6 of 8 rooms' was a label count | `runs/*/room_coverage.json` |
| 2026-11-07 | **Live run 5 (adds F38-F40, compiled wavefront)**: completed by itself at 2186 s; all 7 real rooms explored (88-100 % of floor). vs baseline: **ATE 31.0 -> 15.3 cm**, largest error 63 -> 38 cm, final drift 0.24 -> 0.02 %, **map accuracy 10.6 -> 5.5 cm, map F1 77.7 -> 90.0 %**, precision 73.9 -> 89.5 %, frontier selection 155 -> 39 ms, real-time factor 0.32 -> 0.53; 23 metrics better, 6 worse. Still open: 47 % of walking after the coverage plateau (doorway tour, F41 fixes) | run `20261107_150823`, `docs/results/comparison_run5.md` |
| 2026-11-07 | Rejected after replay: soft-pull SLAM correction (worse on 3 of 4 routes) | not committed |
| 2026-11-07 | **Live run 6 (adds F41 doorway skip)**: all 7 real rooms 99-100 % seen, everything explored at **1604 s** (run 5: 2186 s). vs baseline: walking after coverage plateau 46 -> 29 %, ATE 31.0 -> 16.1 cm, largest error 63 -> 33 cm, map accuracy 10.6 -> 5.5 cm, precision 73.9 -> 91.6 %, real-time factor 0.32 -> 0.64, peak memory -13 %; **29 better, 3 worse** | run `20261107_161737`, `docs/results/comparison_run6.md` |
| 2026-11-07 | **Live run 7 (50 min mission, adds F42 pure-pursuit steering)**: all 7 real rooms; everything explored at 1736 s, then **idle** (found the F43 bug; stopped at 2100 s). vs baseline: **ATE 14.6 cm (target <= 15 met)**, **map accuracy 4.1 cm (target <= 5 met)**, map F1 91.7 %, real-time factor 0.59; steering turn reversals 12.7 -> 6.9 /min, yaw wobble 10.0 -> 7.9 deg/s; 28 better, 3 worse (largest error 69 cm) | run `20261107_170841`, `docs/results/comparison_run7.md` |
| 2026-11-07 | **F43 patrol after exploration** instead of idling | commit `2a9988b` |
| 2026-11-07 | **Live run 8: first full 50 min mission (adds F43 patrol)**: all 7 real rooms (98.6-100 % floor), everything explored at 1736 s, then 62 patrol targets with no idle time; patrol re-opened frontiers and exploration resumed by itself. vs baseline: **ATE 14.7 cm, map accuracy 4.4 cm, map precision 95.4 %, F1 93.4 %**, stuck events 4 -> 2, real-time factor 0.55; 25 better, 4 worse. Worse by design of the 50 min rule: path after coverage plateau 61 % (patrol walking counts as waste; metric needs redefining); peak memory 4.4 GB (recording buffer over 50 min, F15) | run `20261107_180907`, `docs/results/comparison_run8.md` |
| 2026-11-07 | **F44 doorway turning cost** (no turning round for a slightly nearer doorway behind) | commit `ff74539`, live check in next run |
| 2026-11-12 | Measurement: wasted walking counted only while exploring (run 8: 61 -> 33 %); recording streamed to disk (F15); replay uses recorded odometry; robot parts no longer scored as wall | commits `3532a2f`, `7181ecb`, `f02ea6a`, `b3e2f7e` |
| 2026-11-12 | Live runs 9-11 with the new gait: run 9 odometry calibration error (fixed), run 10 wall contact with 122 cm phantom drift, run 11 wedged at Cylinder3 and seen upside down. Diagnosed from recordings and simulator re-enactment; fixed by F45-F50 | runs `20261112_101356`, `20261112_103326`, `20261112_105216` |
| 2026-11-12 | Rejected after replay: loop-closure temporal-consistency check (inconsistent across routes) | `RESEARCH.md` |
| 2026-11-12 | **Live run 12 (F45-F50)**: all 7 real rooms explored by **1072 s (fastest)**, wasted walking while exploring 22 % (best), 3 single contacts and no loops or tipping, real-time factor 0.69, peak memory 606 MB (-80 %). Patrol phase: false loop closures against keyframes 40-142 m old (gate opens to 1.5 m) pushed the error to 77 cm repeatedly; whole-run ATE 34.6 cm. Stopped at 2100 s | run `20261112_111339`, `docs/results/comparison_run12.md` |
| 2026-11-12 | Loop-closure gate cap replay (0.3 / 0.4 / 0.6 m): run 2 24.8 -> 12.9, baseline 22.0 -> 17.6, run 12 34.6 -> 14.5 cm, but run 8 14.7 -> 43.5 cm (its large corrections were genuine). Not adopted: closures need verification beyond the 2D scan (camera check) | replay, 4 recordings |

## Not done yet
| Item | Status | Needs |
|---|---|---|
| F33 map noise seals 2 rooms | Diagnosed, not fixed | After F1/F2 live check (it is a map-accuracy fix, grouped with F7 to F9) |
| Wasted walking after coverage plateau 35 % (target < 10 %); stuck events 17 vs 4 | Open | Diagnose from run 4 |
| Localization ATE 27 cm (target <= 15 cm); scan-match acceptance 25 % | Open | F16, F7 |
| Path line drawn in SLAM frame, looks off the robot | Open | Draw the estimated pose too |
| Gait speed 0.09 m/s, not smooth | Open | F5, F23 |
| Frontier selection 215 ms median, memory peak 3.3 GB | Open | numba geodesic field (F14), stream recording (F15) |
| F3 part 2: room labels drift (run 3 used IDs up to 13 for 8 rooms) | Open | Freeze labels of explored rooms |
| F14 frontier geodesic field and A* still in Python | Open | numba next if they show up in the profile |
| F7 RPE, F16 scan-match rejection reasons, ATE <= 15 cm | Open | ATE now about 22 cm on replay |
| Room entries metric | Divides entries by rooms *found*, so 4 entries / 8 found = 0.50 looks "better"; should be entries per room entered | Measurement fix, own commit |
| F3 part 2 room labels drift | Diagnosed, not fixed | After the live check |
| Diagnose results 3, 4, 5, 6, 8, 9 (ATE, RPE, map accuracy, ICP acceptance, real time, detection) | Recorded in `FIXES.md`, detailed diagnosis not started | One at a time, in order |
| Doc fix `97096d2` to `main` | Waiting | Owner approval |
| Back up `runs/20261024_210638/recording.npz` (350 MB, not in git) | Owner | External drive or cloud |
| Demo video link in README | Owner | Paste the Drive link at `PASTE_GOOGLE_DRIVE_LINK_HERE` |

## Order of work
1. Result 1 (mission completes): F1, F2 done offline → live check → F33, F3.
2. Results 2 to 9, one at a time: diagnose, fix, verify with `compare_runs`, record here, then move on.

## 2026-11-12: "spinning in place left/right" investigated
- Runs 20261112_101356 / _103326 / _105216 (before F48, 11:07) show it: turn commands up to 0.18 rad/step (206 deg/s), 200+ direction changes in 20 s, under 5 cm moved. F48 (rate cap 0.06) fixed it.
- Run 12 (after F48): no dithering. Only 4 back-to-back opposite pivots in 2107 s. What remains are 180 deg turn-arounds when a room closes and the next target lies behind (14 during exploration; all pivots 87 s of 1072 s). Expected at dead-end rooms; not changed.
- Fixed: the frontier "room explored" print used a star glyph that crashes a non-UTF-8 console inside target selection (test_agents failed under cp1252). Now ASCII. All 4 test suites pass.

## 2026-11-12: T1, best configuration frozen (tag best-sim-v1)
Rejected: loop gate measured from the last accepted closure instead of from the keyframe. Replay ATE: baseline 22.0 -> 40.9 cm, run 7 14.6 -> 34.9 cm (a small accepted closure resets the gate and blocks the later genuine large ones). Reverted.

Best results so far:
| Measure | Run 8 (20261107_180907) | Run 12 (20261112_111339, current code) |
|---|---|---|
| Real rooms explored | 7 / 7 | 7 / 7 |
| Time to explore all | 1736 s | **1072 s** |
| Wasted walking while exploring | 33 % | **22 %** |
| ATE (whole 50 min run) | **14.7 cm** | 34.6 cm (patrol false closures) |
| Map accuracy vs real scene | **4.4 cm** | 7.0 cm |
| Map F1 vs real scene | **93.4 %** | 87.3 % |
| Contacts / tipping | 2 stuck | 3 single hits, no loops, no tipping |
| Real-time factor / peak memory | 0.55 / 4.4 GB | **0.69 / 606 MB** |
Current code = run 12. Remaining gap: false loop closures during patrol; the fix is a camera check on closures (needs RGB recorded).

## 2026-11-12 (evening): loop-closure study, F51 camera check, F52 recovery
Audit of every accepted closure against the true pose (runs 8 and 12): about a third help, a third hurt, a third change nothing; RMSE, match ratio and keyframe age do not separate them. One wrong closure (87 cm) in run 12 and one (70 cm, keyframe 38 m old, 0 agreeing camera features) in run 13.

Replay ATE (cm), same recordings, closures on / off:
| Recording | Closures on | Closures off |
|---|---|---|
| baseline 20261024_210638 | 22.0 | 17.3 |
| run 7 | 14.6 | 58.7 |
| run 8 | 14.7 | 64.4 |
| run 12 | 34.6 | 38.1 |
| run 13 (830 s) | 35.5 | 39.5 |
| run 14 (813 s) | 27.2 | 13.1 |
| run 15 (620 s) | 28.1 | 15.9 |
Reading: on long missions closures are essential (run 7/8: 15 vs 60 cm); short routes can do without. The goal is to refuse only the wrong ones.

- **F51 camera check** (core/loop_verify.py): a scan-accepted closure against a keyframe more than 10 m of walking old needs at least 5 agreeing ORB features between the current view and the keyframe's view. Run 13 replay 35.5 -> 10.6 cm (the data it was tuned on), run 14 27.2 -> 11.8, run 15 28.1 -> 32.5. First live version (all ages, 15 features) refused good early closures and was worse (run 13: 35 cm at 600 s); the age limit fixed that. Recordings now keep a 192x144 grayscale view every 0.3 s.
- **F52 recovery**: runs 13 and 14 tipped within 0.4 s of the first reverse step. A turn/walk dither was sampled mid-walk by the stall watchdog and called a stall; the reverse gait then flipped the body. Now a stall needs 80 % of the last second walking, and recovery stops the legs and turns 70 deg towards the open side. Two tests fail on the old code and pass on the new.
- Not adopted: gate measured from the last closure (baseline 22 -> 41, run 7 14.6 -> 35); odometry scale 1.08 (run 14 27 -> 51, run 15 no-closure 16 -> 24); degeneracy-aware closures (core/slam.py `degeneracy_ratio`, default off): ratio 0.15 better on 4 of 7 recordings, worse on 3 (mean 25.2 -> 21.7 cm).
- Same code, different routes: 7 cm to 32 cm average error at 600 s (runs 14 vs 15). Accuracy claims need several full runs.

### F51 camera check: not shown to help, switched off by default (--loop-camera-inliers 5 turns it on)
Run 16 (live, camera on, 1215 s, no tip-over): 32 cm at 1200 s; replay of its own recording: camera on 32.2 cm, scan only 20.6 cm. The check refused two genuine closures against 65-79 m old keyframes (+16 and +10 cm) because the views differ in heading (0 agreeing features), exactly as it refuses wrong ones.
Rule tuned on run 13 only. On the three recordings it was not tuned on (scan only -> camera): run 14 27.2 -> 11.8, run 15 28.1 -> 32.5, run 16 20.6 -> 32.2; mean 25.3 -> 25.5. No demonstrated gain, so it is off by default. Camera views are still recorded for research. A heading-independent place check is the open problem.
Live run 16 also fired the F52 turn-away recovery twice (legs loaded at (-3.96, -2.31)) with no tip-over.

### F53 closure cap 1.5 -> 0.70 m (2026-11-12 night)
Live run 17 (config of runs 12-16): a single closure at 896.6 s applied 87 cm against a keyframe 33 m old, error 40 -> 98 cm, 56 frontiers then "unreachable", mission declared complete. Same signature as run 12 (87 cm), run 13 (70 cm). The largest closure that was right in any recording is 65 cm (run 8).
Replay ATE (cm), cap 1.5 -> 0.70: run 7 14.6 -> 14.6, run 8 14.7 -> 14.7, run 12 34.6 -> 22.8, run 13 35.5 -> 35.5 (its jump is 0.70 m, just under), run 17 28.3 -> 25.5 (max error 42 cm). Never worse. Adopted.
Series note: runs 17 and 18 ran with cap 1.5; run 19 runs with 0.70.
Run 17: failed at 897 s (98 cm), stopped at 910 s. Run 18: 300 s 10 cm, 600 s 9 cm average, F1 99 %.

### Series runs 17-19 (final-code check, 2026-11-12/13)
| Run | Cap | Outcome |
|---|---|---|
| 17 (20261112_222428) | 1.5 m | Failed at 897 s: wrong 87 cm closure (keyframe 33 m old), error 40 -> 98 cm. Stopped at 910 s. |
| 18 (20261112_225001) | 1.5 m | Failed at 726 s: wrong 59 cm closure (keyframe 18 m old), error 12 -> 51 cm. Stopped at 920 s. |
| 19 (20261112_232106) | 0.70 m (F53) | **Full 3000 s mission, no tip-over.** ATE 16.5 cm, map accuracy 4.7 cm, F1 92.0 %, 7/7 rooms (floor seen 88-100 %), 92.8 % floor mapped, everything reachable explored at 1371 s, 32 % wasted walking while exploring, RTF 0.69, peak memory 1339 MB, 252 closures, 4 stuck events. vs baseline: 26 better, 1 worse, 5 same (docs/results/comparison_run19.md). |
Gate slope test (3.0 / 1.5 / 1.0 cm per metre, cap 0.70): helps runs 13 and 18 (35.5 -> 24.4, 27.2 -> 9.5) but costs runs 7 and 8 about 11 cm each (14.6 -> 25.5, 14.7 -> 26.3), so the slope stays 3.0.
vs run 8 (14.7 cm, 4.4 cm, F1 93.4 %): run 19 is slightly behind on accuracy (16.5 cm, 4.7 cm, 92.0 %) and 365 s faster to explore everything. Only one run uses the final code; runs 20 and 21 follow.

### Correction: the simulator is repeatable on identical code (2026-11-13)
Run 20 (identical code to run 19) matched run 19 line for line at 30, 60, 90, 300 and 600 s (rooms, explored %, ATE, F1), so it was stopped at ~650 s as a duplicate. The large swings between runs 13-18 therefore came from small code changes (each changes the route, and the route is chaotic), not from random run-to-run noise. Consequences: (1) repeating a run on the same code adds nothing; independent samples need a changed condition (injected sensor noise with a seed, or a different start pose); (2) results hold for this route and scene, and a different route can break (runs 17 and 18 were a different route from run 19 on a 0.70 m looser cap). Next: seeded sensor-noise stage (depth noise and dropouts, gyro bias, odometry slip), which also covers the sim-to-real question.

### F54 tried and reverted: run 8's gait with all later fixes (live run 22, 2026-11-13)
Run 8's code is tagged `run8-code` (ff74539). F54 put its gait back (leg lift 0.025, walk command 1.0, its speed curve) on top of F46-F53, and recalibrated the stall detector at command 1.0 (stall_probe: free max 0.0118, blocked min 0.0242; threshold stays 0.015). Live run 22 stopped at 620 s: error 4 cm at 30 s, 29 cm at 300 s, +38 cm slip at 390-420 s (no closure), 69 cm at 600 s (ATE 45 cm), odometry -15.2 % vs -12.6 % with the F45 gait; 3 contacts in the first 300 s, no tip. Run 19 (F45 gait, same fixes) was 10 cm at 600 s. The F45 bench test had already shown the old gait sliding backward in 18 % of samples, which makes the body travel less than the gait model reports. Run 8 relied on 202 closures to repair this over 50 min. Reverted to the run-19 files (core/robot.py, agents/locomotion_agent.py, tests/test_agents.py from best-sim-v2). Open: the F45 gait is faster on the ground (0.090 vs 0.079 m/s measured) and shakier to the eye; a slower F45 gait (lower walk command) is the next candidate.

### Final-run configuration (2026-11-13)
Rejected: confirming a large closure (>0.20/0.30 m) by a second match (any or same keyframe, 6 or 15 s): fixes run 18 (27.2 -> 9.5 cm) but blocks the genuine large corrections of run 7 (14.6 -> 43.4) and run 8 (14.7 -> 42.9). Off by default (`confirm_above_m`).
Replays of run 19's recording (the route the next identical-code run repeats): base 16.5 cm (max 62); degeneracy 0.15: 15.0 (max 43); gate 1.5-2.0 cm/m: 19.8; camera check: 22.8. Adopted: `degeneracy_ratio` default 0.15 (also better on 4 of 7 recordings, mean 25.2 -> 21.7 cm).
F56: live run 23 hit an obstacle the camera cannot see five times at (3.0, 0.1) within seconds (turn-away, then the planner routed straight back). Contacts now mark one map cell 0.25 m ahead as occupied, re-applied after every camera update. Run 23 stopped at ~160 s; final run restarted as run 24.
F57: planner kept re-planning from scratch every 4 s to the same goal, so the blue line flipped between two near-equal routes (left/right of an obstacle) and the robot turned to follow. A re-plan to the same goal now keeps the current path unless the new one is at most 85 % as long, or the current one is blocked. Applies from the next run (run 24 was already running).

### Final run 24 (20261113_120000), code cf2102a: F55 degeneracy 0.15 + F56 contact memory on run 19's code
Full 3000 s, no tip-over, 2 contacts (no repeat at the same spot), everything reachable explored at 1257 s, 7/7 real rooms, wasted walking while exploring 21 %, ATE 17.9 cm (largest 58.5 cm, first > 0.5 m at 1255 s), map accuracy 6.0 cm, F1 90.7 %, floor mapped 91.7 %, RTF 0.58, peak memory 1354 MB. vs baseline: 24 better, 4 worse, 4 same (docs/results/comparison_run24.md).
vs run 19: faster to explore everything (1257 vs 1371 s), less wasted walking (21 vs 32 %), fewer contacts (2 vs 4); accuracy a little worse (17.9 vs 16.5 cm ATE, 6.0 vs 4.7 cm map, F1 90.7 vs 92.0 %). F57 (path hysteresis) was committed during the run and is not in it.

### Root-cause investigation of the remaining drift (2026-11-13, all rejected or reverted)
- Scan-match rejection (run 24 replay): 68.7 % rejected for disagreeing with odometry (median 3.7 cm in a 0.4 cm step), 22.6 % accepted.
- F58 level scans by body roll/pitch: walking tilt is real (tilt_probe: roll -5.3..+5.8 deg, p95 5.8 deg; camera rigidly mounted). Live run 25 (stopped at 460 s): acceptance 17.6 % vs run 24's 20.2 % over the same 460 s, ATE 14.2 vs 11.4 cm. Reverted.
- Gate after removing the along-wall slide: worse (run 18 27.2 -> 32.2, run 7 14.6 -> 22.2). Removed.
- Ground truth per step (run 24, first 460 s): scan-match step error median 3.0 cm, odometry 0.38 cm. Scan-to-scan (2.2 cm) and 3-keyframe submaps (2.9 cm) are no better than 8 keyframes, and ATE is worse (34.0 / 27.6 vs 11.4 cm).
- Is the scan-matched pose closer to the truth than the odometry pose? Gap < 1.4 cm (accepted): 50 %; 1.4-3 cm: 48 %; 3-6 cm: 39 %; 6-10 cm: 22 %; > 10 cm: 8 %. The gate rejects the right steps.
- More heading from the scan match: icp_yaw_weight 0.3 -> 11.2 cm, 0.6 -> 21.0, 1.0 -> 38.5; manhattan_gain 0.3 -> 34.4 (base 11.4). Rejected.
- Step from odometry, scan match only as a fit check: 11.4 -> 16.2 cm. The accepted matches keep the pose consistent with the map. Rejected.
Conclusion: the front end is limited by a 60 deg depth camera sliced to 2D; per-step matches carry little information and the error between closures is drift that only loop closures repair. Further gains need a different front end (3D depth matching or a wider sensor), not tuning.

### 3D matching tested (2026-11-13): does not help, 2D front end kept
Recorded a 600 s run with the full 3D view (6 cm voxels, ~990 points per view; `--record-3d`, ~250 MB per 10 min) and compared front ends with loop closures off (evaluation/icp3d_probe.py, 6-DoF point-to-plane ICP against the last 6 3D keyframes, seeded by odometry; self-check recovers a synthetic shift exactly). ATE on runs/20261113_161435: odometry only 95.3 cm; 2D front end 39.8 cm; 3D 60.1 cm; 3D without floor 67.7; 3D with gyro heading and no floor 62.4; 3D with gyro heading 228.4; 3D, 12 keyframes 113.5. Full system with loop closures: 14.5 cm. The 3D view is mostly floor and flat wall, which constrain height and tilt but not position on the floor. 3D recording stays available behind --record-3d; nothing changed in the live pipeline.

### Run 8 does not reproduce (2026-11-13)
Reran run 8's code in a separate worktree: tag run8-code (ff74539, the commit in run 8's manifest, written at the end of the run) and 2a9988b (the commit when run 8 started; F44 was committed during the run). Both reruns are identical to each other and diverge from run 8 from 30 s: 30 s 16.5 % explored / 5 cm / now 9 cm vs run 8's 16.1 % / 3 / 4; 60 s 21.1 % / 8 / 16 vs 19.0 % / 4 / 5; the first rerun reached 84 cm error at 90 s and ATE 75 cm, F1 39 % at 300 s. Scene file in the repo unchanged, so the environment (the scene as loaded in CoppeliaSim, or library versions) differs from 2026-11-07. Run 8's 14.7 cm cannot be reproduced today; run 19 (best-sim-v2) is the best reproducible result.
Final candidate: run 19's code (best-sim-v2) + F56 contact memory + F57 stable path; F55 degeneracy off (default 0.0). Confirmation run 29.

### Confirmation run 29 failed; F59 closure cap 0.70 -> 0.50 m (2026-11-13)
Run 29 (20261113_171057: run 19 code + F56 + F57): healthy to 1837 s (error 3 cm, ATE 19 cm, 0 contacts, all explored at ~1800 s), then one wrong 67 cm closure against a keyframe 176 m old -> 70 cm; stopped at 2100 s (ATE 30 cm). Degeneracy 0.15 would not have stopped it (replay 28.7 vs 30.7 cm).
Cap 0.70 -> 0.50 replay: run 29 30.7 -> 17.6, run 19 16.5 -> 15.2, run 24 17.9 -> 19.0; old-gait runs 7 and 8 get worse (14.6 -> 43.4, 14.7 -> 38.4) because that gait slipped more and needed corrections over 50 cm. With the current gait every closure over 50 cm seen was wrong (59, 67, 70, 87, 87 cm in runs 18, 29, 13, 12, 17). Adopted 0.50 for the current gait. Confirmation run 30 follows.

### Test-condition findings (2026-11-13)
- Start pose was not constant: runs 1-11 and 24+ start at (4.000, 4.000, -72.6 deg); runs 12-17 at (4.276, -1.770, 96.6 deg); runs 18-22 (incl. run 19) at (4.276, -1.770, -142.9 deg). Cause unknown (scene state in CoppeliaSim on 2026-11-12; back to the file's pose after restart). Runs from different start poses are different conditions and must not be ranked against each other as code changes.
- The simulator is not always repeatable: runs 19/20 matched line for line, but runs 29/30 (same start pose, code identical until a >50 cm closure) differ from the first target. A run is one sample.
