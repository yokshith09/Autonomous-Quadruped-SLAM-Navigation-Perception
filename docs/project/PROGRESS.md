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
