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

## Not done yet
| Item | Status | Needs |
|---|---|---|
| F33 map noise seals 2 rooms | Diagnosed, not fixed | After F1/F2 live check (it is a map-accuracy fix, grouped with F7 to F9) |
| **Localization drift (blocks all-rooms)** | Diagnosing: run 2 error grows in jumps at turns and in the south area, not smoothly along a corridor; replay ablation running to find which SLAM part adds the error | Recording `runs/20261107_011529/recording.npz` |
| Room entries metric | Divides entries by rooms *found*, so 4 entries / 8 found = 0.50 looks "better"; should be entries per room entered | Measurement fix, own commit |
| F3 part 2 room labels drift | Diagnosed, not fixed | After the live check |
| F14 compiled ray-casting (numba) | Proposed | Owner decision on the new dependency |
| Diagnose results 3, 4, 5, 6, 8, 9 (ATE, RPE, map accuracy, ICP acceptance, real time, detection) | Recorded in `FIXES.md`, detailed diagnosis not started | One at a time, in order |
| Doc fix `97096d2` to `main` | Waiting | Owner approval |
| Back up `runs/20261024_210638/recording.npz` (350 MB, not in git) | Owner | External drive or cloud |
| Demo video link in README | Owner | Paste the Drive link at `PASTE_GOOGLE_DRIVE_LINK_HERE` |

## Order of work
1. Result 1 (mission completes): F1, F2 done offline → live check → F33, F3.
2. Results 2 to 9, one at a time: diagnose, fix, verify with `compare_runs`, record here, then move on.
