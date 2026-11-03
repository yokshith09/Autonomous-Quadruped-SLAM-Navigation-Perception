# Progress tracker

What is done and what is not, updated after every step. Fix details and evidence live in `FIXES.md`; the rules live in
`IMPROVEMENT_PROTOCOL.md`. Branch for all work: `improve/exploration`. `main` changes only with the owner's approval.

_Last updated: 2026-11-03_

## Done
| Date | What | Where / evidence |
|---|---|---|
| 2026-10-24 | Baseline run measured (1800 s, 6 of 8 rooms) | `runs/20261024_210638/`, tag `baseline-code-run1` |
| 2026-10-25 | Results, diagnostic, comparison and project report (PDF, Word) | `docs/results/` |
| 2026-11-01 | Repo cleanup, README, baseline tags, `compare_runs` tool, progress curve per run | tags `baseline-v1`, `baseline-code-run1` |
| 2026-11-01 | Tool attribution lines removed from all commit messages (history rewrite, force-pushed) | 0 trailers on every branch; GitHub API contributors = owner only |
| 2026-11-02 | Diagnosis of result 1 (mission never completes) and 2 (wasted exploration) | `FIXES.md` F1, F2, F33 |
| 2026-11-02 | **F1, F2 implemented** (observable-frontier check; retirement by frontier cell, never cleared) | commit `1b37f3f`; offline: old code repeats one target 298/300 times, new code completes in 3 decisions; 2 new tests fail on old code, pass on new |

## Not done yet
| Item | Status | Needs |
|---|---|---|
| F1, F2 live verification | Pending | CoppeliaSim open; full run + `compare_runs` against the baseline |
| Profiling (F12) | Pending | CoppeliaSim open; short run under cProfile |
| F33 map noise seals 2 rooms | Diagnosed, not fixed | After F1/F2 live check (it is a map-accuracy fix, grouped with F7 to F9) |
| F3 room labels flip | Not started | Next after F1/F2 are verified |
| Diagnose results 3, 4, 5, 6, 8, 9 (ATE, RPE, map accuracy, ICP acceptance, real time, detection) | Recorded in `FIXES.md`, detailed diagnosis not started | One at a time, in order |
| Doc fix `97096d2` to `main` | Waiting | Owner approval |
| GitHub sidebar still shows `@claude` | Data is clean (API lists the owner only); the sidebar is a GitHub cache | Owner: switch default branch away and back, or ask GitHub Support to refresh the contributors cache |
| Back up `runs/20261024_210638/recording.npz` (350 MB, not in git) | Owner | External drive or cloud |
| Demo video link in README | Owner | Paste the Drive link at `PASTE_GOOGLE_DRIVE_LINK_HERE` |

## Order of work
1. Result 1 (mission completes): F1, F2 done offline → live check → F33, F3.
2. Results 2 to 9, one at a time: diagnose, fix, verify with `compare_runs`, record here, then move on.
