# Improvement protocol

How to improve the system from the baseline without losing the ability to say whether a change helped.

## Reference points (do not move or rewrite these)
| What | Where |
|---|---|
| Baseline code that produced the run | git tag `baseline-code-run1` |
| Baseline with its documents | git tag `baseline-v1` |
| Baseline results | `runs/20261024_210638/` (metrics, trajectory, report), `docs/results/` (PDFs, Word) |
| Baseline raw sensor recording | `runs/20261024_210638/recording.npz` (350 MB, **not in git**, SHA-256 starts `321fdb51f4d679ac`) |

The recording exists only on one computer. Copy it somewhere safe (external drive or cloud) before changing anything: it is the only way to prove that a speed-up leaves the SLAM result unchanged.

## Rules
1. **`main` stays at the baseline.** All work (fixes, runs, tests, comparisons) happens on a branch such as `improve/exploration`. A change is merged to `main` **only with the project owner's explicit approval**, after the measurements below show it helps; never automatically.
2. **One fix per commit**, named by its ID in `FIXES.md` (for example `fix(F1): ...`). Never mix a behaviour change with a measurement change in one commit.
3. **Do not change how a number is measured while improving the thing measured.** If a metric turns out to be wrong, fix it in its own commit and rebuild the baseline report.

## Checklist for every change
| Kind of change | Required evidence |
|---|---|
| Speed-up with no intended behaviour change (F12 to F15) | Tests pass; replay of the baseline recording gives the **same trajectory** as before (`python -m evaluation.ablate runs/20261024_210638`, full-system row unchanged); wall time before and after |
| Behaviour change (F1 to F9, F28, F29) | Tests pass; at least one live run, ideally three with different seeds, each scored with `python -m evaluation.truth_eval`; the acceptance check written in `FIXES.md` is met |
| Any change | `python tests/test_agents.py`, `python tests/test_slam_accuracy.py`, `python tests/test_run_telemetry.py` |

## Comparing with the baseline
One command prints the whole comparison, with a Better / Worse / Same verdict per metric and the mean and spread when several runs are given:

```
python -m evaluation.compare_runs runs/20261024_210638 runs/<new_run> [runs/<another_new_run> ...] --out comparison.md
```

Each run folder now also saves its own `progress.csv` (coverage, rooms and error every 30 simulated seconds), which the coverage-plateau metric needs.
It compares the same rows as `docs/results/RESULTS.pdf`: task completed, time to complete, distance and speed, real floor mapped, ATE and drift, RPE, map accuracy/precision/recall against the real scene, frontiers remaining and room entries, per-agent time and real-time factor. Report **before and after together**, and say how many runs each number comes from.

## Order of work
1. F1 to F4: make exploration finish (without this no timing or coverage comparison is meaningful).
2. Profile (F12): find where the 54% of wall time outside the agents goes.
3. F7 to F9: accuracy. 4. F5, F13 to F15: speed. 5. F28, F29: perception and knowledge graph. 6. F25 to F27: external validation.
Update the Status of each fix in `FIXES.md` as it is verified, and keep every run folder you compare against.
