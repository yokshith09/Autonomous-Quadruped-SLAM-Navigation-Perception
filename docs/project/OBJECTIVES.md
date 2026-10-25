# Objectives and status

## Objective
Measure the **actual** performance of the existing robot and software from real execution data (no assumed or hard-coded
numbers): overall, per agent, localization and mapping separately, and compute cost. Establish an honest baseline
**before** optimizing anything. Only after the measurements are verified, pick fair external systems and benchmarks to
compare against.

## Rules that apply to every step
- Inspect before modifying; reuse the existing architecture and tools; do not rewrite the system.
- Every reported value is computed from the current run; missing values are shown as `n/a (reason)`, never as zero.
- Distinguish measured result, estimate, diagnostic indicator, unavailable. No repeatability claims from one run.
- Measurement must not interfere with the control loop or bypass any safety mechanism; ablations are offline replays only.
- Do not optimize to improve numbers; do not finalize competitor comparisons before the baseline is verified.

## Requirements and status
Status as of baseline run `20261024_210638`. Results: `docs/results/RESULTS.pdf` (shareable), `docs/results/DIAGNOSTIC_REPORT.pdf` (internal), `docs/project/BASELINE.md`, `runs/20261024_210638/REPORT.md` (raw). Fixes: `docs/project/FIXES.md`.

| # | Requirement | Status | Where / note |
|---|---|---|---|
| R1 | Run normally; results computed and saved automatically | Done | `python main_agentic.py`, `evaluation/run_telemetry.py` |
| R2 | Overall: completion, time, distance, coverage, stuck/recovery events, bottlenecks | Done | Interventions and collisions are `n/a`: no channel exists for either |
| R3 | Per agent: calls, successes, errors, latency p50/p95, task metric | Done | Section 2a of the report. Per-agent memory is not separable in one process |
| R4 | Component contribution by offline ablation | Partly | Scan matching, loop closure, degeneracy filter, Manhattan prior (`evaluation/ablate.py`). Mapping, frontier selection and planning contributions are unmeasured |
| R5 | Localization: ATE, RPE, drift, tracking loss, relocalization, with frames and definitions | Done | Section 3. Tracking-loss numbers are diagnostic: rejection reasons are not recorded |
| R6 | Mapping: coverage, accuracy vs reference, throughput, size, incomplete regions | Done | Against the true-pose reference grid and, independently, against the real scene geometry read from CoppeliaSim (true coverage, map accuracy, object detection) |
| R7 | Compute: CPU, memory, GPU, latency, bottleneck, hardware | Done | GPU present but unused by any code path |
| R8 | Terminal summary (7 parts), occasional progress, JSON/CSV, manifest, raw recording kept | Done | The baseline run still printed the old published-benchmark block; removed afterwards |
| R9 | All numbers dynamic; missing shown as unavailable | Done | |
| R10 | Automated tests of the formulas (synthetic data only) | Done | `tests/test_run_telemetry.py` |
| R11 | Non-blocking, bounded, failure-isolated, safety untouched | Partly | Bounded and isolated; file writes happen after the loop (not a background thread); the in-memory recording buffer grows with run length (peak 3 GB over 30 min) |
| R12 | Validation: right files, units, formulas, duplicates, stale data, overhead, replay consistency | Done, one gap | Replay matches the live run to 0.01 cm and is deterministic; overhead is a calibrated estimate, not an A/B run (F21) |
| R13 | Output sections 1-6 as tables with units and explanations | Done | `evaluation/report.py` -> `REPORT.md`; `evaluation/baseline_pdf.py` -> PDF with classes |
| R14 | Phase B: direct and partial competitors with sources and fair-comparison caveats | Done (partial matches only) | PDF section 3. No published system is directly comparable; closest are A1 SLAM, the ANYmal D evaluation and FARE. Key numbers re-checked against the sources on 2026-10-25. The fair head-to-head experiments are not done (F25-F27) |
| R15 | Phase C: conclusions backed by measurements | Done | `docs/project/BASELINE.md` and PDF sections 1, 2 and 5 |

## Requests made during the work, and status
| Request | Status |
|---|---|
| Run live and let the robot explore every room without stopping | **Not achieved**: ended at the 1800 s cap with 6/8 rooms; see `docs/project/BASELINE.md` for the cause |
| Diagnose why it revisits rooms and cells | Done (diagnosis); fixes not applied yet |
| Explain agent efficiency and whether changes alter results | Answered; profiling pending |
| Commit baseline snapshot, finish baseline, update docs, clean the repo | Done |
| Draft results, compare with published systems, PDF with explanations and good/improve/poor classes, fix register, organize the repo | Done (`docs/results/DIAGNOSTIC_REPORT.pdf`, `docs/project/FIXES.md`, `evaluation/` package) |
| Compare SLAM as a whole (pose + map), and a separate shareable PDF with results and comparison only, each value defined, no explanations | Done (`docs/results/RESULTS.pdf`) |
| Measure only, no fixes; plain results for the IEEE draft; score against the real scene; documents for run 1 only | Done (`docs/results/RESULTS.pdf`, `docs/project/BASELINE.md`) |
| Complete formal project report (what, inputs, outputs, processing, why, methods, all results, comparisons, evaluation, graphs) and a restructured README with quick start | Done (`docs/results/PROJECT_REPORT.pdf`, `README.md`) |
| Full market comparison and a like-for-like reference for later dataset runs | Done (RESULTS.pdf sections 13-14): 6 commercial systems; TUM RGB-D Pioneer reference, ours not yet run |

## Next steps (in order; details and acceptance checks in `docs/project/FIXES.md`)
1. Profile one short run (`cProfile`) to find where the 3074 s outside the agents goes (F12).
2. Apply the exploration fixes F1-F4, run uncapped, compare against this baseline.
3. Speed-ups (F12-F15) that must not change results; verify by replaying the baseline recording.
4. Fair comparison experiments F25-F27, then repeat runs (F18).
