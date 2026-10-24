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
Status as of baseline run `20261024_210638` (details in `runs/20261024_210638/REPORT.md`).

| # | Requirement | Status | Where / note |
|---|---|---|---|
| R1 | Run normally; results computed and saved automatically | Done | `python main_agentic.py`, `run_telemetry.py` |
| R2 | Overall: completion, time, distance, coverage, stuck/recovery events, bottlenecks | Done | Interventions and collisions are `n/a`: no channel exists for either |
| R3 | Per agent: calls, successes, errors, latency p50/p95, task metric | Done | Section 2a of the report. Per-agent memory is not separable in one process |
| R4 | Component contribution by offline ablation | Partly | Scan matching, loop closure, degeneracy filter, Manhattan prior (`ablate.py`). Mapping, frontier selection and planning contributions are unmeasured |
| R5 | Localization: ATE, RPE, drift, tracking loss, relocalization, with frames and definitions | Done | Section 3. Tracking-loss numbers are diagnostic: rejection reasons are not recorded |
| R6 | Mapping: coverage, accuracy vs reference, throughput, size, incomplete regions | Done | Section 4. The reference grid uses the true pose; it is not an independent floor plan |
| R7 | Compute: CPU, memory, GPU, latency, bottleneck, hardware | Done | GPU present but unused by any code path |
| R8 | Terminal summary (7 parts), occasional progress, JSON/CSV, manifest, raw recording kept | Done | The baseline run still printed the old published-benchmark block; removed afterwards |
| R9 | All numbers dynamic; missing shown as unavailable | Done | |
| R10 | Automated tests of the formulas (synthetic data only) | Done | `tests/test_run_telemetry.py` |
| R11 | Non-blocking, bounded, failure-isolated, safety untouched | Partly | Bounded and isolated; file writes happen after the loop (not a background thread); the in-memory recording buffer grows with run length (peak 3 GB over 30 min) |
| R12 | Validation: right files, units, formulas, duplicates, stale data, overhead, replay consistency | Partly | Overhead is a calibrated estimate, not an A/B run. Replay consistency in the ablation table |
| R13 | Output sections 1-6 as tables with units and explanations | Done | `report.py` -> `REPORT.md` |
| R14 | Phase B: direct and partial competitors with sources and fair-comparison caveats | Pending | Published numbers collected in `benchmarks.py`; not used until the baseline is verified |
| R15 | Phase C: conclusions backed by measurements | Partly | Measured-only interpretation in `docs/BASELINE.md`; external parts pending |

## Requests made during the work, and status
| Request | Status |
|---|---|
| Run live and let the robot explore every room without stopping | **Not achieved**: ended at the 1800 s cap with 6/8 rooms; see `docs/BASELINE.md` for the cause |
| Diagnose why it revisits rooms and cells | Done (diagnosis); fixes not applied yet |
| Explain agent efficiency and whether changes alter results | Answered; profiling pending |
| Commit baseline snapshot, finish baseline, update docs, clean the repo | Done in this change |

## Next steps (in order)
1. Profile one short run (`cProfile`) to find where the 3074 s outside the agents goes.
2. Apply the four exploration fixes (see `docs/BASELINE.md`), run uncapped, compare against this baseline.
3. Speed-ups that must not change results; verify by replaying the same recording.
4. Phase B comparison against the verified baseline.
