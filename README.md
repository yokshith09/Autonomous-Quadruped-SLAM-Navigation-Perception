# Autonomous Quadruped Multi-Agent SLAM and Exploration Stack

CPU-only 2D SLAM, room segmentation and room-by-room exploration for a legged robot in CoppeliaSim
(`robot_working.ttt`). Seven cooperating agents run in one Python process and talk over a message bus.

Current state: the system has been **measured honestly but not yet improved**. Read these first:
- [`docs/BASELINE_REPORT.pdf`](docs/BASELINE_REPORT.pdf): results, what each value means and why, Good / Improve / Poor classes, comparison with published systems.
- [`docs/BASELINE.md`](docs/BASELINE.md): short summary and diagnosis. [`docs/FIXES.md`](docs/FIXES.md): every fix needed. [`docs/OBJECTIVES.md`](docs/OBJECTIVES.md): what was asked and its status.

## Setup
```powershell
pip install -r requirements.txt
```
Open CoppeliaSim EDU 4.5+ and load `robot_working.ttt` (do not press play; the script starts the simulation).

## Commands (run from the repository root)
| Task | Command |
|---|---|
| Run the robot and measure it | `python main_agentic.py` (`--headless` skips the HUD window, `--max-time S` changes the safety cap, default 3600 simulated seconds) |
| Component ablation by offline replay of that run | `python -m evaluation.ablate runs/<run_id>` |
| Rebuild the run report (after the ablation) | `python -m evaluation.report runs/<run_id>` |
| Build the baseline PDF for a run | `python -m evaluation.baseline_pdf runs/<run_id>` |
| Replay SLAM variants on a recording | `python -m evaluation.slam_replay runs/<run_id>/recording.npz --set icp_yaw_weight=0.2` |
| Tests | `python tests/test_agents.py`, `python tests/test_slam_accuracy.py`, `python tests/test_run_telemetry.py` |

Each run writes `runs/<run_id>/`: `REPORT.md`, `metrics.json`, `agents.csv`, `trajectory.csv` (ground truth and estimate per
step), `manifest.json` (commit, hardware, configuration, metric definitions), `recording.npz` (raw SLAM input, git-ignored),
`artifacts/` (grids, knowledge graph) and `terminal.log`. The terminal shows a progress line every 30 simulated seconds and one
summary at the end. The baseline run is `runs/20261024_210638`.

## How it works
- **Agents** (`agents/`): Perception (depth to body-frame points), SLAM (point-to-line ICP, gyro-led yaw, Manhattan heading
  prior, keyframe loop closure), Mapping (Bayesian log-odds grid, room segmentation), Frontier (next target), Planning (A* on
  known free space), Locomotion (gait, steering, stuck watchdog), Supervisor (knowledge graph, event log).
- **Odometry** is dead-reckoned from the commanded gait plus a simulated noisy gyro. The estimator never reads the simulator
  pose; `evaluation/audit.py` checks this for every report.
- **Map evaluation** keeps two grids: the SLAM grid (estimated pose) and a reference grid built from the same depth rays with the
  true pose. The difference isolates the effect of pose error; it is not an independent floor plan.
- **Known simulation shortcut:** `LizardRobot.steer_heading` turns the body by setting its orientation directly (fix F23).

## Repository layout
```text
main_agentic.py   entry point: wires the agents to CoppeliaSim, runs the mission, records the run
slam.py grid.py planner.py robot.py sensors.py viz.py   algorithm and simulator-interface code
agents/           the seven agents and the message bus
evaluation/       run_telemetry (measurement), report (REPORT.md), baseline_pdf (PDF), ablate (offline ablation),
                  slam_replay, audit (ground-truth independence), accuracy_evaluator (map and trajectory metrics),
                  benchmarks (published third-party numbers with sources)
tests/            formula and algorithm tests
docs/             baseline report, objectives, fixes, panel review documents, architecture diagram
runs/             one folder per measured run (the raw recording stays local)
```
